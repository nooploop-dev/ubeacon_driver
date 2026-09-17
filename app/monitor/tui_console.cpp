#include "tui_console.hpp"

#include <algorithm>
#include <cstdio>
#include <deque>
#include <ftxui/component/app.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/component_options.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/mouse.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/box.hpp>
#include <ftxui/screen/color.hpp>
#include <ftxui/screen/terminal.hpp>
#include <mutex>
#include <spdlog/sinks/base_sink.h>
#include <string>
#include <utility>

namespace {

// 滚动区只显示最新的一屏，缓存略多于一屏即可，完整日志在日志文件里
constexpr std::size_t LINES_MAX = 500;
constexpr std::size_t HISTORY_MAX = 100;
// 鼠标滚轮每格滚动的行数
constexpr std::size_t WHEEL_STEP = 3;

struct Line {
  spdlog::level::level_enum level;
  std::string text;
  // 与spdlog彩色sink一样只给这一段(默认格式里的级别名)上色，其余用默认色
  std::size_t color_begin;
  std::size_t color_end;
};

enum class Phase { PENDING, RUNNING, FINISHED };

// 以下状态在日志线程与界面线程间共享，由g_mutex保护
std::mutex g_mutex;
Phase g_phase = Phase::PENDING;
bool g_exit_requested = false;
ftxui::App *g_app = nullptr;
std::deque<Line> g_lines;
// 暂停刷新期间界面渲染的是暂停那一刻的画面副本，g_lines照常接收新日志；
// g_scroll为副本向上翻看的行数(按折行后的屏幕行计)
bool g_paused = false;
std::deque<Line> g_frozen;
std::size_t g_scroll = 0;
// 由向上翻看引起的暂停在翻回底部时自动恢复；help、按钮等明确要求的暂停只能明确恢复
bool g_paused_by_scroll = false;

// 配色沿用spdlog的stdout_color_sink
ftxui::Decorator level_style(spdlog::level::level_enum level) {
  using namespace ftxui;
  switch (level) {
  case spdlog::level::trace:
    return color(Color::White);
  case spdlog::level::debug:
    return color(Color::Cyan);
  case spdlog::level::info:
    return color(Color::Green);
  case spdlog::level::warn:
    return color(Color::Yellow) | bold;
  case spdlog::level::err:
    return color(Color::Red) | bold;
  case spdlog::level::critical:
    return bgcolor(Color::Red) | color(Color::White) | bold;
  default:
    return nothing;
  }
}

class TuiSink : public spdlog::sinks::base_sink<std::mutex> {
protected:
  void sink_it_(const spdlog::details::log_msg &msg) override {
    spdlog::memory_buf_t formatted;
    formatter_->format(msg, formatted);
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_phase == Phase::FINISHED) {
      std::fwrite(formatted.data(), 1, formatted.size(), stdout);
      return;
    }
    std::string text(formatted.data(), formatted.size());
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
      text.pop_back();
    }
    // 着色范围由formatter按%^ %$标记回填到msg里
    const std::size_t color_end = std::min(msg.color_range_end, text.size());
    const std::size_t color_begin = std::min(msg.color_range_start, color_end);
    g_lines.push_back({msg.level, std::move(text), color_begin, color_end});
    if (g_lines.size() > LINES_MAX) {
      g_lines.pop_front();
    }
    // 闭包任务不会触发重绘，只有事件会
    if (g_app) {
      g_app->PostEvent(ftxui::Event::Custom);
    }
  }

  void flush_() override { std::fflush(stdout); }
};

// 日志区的可用行数：终端高度扣掉状态行与输入行
std::size_t log_area_rows() {
  return (std::size_t)std::max(1, ftxui::Terminal::Size().dimy - 2);
}

// 一行日志折行后占的屏幕行数
std::size_t row_count(const Line &line, std::size_t width) {
  return std::max<std::size_t>(1, (line.text.size() + width - 1) / width);
}

// 渲染日志的[begin, end)字节区间，落在着色范围内的部分按级别上色
ftxui::Element render_row(const Line &line, std::size_t begin,
                          std::size_t end) {
  using namespace ftxui;
  Elements parts;
  auto piece = [&](std::size_t from, std::size_t to, bool colored) {
    if (from >= to) {
      return;
    }
    Element part = text(line.text.substr(from, to - from));
    parts.push_back(colored ? part | level_style(line.level) : part);
  };
  piece(begin, std::min(end, line.color_begin), false);
  piece(std::max(begin, line.color_begin), std::min(end, line.color_end), true);
  piece(std::max(begin, line.color_end), end, false);
  return hbox(std::move(parts));
}

// 日志区加其下的状态行。日志从最新一行往前凑够一屏，长行按终端宽度折行
// (日志为ASCII，按字节切分)；界面钉在底部，更早的行不必渲染
ftxui::Element render_log_area() {
  using namespace ftxui;
  const std::size_t width = (std::size_t)std::max(1, Terminal::Size().dimx);
  const std::size_t area = log_area_rows();
  Elements rows;
  std::lock_guard<std::mutex> lock(g_mutex);
  const std::deque<Line> &lines = g_paused ? g_frozen : g_lines;
  std::size_t skip = 0;
  if (g_paused) {
    std::size_t total = 0;
    for (const auto &line : lines) {
      total += row_count(line, width);
    }
    g_scroll = std::min(g_scroll, total > area ? total - area : 0);
    skip = g_scroll;
  }
  for (auto it = lines.rbegin(); it != lines.rend() && rows.size() < area;
       ++it) {
    for (std::size_t i = row_count(*it, width);
         i-- > 0 && rows.size() < area;) {
      if (skip > 0) {
        skip--;
        continue;
      }
      rows.push_back(render_row(*it, i * width,
                                std::min(it->text.size(), (i + 1) * width)));
    }
  }
  std::reverse(rows.begin(), rows.end());
  Element status =
      !g_paused ? separator()
      : g_paused_by_scroll
          ? hbox({text(" scrolled back | scroll to bottom, Esc or Enter to "
                       "resume"),
                  filler()}) |
                inverted
          : hbox({text(" paused | scroll: wheel, PageUp/PageDown | resume: "
                       "Esc, Enter, click [ resume ]"),
                  filler()}) |
                inverted;
  return vbox({
             vbox(std::move(rows)) | focusPositionRelative(0.f, 1.f) | yframe |
                 flex,
             std::move(status),
         }) |
         flex;
}

// 以下函数须持有g_mutex
void pause_locked(bool by_scroll) {
  if (!g_paused) {
    g_frozen = g_lines;
    g_paused = true;
    g_paused_by_scroll = by_scroll;
  }
}

void resume_locked() {
  g_paused = false;
  g_frozen.clear();
  g_scroll = 0;
}

// 向上翻看即暂停
void scroll_up_locked(std::size_t rows) {
  pause_locked(true);
  g_scroll += rows;
}

void scroll_down_locked(std::size_t rows) {
  g_scroll -= std::min(g_scroll, rows);
  if (g_scroll == 0 && g_paused_by_scroll) {
    resume_locked();
  }
}

bool is_paused() {
  std::lock_guard<std::mutex> lock(g_mutex);
  return g_paused;
}

std::string trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

} // namespace

namespace tui_console {

spdlog::sink_ptr make_sink() { return std::make_shared<TuiSink>(); }

void run(on_input_f on_input) {
  using namespace ftxui;
  auto app = App::Fullscreen();
  // Ctrl-C由下面的CatchEvent处理并正常退出主循环，不让FTXUI在退出后重新抛出SIGINT
  app.ForceHandleCtrlC(false);
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_exit_requested) {
      for (const auto &line : g_lines) {
        std::printf("%s\n", line.text.c_str());
      }
      g_lines.clear();
      g_phase = Phase::FINISHED;
      return;
    }
    g_phase = Phase::RUNNING;
    g_app = &app;
  }

  std::string content;
  int cursor = 0;
  std::deque<std::string> history;
  std::size_t history_pos = 0; // 等于history.size()表示正在编辑新输入

  InputOption option;
  option.multiline = false;
  option.cursor_position = &cursor;
  option.placeholder = "command, Enter to send, 'help' for usage";
  option.on_enter = [&] {
    // 回车既恢复刷新也提交命令，这样命令的输出能和实时日志一起看到
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      resume_locked();
    }
    const std::string line = trim(content);
    content.clear();
    cursor = 0;
    if (line.empty()) {
      return;
    }
    history.push_back(line);
    if (history.size() > HISTORY_MAX) {
      history.pop_front();
    }
    history_pos = history.size();
    on_input(line);
  };
  Component input = Input(&content, option);

  // 输入行左侧的暂停/继续按钮。不用FTXUI的Button，它被点击后会夺走焦点，
  // 用户还得再点回输入框；这里只记录按钮渲染位置并自行判断点击
  Box button_box;
  bool button_hover = false;

  // Windows传统控制台上首帧会错位一行，之后的帧按相对位置重绘就正常，
  // 所以首帧渲染完立即再补一帧
  bool first_frame = true;
  Component ui = Renderer(input, [&] {
    if (first_frame) {
      first_frame = false;
      app.PostEvent(Event::Custom);
    }
    Element button = text(is_paused() ? "[ resume ]" : "[ pause ]") | bold |
                     reflect(button_box);
    if (button_hover) {
      button = button | inverted;
    }
    return vbox({
        render_log_area(),
        hbox({std::move(button), text(" > "), input->Render()}),
    });
  });
  ui = CatchEvent(ui, [&](Event event) {
    // 鼠标只用于按钮和滚轮，不交给输入框
    if (event.is_mouse()) {
      const Mouse &mouse = event.mouse();
      button_hover = button_box.Contain(mouse.x, mouse.y);
      if (mouse.motion != Mouse::Pressed) {
        return true;
      }
      std::lock_guard<std::mutex> lock(g_mutex);
      if (mouse.button == Mouse::WheelUp) {
        scroll_up_locked(WHEEL_STEP);
      } else if (mouse.button == Mouse::WheelDown) {
        scroll_down_locked(WHEEL_STEP);
      } else if (mouse.button == Mouse::Left && button_hover) {
        if (g_paused) {
          resume_locked();
        } else {
          pause_locked(false);
        }
      }
      return true;
    }
    if (event == Event::CtrlC) {
      app.Exit();
      return true;
    }
    if (event == Event::Escape) {
      std::lock_guard<std::mutex> lock(g_mutex);
      resume_locked();
      return true;
    }
    // 翻页时保留一行上下文
    if (event == Event::PageUp || event == Event::PageDown) {
      const std::size_t step = std::max<std::size_t>(1, log_area_rows() - 1);
      std::lock_guard<std::mutex> lock(g_mutex);
      if (event == Event::PageUp) {
        scroll_up_locked(step);
      } else {
        scroll_down_locked(step);
      }
      return true;
    }
    if (event == Event::ArrowUp || event == Event::ArrowDown) {
      const std::size_t target =
          event == Event::ArrowUp
              ? (history_pos > 0 ? history_pos - 1 : history_pos)
              : std::min(history_pos + 1, history.size());
      if (target != history_pos) {
        history_pos = target;
        content = history_pos < history.size() ? history[history_pos] : "";
        cursor = (int)content.size();
      }
      return true;
    }
    return false;
  });

  app.Loop(ui);

  std::lock_guard<std::mutex> lock(g_mutex);
  g_app = nullptr;
  g_phase = Phase::FINISHED;
}

void pause() {
  std::lock_guard<std::mutex> lock(g_mutex);
  pause_locked(false);
  if (g_app) {
    g_app->PostEvent(ftxui::Event::Custom);
  }
}

void request_exit() {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_exit_requested = true;
  if (g_app) {
    g_app->Exit();
  }
}

} // namespace tui_console
