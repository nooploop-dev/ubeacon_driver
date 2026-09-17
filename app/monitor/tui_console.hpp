#pragma once

#include <functional>
#include <spdlog/sinks/sink.h>
#include <string>

// 终端交互界面：上方持续滚动显示日志，最底一行接收用户输入，回车提交
namespace tui_console {

// 用户回车提交的一行输入(已去掉首尾空白，非空)，在界面线程回调
using on_input_f = std::function<void(const std::string &line)>;

// 日志sink：run()结束前写入界面滚动区(含run()之前的日志)，之后直接写到stdout
spdlog::sink_ptr make_sink();

// 阻塞运行界面直到用户退出或request_exit()，须在主线程调用
void run(on_input_f on_input);

// 暂停日志刷新以便阅读当前画面(新日志照常记录，只是不上屏)，
// 底部提示用户按Esc或回车恢复。可在任意线程调用
void pause();

// 请求界面退出，可在任意线程调用，也可在run()之前调用
void request_exit();

} // namespace tui_console
