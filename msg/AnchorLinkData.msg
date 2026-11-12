int64 local_time_us
uint16 src                      # 主信标，mccp 发送方
uint16 dst                      # 跟随 src 发出 sccp 的信标
float32 rx_rate                 # dst 近期收到 src mccp 的收包率 0~1
bool is_dis_valid               # false 时 dis 与 dis_std 为量程边界值
uint8 area_id
float32 dis                     # dst 与 src 的近期测距中值, m
float32 dis_std                 # m
float32 uwb_clock_offset_ppm    # dst 测得的与 src 之间的频偏
float32 mcu_clock_offset_ppm
