float32 expect_z                # m
float32 z_noise                 # m
uint8 smooth_window             # 0~5
float32[3] max_acceleration     # m/s^2
bool output_tag_pos
bool output_anchor_packet
bool output_anchor_pos
bool output_anchor_link_data
bool output_anchor_signal
bool output_anchor_ddoa
bool output_tag_pos_even_error
bool output_anchor_link_status
bool output_imu
bool output_attitude
bool output_dis
bool output_pdoa
uint8 sniff_duty_cycle          # 百分比 1~100
uint8 update_interval_max       # s
uint16 reset_interval           # s
uint8 min_sync_anchor_count
uint8 min_sniff_duty_cycle      # 百分比 0~100
uint8 static_map_id             # 0 表示不固定
