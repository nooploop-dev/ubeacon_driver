# location_mode 取值，决定 begin_time_us 与 rx_rate 的含义，见 ub_location_mode_e
uint8 LOCATION_MODE_TDOA=0
uint8 LOCATION_MODE_TWR=1
uint8 LOCATION_MODE_RSSI=2
int64 begin_time_us
uint8 location_mode
Anchor2TagSignalItem[] datas
