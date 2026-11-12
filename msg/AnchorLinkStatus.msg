int64 local_time_us
uint16 src              # 广播该状态的信标
uint16[] wait_syncs     # src 应监听但尚未同步上的信标，先 location_link 后 mesh_link
