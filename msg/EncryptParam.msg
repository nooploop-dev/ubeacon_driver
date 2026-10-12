bool success
bool too_frequent         # 写请求间隔过短被拒绝
uint8[16] key             # 成功时为新密钥，失败时全零
