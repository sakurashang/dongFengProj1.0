# 进行信息解析的测试
import time

import component_0.classOfDF2204 as classOfDF2204

tractor_recv = classOfDF2204.Tractor("recv")
while True:
    print(tractor_recv.recv_msg_full_dbc_id())
    #time.sleep(0.1)
