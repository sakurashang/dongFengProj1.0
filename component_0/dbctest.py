import cantools
import os
import can
import time

class CANMsgTrans(object):
    db = 0

    def __init__(self, dbc_name):
        self.dbc_name = dbc_name
        dbc_path = os.path.join(os.getcwd(), dbc_name)
        self.db = cantools.database.load_file(dbc_path)
        # print(self.db)

    def can_msg_produce(self, msg_name, msg_list):
        msg = self.db.get_message_by_name(msg_name)
        # 消息发送初始化
        msg_data = {}
        j = 0
        for i in msg.signal_tree:
            msg_data[i] = msg_list[j]
            j = j + 1
        print(msg_data)
        data = msg.encode(msg_data)
        # message = can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False)
        msg_frame_id = msg.frame_id
        print(msg.frame_id)
        if msg.frame_id == 0x21C:
            msg_frame_id = 0x18FF921C
            print("=======")
        if msg.frame_id == 0x11C:
            msg_frame_id = 0x18FF911C
        if msg.frame_id == 0x31C:
            msg_frame_id = 0x18FF931C
        message = can.Message(arbitration_id=msg_frame_id, data=data, is_extended_id=True)
        return message

class Tractor(object):
    __NAVI_msg_list = [0.0, 0, 0, 2, 0, 0]
    __NAVI2_msg_list = [3, 0, 250, 250, 250]
    # __NAVI3_msg_list = [0, 0, 0, 0]    # 第一版协议
    __NAVI3_msg_list = [0, 0, 0, 0, 0, 0, 0, 0]  # debug 20210930 已修改完成
    __VC6_msg_list = [0, 0, 0, 0, 0, 0]
    # __can_msg_trans = CANMsgTrans("/home/wen/PycharmProjects/dongFengProj_1.0/component_0/dongFeng2204_2.dbc")
    __can_msg_trans = CANMsgTrans("./dongFeng2204_2.dbc")

    # __msg = can_msg_trans.can_msg_produce("NAVI", self.__NAVI_msg_list)

    def __init__(self, flag):
        # 新增权限赋予
        # print(os.system("./component_0/modprobe_peak_usb_forDongFeng2204"))  # 这样会不会使得成为单例模式？
        # print(os.system("./component_0/modprobe_vcan"))    # 测试时候使用

        self.can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=250000)
        if flag == "send":  # 当前是发送消息的功能
            self.task_NAVI = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NAVI", self.__NAVI_msg_list), period=0.1, duration=None,
                store_task=True)  # 一个task值对应一个can.Message.arbitration_id
            # task.modify_data(msg)

            self.task_NAVI2 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NAVI2", self.__NAVI2_msg_list), period=0.1,
                duration=None, store_task=True)
            print('debug navi2')
            self.task_NAVI3 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NAVI3", self.__NAVI3_msg_list), period=0.1,
                duration=None, store_task=True)
            # self.task_VC6 = self.can_bus.send_periodic(self.__can_msg_trans.can_msg_produce("VC6", self.__VC6_msg_list),
            #                                            period=0.1,
            #                                            duration=None, store_task=True)
        else:  # 当前是接受消息的功能
            pass


if __name__ == "__main__":
    NAVI_cmd_list = [3.0, 1, 0, 2, 0, 0]  # v=3,前进挡，测试横纵向连通性
    # NAVI_cmd_list = [0.0, 0, 0, 2, 0, 0]
    NAVI2_cmd_list = [3, 0, 250, 250, 250]
    NAVI3_cmd_list = [0, 0, 0, 0, 0, 0, 0, 0]

    # 20210705 更新:测试横向跟踪性能
    tractor = Tractor("send")
