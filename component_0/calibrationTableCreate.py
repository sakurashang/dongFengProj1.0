# dbc文件解析类
# 进行dbc的解析
# 四个消息字段：发送消息message('NAVI', 0x18ff911c, True, 8, None)、message('NAVI2', 0x18ff921c, True, 8, None)、
# message('NAVI3', 0x18ff931c, True, 8, None)，接收消息message('VC6', 0x18ffa127, True, 8, None)
# 要实现消息的解析和发送

import cantools
import os
import can
import time
import numpy as np
import threading
import component_0.classOfIMU_CGI610 as classOfIMU_CGI610
from component_0 import classOfTractorInfoRead, classOfDataRecord, calibrationDataRecord


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
        if msg.frame_id == 0x21C:  #NAVI2
            msg_frame_id = 0x18FF921C
        if msg.frame_id == 0x11C:   #NAVI
            msg_frame_id = 0x18FF911C
        if msg.frame_id == 0x31C:   #NAVI3
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
    __can_msg_trans = CANMsgTrans("component_0/dongFeng2204_2.dbc")

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

    def send_msg(self, msg_name, cmd_list):
        # 把can_msg的生成放进来
        can_msg = self.__can_msg_trans.can_msg_produce(msg_name, cmd_list)
        if msg_name == "NAVI":
            self.task_NAVI.modify_data(can_msg)
        if msg_name == "NAVI2":
            self.task_NAVI2.modify_data(can_msg)
        if msg_name == "NAVI3":
            self.task_NAVI3.modify_data(can_msg)
        # if msg_name == "VC6":
        #     self.task_VC6.modify_data(can_msg)

    def recv_msg_full_dbc_id(self):
        # 等待接收，直到收到信息才会返回。返回dbc里的两个反馈，其他的不反馈。
        get_data = self.can_bus.recv()
        while 1:
            if get_data.arbitration_id == 0x18FFA127:  # vcan会丢弃前面的，完整的是0x18FFA127,这个是扩展帧的原因
                # print("收到一条信息")
                # motion_info = self.__can_msg_trans.db.decode_message(get_data.arbitration_id, get_data.data)
                motion_info = self.__can_msg_trans.db.decode_message(0x18FFA127,
                                                                     get_data.data)  # eg:{'gearnum': 14, 'IO_vVeh': 51.1328125, 'Rx_valuePositionSensorFrmEHR': 68, 'Rx_valueSumofDraftFrmEHR': 85, 'Rx_staErr1': 0, 'Rx_staErr2': 1, 'Rx_rawwheelangle': 19}
                # 增加明文信息
                motion_dict = {"档位": motion_info["gearnum"], "车速": motion_info["IO_vVeh"],
                               "提升器高度": motion_info["Rx_valuePositionSensorFrmEHR"],
                               "提升器合力": motion_info["Rx_valueSumofDraftFrmEHR"],
                               "车不动原因": motion_info["Rx_staErr1"], "前轮转角": motion_info["Rx_rawwheelangle"]}
                # print(motion_dict)
                return [0x18FFA127, motion_dict]
            elif get_data.arbitration_id == 0x18FFA227:
                security_info = self.__can_msg_trans.db.decode_message(0x18FFA227,
                                                                       get_data.data)  # eg:{'Rx_staBreak': 17, 'Rx_trqEngActFrmEMS': -91, 'Rx_staWheelmotorError': 51, 'Rx_rawSteerlPos': 21828}
                security_dict = {"刹车状态": security_info["Rx_staBreak"], "发动机扭矩百分比": security_info["Rx_trqEngActFrmEMS"],
                                 "方向盘故障": security_info["Rx_staWheelmotorError"],
                                 "方向盘位置传感器值": security_info["Rx_rawSteerlPos"]}
                # print(security_dict)
                return [0x18FFA227, security_dict]
            else:
                # print("收到其他arbitration_id：", get_data.arbitration_id)
                get_data = self.can_bus.recv()
                # return [get_data.arbitration_id, get_data]


class VCUCmd(object):
    """
        本类进行拖拉机整车（横纵向运动、提升器、PTO、液压输出）控制指令的封装，按特定功能来设计函数。
    """
    # 类成员变量
    #可能被用到的参数
    #1、车速（0-60） 2、前进（1前进，0空档，-1后退）4、导航模式（0、手动模式 1、扭矩2、自动模式）
    __NAVI_cmd_list = [0.0, 0, 0, 2, 0, 0]
    #1、档位选择（3抵挡、11高档）
    __NAVI2_cmd_list = [3, 0, 250, 250, 250]
    #5、前轮转角 7、请求刹车(0不刹车、1、刹车 ，扭矩模式时起作用） 8控制扭矩（扭矩模式时起作用）
    __NAVI3_cmd_list = [0, 0, 0, 0, 0, 0, 0, 0]

    # 按实现不同功能设计类方法
    def __init__(self, tractor):
        """
            进行变量的初始化
        """
        self.tractor = tractor

    # 用扭矩进行控制
    def send_motion_ctrl_msg(self, Drive_Mode_Req, ShiftLevel_Req, Steering_Wheel_Angle, Torque, Brk_En):
        """
            发送车辆横纵向控制指令。后期把参数换成英文。
        :param Drive_Mode_Req:0:手动模式、1：扭矩受导航请求控制、2：自动模式
        :param ShiftLevel_Req:字符串类型，分为“前进高档”、“前进低档”、“后退高档”、“后退低档”、“空挡”
        :param Steering_Wheel_Angle:
        :param Target_Speed:
        :param Brk_En:
        :return:
        """
        # 首先检查驾驶模式
        if Drive_Mode_Req == 0:  # 手动模式
            self.__NAVI_cmd_list[3] = 0
            self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)
            # return "手动模式" # debug20211001注释掉本句，想在手动驾驶的时候进行方向盘的控制

        # TODO:检查下有车速情况下发空挡是什么反应
        # 刹车的优先级最高放在这里，刹车后就返回(刹车转换成速度为0、不管转角)
        if Brk_En == 1:
            self.__NAVI_cmd_list[0] = 0  # 修改车速(后期看是否要挂空挡)
            # 发送指令直接返回
            self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)
            return "程序刹车制动"

        # 修改变量序列
        self.__NAVI_cmd_list3[7] = Torque  # 修改扭矩

        if ShiftLevel_Req == "前进高档":  # 测试下是否高效
            self.__NAVI_cmd_list[1] = 1  # 修改档位(前进1、空挡0、后退1)
            # self.__NAVI2_cmd_list[2]
            self.__NAVI2_cmd_list[0] = 11  # 高低档（高档11，低档3）
        elif ShiftLevel_Req == "前进低档":
            self.__NAVI_cmd_list[1] = 1
            self.__NAVI2_cmd_list[0] = 3
        elif ShiftLevel_Req == "后退高档":
            self.__NAVI_cmd_list[1] = -1
            self.__NAVI2_cmd_list[0] = 11
        elif ShiftLevel_Req == "后退低档":
            self.__NAVI_cmd_list[1] = -1
            self.__NAVI2_cmd_list[0] = 3
        else:
            # “空挡”
            self.__NAVI_cmd_list[1] = 0

        # self.__NAVI_cmd_list[2]   # 修改提升器指令
        self.__NAVI_cmd_list[3] = Drive_Mode_Req  # 修改导航模式(0:手动模式、1：扭矩受导航请求控制、2：自动模式)
        # self.__NAVI_cmd_list[4]   # 修改点火熄火信号
        # self.__NAVI_cmd_list[5]   # 修改PTO指令

        # 前轮转角(后期读取前轮实际转角进行误差消除)
        self.__NAVI3_cmd_list[4] = Steering_Wheel_Angle  # 前轮目标转角

        # 发送修改的变量
        print('self.__NAVI_cmd_list', self.__NAVI_cmd_list)
        self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)

        self.tractor.send_msg("NAVI2", self.__NAVI2_cmd_list)
        self.tractor.send_msg("NAVI3", self.__NAVI3_cmd_list)
        return "程序控制指令发送成功"


class SafetyGuarantee(object):
    """提供安全保障功能"""

    # 读取刹车状态、方向盘故障信息，然后切换成手动驾驶。使用多线程的方式实现。后期完成
    def __init__(self, tractor_recv):
        self.__tractor_recv = tractor_recv
        # self.brake_flag = brake_flag     # 默认为0不刹车
        global brake_flag

    def monitor_brake(self):
        # 暂时先这样
        while 1:
            recv = self.__tractor_recv.recv_msg_full_dbc_id()
            if recv[0] == 0x18FFA227 and recv[1]["刹车状态"] == 1:
                # 有刹车
                print("在刹车")
                brake_flag = 1
                # 执行速度为0、抬机具，δ时间后切换成空挡，判断切换成空挡后换成手动模式
                return brake_flag


if __name__ == "__main__":
    NAVI_cmd_list = [3.0, 1, 0, 2, 0, 0]  # v=3,前进挡，测试横纵向连通性
    # NAVI_cmd_list = [0.0, 0, 0, 2, 0, 0]
    NAVI2_cmd_list = [3, 0, 250, 250, 250]
    NAVI3_cmd_list = [0, 0, 0, 0, 0, 0, 0, 0]

    # 20210705 更新:测试横向跟踪性能
    tractor = Tractor("send")
    imu = classOfIMU_CGI610.Imu()
    time.sleep(3)
    vcu_cmd = VCUCmd(tractor)
    path = r'calibrationData/'
    record = calibrationDataRecord.DataRecord(path + "标定行驶时数据" +
                                          time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) +
                                          ".csv")
    #获取行驶中的数据
    inte_navi_info = imu.stateOfCar()
    tractor_info_read = classOfTractorInfoRead.TractorInfoRead()
    tractor_info_dict = tractor_info_read.vehicle_state_info_return()
    msg = [time.time(), inte_navi_info, tractor_info_read]

    # 标定表 标定正向扭矩
    Torque = 1
    vcu_cmd.send_motion_ctrl_msg(1, "前进高档", 0, Torque, 0)
    # 标定表所需要的数据，当前的扭矩数值，速度、加速度、时间、经纬度、高程、Utm、
    record.data_record(msg)

    #标定表 标定制动的扭矩
    Torque = 100 #用最大值
    vcu_cmd.send_motion_ctrl_msg(1, "前进高档", 0, Torque, 0)
    v_navi = inte_navi_info[19]     # m/s
    v_navi = v_navi * 3.6
    v_trac = tractor_info_dict["车速"]  # 单位km/h 应该是
    if v_trac > 30 :
        Torque = 1
        vcu_cmd.send_motion_ctrl_msg(1, "前进高档", 0, Torque, 0)
        record.data_record(msg)


