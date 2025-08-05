import json
import os
import time
from queue import Queue
from threading import Lock
import math, h5py
import cv2
import numpy as np
from tqdm import tqdm
from pyorbbecsdk import *
from tcp_tx import PersistentClient
from utils import frame_to_bgr_image
import random
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QLineEdit,QPushButton, QLabel, QVBoxLayout, QFormLayout,QProgressBar,QHBoxLayout,QMessageBox
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer,QThread, pyqtSignal, QMutex
import threading
from camera_hot_plug import CAMERA_HOT_PLUG
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLineEdit, QPushButton, QProgressBar, QSizePolicy
)
today_str = datetime.today().strftime("%m-%d")  # 输出例如 "07-28"

frames_queue_lock = Lock()

# Configuration settings
MAX_DEVICES = 3
MAX_QUEUE_SIZE = 2
save_points_dir = os.path.join(os.getcwd(), "point_clouds")
save_depth_image_dir = os.path.join(os.getcwd(), "depth_images")
save_color_image_dir = os.path.join(os.getcwd(), "color_images")
action_play:bool = False
# Load config file for multiple devices
config_file_path = os.path.join(os.path.dirname(__file__), "../config/multi_device_sync_config.json")
multi_device_sync_config = {}
camera_names = ['left_wrist','top','right_wrist']
save_signal = False
traj_signal = 1
class   GPCONTROL(QThread):
    error_signal = pyqtSignal(object)
    gp_control_state_signal = pyqtSignal(object)  # 反馈信号，用于 UI 连接
    def __init__(self,parent=None, DEFAULT_SERIAL_PORTS = ("/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2")):
        super().__init__(parent)
        self.state_flag = 128  # 夹爪状态: 0=关, 1=半开, 2=开
        self.running = True  # 控制线程运行
        self.control_command = ""  # 当前控制命令
        self.DEFAULT_SERIAL_PORTS = DEFAULT_SERIAL_PORTS
        self.BAUD_RATE = 50000
        self.min_data = b'\x00\x00\xFF\xFF\xFF\xFF\x00\x00'
        self.max_data = b'\x00\xFF\xFF\xFF\xFF\xFF\x00\x00'
        self.ser = self.open_serial()
        self.is_sending = False
        self.state_data_1=128
        self.state_data_2=0
        self.task_complete = False
        self.is_configured = False  # 配置标志位
        set_can1 = b'\x49\x3B\x42\x57\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'
        start_can = b'\x49\x3B\x44\x57\x01\x00\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'
        set_can0 = b'\x49\x3B\x42\x57\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x45\x2E'
        self.send_data(set_can1)  # 发送配置指令
        self.send_data(set_can0)
        self.read_data()
        self.send_data(start_can)  # 启动 CAN 通道
        self.read_data()    
    def run(self):
        state =None
        while self.running:
            # print(f"control:{self.state_flag}")
            # 1. 根据当前的 `state_flag` 设定控制命令
            # if self.state_flag == 0:
            #     self.control_command = "CLOSE_GRIPPER"
            #     state = self.close_gp()
            #     # print("start gp close gp command ")
            # elif self.state_flag == 1:
            #     self.control_command = "HALF_OPEN_GRIPPER"
            #     state = self.open_half_gp()
            # elif self.state_flag == 2:
            #     self.control_command = "OPEN_GRIPPER"
            #     state = self.open_all_gp()
            start_time = time.time()
            # print("[INFO] Running gripper control loop...")
            state_1 = self.set_gp_state(self.state_data_1,can_id=0)
            # state_2 = state_1
            state_2 = self.set_gp_state(self.state_data_2,can_id=1)
            # print(f"state_1:{state_1},state_2:{state_2}")
            # 3. 强制获取返回帧
            if state_1 is not None and state_2 is not None:
                feedback = [state_1,state_2]
                if feedback == []:
                    pass
                else:
                    # print(f"返回帧:{feedback}")
                    self.gp_control_state_signal.emit(feedback)  # 发送反馈信号
            # 4. 以 50Hz 频率运行（20ms 间隔）
            # time.sleep(0.02)
            end_time = time.time()
            # print(f"耗时:{(end_time - start_time)*1000:.2f} ms")
    def set_state_flag(self,value):
        """修改 self.state_flag"""
        self.state_data_1 = value[0]
        self.state_data_2 = value[1]
        # while True:
            
    def stop(self):
        """
        退出线程
        """
        self.running = False
        print("[INFO] Gripper thread stopping...")

    def open_serial(self):
        """尝试打开两个串口，如果都失败则报错"""
        for port in self.DEFAULT_SERIAL_PORTS:
            try:
                ser = serial.Serial(port, self.BAUD_RATE, timeout=1)
                print(f"串口 {port} 已打开，波特率 {self.BAUD_RATE}")
                return ser
            except Exception as e:
                print(f"无法打开串口 {port}: {e}")
        
        # If both attempts fail, raise an error
        print(f"无法打开任何串口: {', '.join(self.DEFAULT_SERIAL_PORTS)}")
    
    def send_data(self, data):
        """发送数据到串口"""
        ser=self.ser
        if ser and ser.is_open:
            ser.write(data)
            # print(f"发送数据: {data.hex()}")
        else:
            print("串口未打开，无法发送数据")


    def filter_can_data(self, data):
        """根据头（0x5A）和尾（0xA5）过滤数据"""
        valid_frames = []

        # 查找所有以 0x5A 开头并以 0xA5 结尾的数据帧
        start_idx = 0
        while start_idx < len(data):
            # 查找下一个0x5A
            start_idx = data.find(b'\x5A', start_idx)
            if start_idx == -1:  # 如果找不到0x5A，退出循环
                break

            # 查找下一个0xA5
            end_idx = data.find(b'\xA5', start_idx)
            if end_idx == -1:  # 如果找不到0xA5，退出循环
                break

            # 提取有效数据帧（包括0x5A和0xA5）
            frame = data[start_idx:end_idx + 1]

            # 确保数据帧长度合理（至少 8 字节）
            if len(frame) >= 8:
                valid_frames.append(frame)

            # 设置起始索引，继续查找下一个帧
            start_idx = end_idx + 1
        return valid_frames

    def read_data(self):
        """读取串口返回数据并过滤符合头尾要求的数据"""
        ser = self.ser
        if ser and ser.is_open:
            data = ser.read(32)  # 读取最大 64 字节
            if data:
                valid_frames = self.filter_can_data(data)
                if valid_frames:
                    back_data=0
                    for frame in valid_frames:
                        if frame[:2].hex()=='5aff':
                            # print("")
                            continue
                        else:
                            # print(f"接收符合条件的CAN数据: {frame.hex()}")
                            back_data=frame.hex()
                    return valid_frames, back_data
                else:
                    pass
            else:
                print("未收到数据")
        else:
            print("串口未打开，无法读取数据")
        return None
    def send_can_data(self, can_id, data, channel):
        """
        发送 CAN 数据帧
        :param ser: 串口对象
        :param can_id: 4字节 CAN ID
        :param data: 发送数据，最大 64 字节
        """
        can_id_bytes = can_id  # CAN ID 转换成 4字节

        data_length = len(data)
        if data_length > 64:
            data = data[:64]  # 限制数据长度为 64 字节
        channel = channel & 0x01  # 确保 channel 只有1位
        frame_header = b'\x5A'  # 帧头
        frame_info_1 = (data_length | channel << 7).to_bytes(1, 'big')  # CAN通道0, DLC数据长度
        frame_info_2 = b'\x00'  # 发送类型: 正常发送, 标准帧, 数据帧, 不加速
        frame_data = data.ljust(64, b'\x00')  # 数据填充到 64 字节
        frame_end = b'\xA5'  # 帧尾

        send_frame = frame_header + frame_info_1 + frame_info_2 + can_id_bytes + frame_data[:data_length] + frame_end
        # print("发送 CAN 帧:", send_frame.hex())
        self.send_data(send_frame)
        # _,data = self.read_data()
        # return data
    def open_half_gp(self):
        half_open_gp = b'\x00\x7f\xFF\xFF\xFF\xFF\x00\x00'
        while 1:
            self.send_can_data(b'\x00\x00\x00\x08', half_open_gp, 0x01)
            data = self.read_data() 
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x08', half_open_gp, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
        
    def open_all_gp(self):
        open_gp = b'\x00\xff\xFF\xFF\xFF\xFF\x00\x00'
        while 1:
            self.send_can_data(b'\x00\x00\x00\x08', open_gp, 0x01)
            data = self.read_data() 
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x08', open_gp, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
    def set_gp_state(self,value,can_id=1):
        assert 0 <= value <= 255, "value must be between 0 and 255"
        open_gp = b'\x00' + value.to_bytes(1, 'big') + b'\xFF\xFF\xFF\xFF\x00\x00'
        
        while 1:
            self.send_can_data(b'\x00\x00\x00\x08', open_gp, can_id)
            data = self.read_data() 
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x08', open_gp, can_id)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                        # print(gpdata,gpdata[2:4])
                        # print(type(data),type(gpdata))

                        # if gpdata[2:4] == b'\x88':
                        #     print("can1")
                        # if gpdata[2:4] == b'\x08':
                        #     print("can0")
                canid, gpstate,gppos,gpforce = gpdata[2:4],gpdata[16:18],gpdata[18:20],gpdata[22:24]
                # print(canid,can_id)
                if canid == '88' and can_id == 1:
                    # print("can1")
                    pass
                elif canid == '08' and can_id == 0:
                    # print("can0")
                    pass
                else:
                    continue
                return [gpstate,gppos,gpforce]
    def close_gp(self):
        close_gp = b'\x00\x00\xFF\xFF\xFF\xFF\x00\x00'
        while 1:
            self.send_can_data(b'\x00\x00\x00\x08', close_gp, 0x01)
            data = self.read_data() 
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x08', close_gp, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
        
    def control_gp(self, gpstate, gppos, gpforce):
        gpstate = gpstate.to_bytes(2, 'big')
        gppos = gppos.to_bytes(2, 'big')
        gpforce = gpforce.to_bytes(2, 'big')
        gpcontrol_data = b'\x00\x00' + gpstate + gppos + b'\x00\x00' + gpforce
        print(f"gpcontrol_data: {gpcontrol_data.hex()}")
            
        while 1:   
            self.send_can_data(b'\x00\x00\x00\x08', gpcontrol_data, 0x01)
            data = self.read_data()
            if data is not None:
                _, gpdata = data
                while gpdata == 0:
                    self.send_can_data(b'\x00\x00\x00\x08', gpcontrol_data, 0x01)
                    data = self.read_data()
                    if data is not None:
                        _, gpdata = data
                gpstate,gppos,gpforce = gpdata[16:18],gpdata[18:20],gpdata[22:24]
                return [gpstate,gppos,gpforce]
            # return data
    
    def close(self):
        if self.ser:
            self.ser.close()

class ROBOT:
    def __init__(self):
        self.joint_state_right=None
        self.Client = PersistentClient('192.168.3.15', 8001)
        # self.Client = PersistentClient('192.168.2.14', 8001)
        # self.Client.set_close(robot_num)
        # self.Client.set_clear(robot_num)
        # self.Client.set_open(robot_num)
        # self.rm_65_b_right_arm = (RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E))
        # self.arm_ini = self.rm_65_b_right_arm.rm_create_robot_arm("192.168.1.18",8080, level=3)

    def get_state(self, model='joint',robot_num=1):#pose
        # self.joint_state_right = self.rm_65_b_right_arm.rm_get_current_arm_state()
        # return_action = self.joint_state_right[1][model]
        if model=='joint':
            action=self.Client.get_arm_position_joint(robotnum=robot_num)
        elif model=='pose':
            action = self.Client.get_arm_position_pose(robotnum=robot_num)
        
        return action
    def set_state(self, action, model='joint',robot_num=1):

        if model=='joint':
            self.Client.set_arm_position(action,'joint',robot_num)
        elif model=='pose':
            self.Client.set_arm_position(action,'pose',robot_num)
        else:
            raise ValueError(f"model {model} is not support")
    def enable_power(self):
        self.Client.set_close(1)
        self.Client.set_clear(1) 
        self.Client.set_close(2)
        self.Client.set_clear(2)
        time.sleep(1)
        self.Client.set_open(2)
        self.Client.set_open(1)
    def stop(self,robot_num):
        self.Client.set_stop(robot_num)
        self.Client.set_reset(robot_num)
    
        # self.rm_65_b_right_arm.rm_set_stop()

class ACTION_PLAN(QThread):
    action_signal = pyqtSignal(object)
    complete_signal = pyqtSignal(object)
    traja_reverse_signal = pyqtSignal(object)
    close_signal = pyqtSignal(object)
    def __init__(self):
        super().__init__()
        self.running = True  # 线程运行状态
        self.Robot=ROBOT()
        self.velocity = 15
        self.gpstate:list
        self.point=None
        self.goal_point = None
        self.local_desktop_point = None
        self.loop_len=1
        self.complete =False
        self.robot_num = 1
        self.task_name = 'exchange'
        # self.duikong_points = [
        #             [-127.243, -584.915, -238.195, 2.83726, -0.121101, 0.283056],#左手，先开爪
        #             [-122.727, -575.222, -306.648, 2.51579, -0.107995, 0.084948],
        #             [-133.111, -603.627, -349.451, 2.41829, 0.0258569, 0.0772928],#关闭左，开右夹爪
        #             [-126.106, -676.665, -177.715, 2.52124, -0.176581, 0.523211],
        #             [-124.602, -752.314, -212.362, 2.47183, -0.00238126, 1.49759],
        #             [-122.668, -810.366, -283.547, 2.47194, -0.00226187, 1.49765],
        #             ]

        self.change_points_right = [
                    [320.871, 8.02429, 569.908, -2.77289, 1.54682, -0.36409],   #右手                 
                    [231.133, 157.494, 682.548, -1.14296, 1.00109, -1.89602],
                    [-77.3069, 482.59, 523.98, -1.74372, -0.200726, -1.37602],
                    [-105.948, 601.398, 173.911, -2.39015, -0.206311, -1.52225],
                    [-118.023, 668.517, -105.539, -2.20681, -0.119449, -2.58369],
                    [-132.427, 620.664, -172.781, -2.37856, -0.125904, -3.09914],
                    ]
        self.right_complete_flag = False
        self.change_points_left = [
                   
                    [-127.205, -584.918, -238.196, 2.8373, -0.121059, 1.07089],#左手，先开爪
                   [-122.71, -575.238, -306.627, 2.51576, -0.107987, 0.866255],
                    [-131.247, -597.281, -312.932, 2.3609, 0.025799, 0.861613],#关闭左，开右夹爪
                   [-126.071, -676.698, -177.695, 2.5211, -0.176558, 1.30888],
                   [-124.581, -752.295, -212.366, 2.47183, -0.00239102, 2.28396],
                     [-123.951, -796.935, -267.617, 2.47178, -0.00240316, 2.28397],
                      [-108.192, -795.045, -270.908, 2.47179, -0.00237977, 2.28395],
                     [-123.951, -796.935, -267.617, 2.47178, -0.00240316, 2.28397],
                    [-91.7134, -874.126, -52.4004, 2.24524, -0.0815834, 2.11197],
                    [-41.0751, -734.438, 207.143, 2.28154, 0.145922, 2.31647],
                    [629.137, -161.689, 590.811, 1.6477, 1.38221, 2.1665],
                    [-41.0751, -734.438, 207.143, 2.28154, 0.145922, 2.31647],
                    [-127.205, -584.918, -238.196, 2.8373, -0.121059, 1.07089]
                       ]  # 存储所有点位
    def val(self, point):
        self.point = point

    def move(self,position,up=False,robot_num=1):
        if up is False:
            # self.Robot.rm_65_b_right_arm.rm_movej_p(position,self.velocity,0,0,1)
            self.Robot.set_state(position,'pose',robot_num)
        else:
            position_=position.copy()
            position_[1]=position_[1]-0.1
            # self.Robot.rm_65_b_right_arm.rm_movej_p(position_,self.velocity,0,0,1)
            self.Robot.set_state(position_,'pose',robot_num)
    def run(self):
        global action_play,save_signal,traj_signal
        while self.running:
            # print(f"running:{self.running}")
            if action_play :
                self.move(self.point,robot_num=self.robot_num)
                # action_play = False
            else:
                # print(save_signal)
                if save_signal:
                    continue
                if self.right_complete_flag:
                    pass
                else:
                    self.exchange_task()
                print(save_signal)
                time.sleep(2)
                if save_signal:
                    continue
                self.duikong()
                time.sleep(100)
                # self.running = False
                self.right_complete_flag = False
                if not self.running:
                    break

    def exchange_task(self):
        global traj_signal
        self.close_signal.emit(False)
        traj_signal = 1
        self.action_signal.emit([128,0])
        time.sleep(1)
        self.complete_signal.emit(False)
        self.traja_task_move(self.change_points_right,robot_num=2)
        self.close_signal.emit(True)
        time.sleep(1)
        self.traja_task_move(self.change_points_left[:3],robot_num=1)
        # time.sleep(1)
        
        self.action_signal.emit([0,0])
        time.sleep(2)
        self.action_signal.emit([0,128])
        time.sleep(4)
        self.close_signal.emit(False)
        # print(len(self.change_points_right),self.change_points_right[:3])
        self.traja_reverse_task_move(self.change_points_right[3:],robot_num=2)
        self.action_signal.emit([0,0])
        # time.sleep(1)
        self.traja_reverse_task_move(self.change_points_right[:3],robot_num=2)
        # print("右手运动完成")
        self.close_signal.emit(True)
        self.traja_task_move(self.change_points_left[3:7],robot_num=1)
        # time.sleep(1)
        self.complete_signal.emit(True)
        # self.close_signal.emit(False)
        self.right_complete_flag = True


    def duikong(self):
        global traj_signal
        traj_signal = 2
        self.traja_task_move(self.change_points_left[7:11],robot_num=1)
        self.action_signal.emit([128,0])
        time.sleep(4)
        self.traja_task_move(self.change_points_left[11:],robot_num=1)
        self.complete_signal.emit(True)

    def traja_task_move(self,points,robot_num):
        # if not hasattr(self, "points") or not self.points:
        #     raise ValueError("请先设置至少一个点位")

        self.complete_signal.emit(False)
        for idx, point in enumerate(points):
            
            if self.is_close(self.Robot.get_state(model='pose'), point, tolerance=0.1):
                continue
            # if point == self.points[3]:
            #     self.close_signal.emit(True)
            # print("运行")
            if point is not None:
                print(point)
                # 判断是否是后三个点
                if idx >= len(points) - 2:
                    self.move(point,robot_num=robot_num)  
                else:
                    rand_point = self.random_positon(point)
                    # print(rand_point)
                    self.move(point,robot_num=robot_num)

            if not self.running:
                self.stop()
        time.sleep(1)
        # self.complete_signal.emit(True)
        # self.close_signal.emit(False)
        print("发送 action_plan 完成信号")
        # time.sleep(2)

    def traja_reverse_task_move(self,points,robot_num):
        # if not hasattr(self, "points") or not self.points:
        #     raise ValueError("请先设置至少一个点位")

        # self.traja_reverse_signal.emit(True)
        
        # 逆序遍历 `self.points`，让机器人按原轨迹返回
        for idx, point in enumerate(reversed(points)):
            # print(point)
            if self.is_close(self.Robot.get_state(model='pose'), point, tolerance=0.1):
                continue

            if point is not None:
                self.move(point,robot_num=robot_num)
            if not self.running:
                self.stop()
        # self.traja_reverse_signal.emit(False)
        # self.complete_signal.emit(True)

        print("发送 action_plan 逆向完成信号")
        time.sleep(2)
    def change_task_move(self):
        self.complete_signal.emit(False)
        time.sleep(2)
        self.action_signal.emit([254,0])
        time.sleep(2)

        self.action_signal.emit([254,1])
        time.sleep(2)
        self.action_signal.emit([0,0])
        time.sleep(2)
        self.action_signal.emit([0,1])
    def is_close(self, actual, target, tolerance=0.1):
        """
        判断两个列表的每个元素是否在允许误差范围内
        :param actual: 实际值列表（如当前机械臂状态）
        :param target: 目标值列表
        :param tolerance: 允许的最大误差（绝对值）
        :return: 所有元素均满足误差要求返回True，否则False
        """
        # 处理None和长度检查
        if actual is None or target is None:
            return False
        if len(actual) != len(target):
            return False
        
        # 逐个元素比较误差
        for a, t in zip(actual, target):
            # print(abs(a - t))
            if abs(a - t) > tolerance:
                return False
        return True

    def random_positon(self,point,a=25,b=30):
        random_pos = point.copy()
        random_pos[0] += random.uniform(a,b)  # 生成 1.5 到 3.5 之间的随机浮点数
        random_pos[1] += random.uniform(a,b)  # 生成 1.5 到 3.5 之间的随机浮点数
        random_pos[2] += random.uniform(a,b)  # 生成 1.5 到 3.5 之间的随机浮点
        # random_pos[3] += random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点数
        # random_pos[4] += random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点数
        # random_pos[5] += random.uniform(1.5, 3.5)  # 生成 1.5 到 3.5 之间的随机浮点
        return random_pos
    def set_loop_len(self,value):
        self.loop_len=value
    def stop(self):
        """ 停止线程 """
        self.running = False
        
        # self.Robot.rm_65_b_right_arm.rm_set_delete_current_trajectory()
        self.quit()
        # self.wait()

class GENERATOR_HDF5:
    def __init__(self):
        pass
    def save_hdf5(self,data_dict,path_to_save_hdf5,episode_idx,compressed=True,arm_name='all'):
        os.makedirs(path_to_save_hdf5, exist_ok=True)
        self.dataset_path = os.path.join(path_to_save_hdf5, f'episode_{episode_idx}.hdf5')

        try:
            with h5py.File(self.dataset_path, 'w') as root:
                root.attrs['sim'] = True
                obs = root.create_group('observations')
                images_group = obs.create_group('images')
                gp = root.create_group('gp')

                # 创建每个相机的数据集并写入数据
                for cam_name in camera_names:
                    if f'/observations/images/{cam_name}' in data_dict:
                        try:
                            cam_data = np.array(data_dict[f'/observations/images/{cam_name}'])
                            print(f"Saving image for {cam_name}, shape: {cam_data.shape}")  # 打印图片数据的尺寸
                            images_group.create_dataset(
                                cam_name.split('/')[-1],
                                data=cam_data,
                                dtype='uint8',
                                compression="gzip", 
                                compression_opts= 4 if compressed else 0
                            )
                        except Exception as e:
                            print(f"Error saving image data for camera {cam_name}: {e}")

                # 写入 qpos 数据
                if '/observations/qpos' in data_dict:
                    if 'qpos' in obs:
                        print("Dataset 'qpos' already exists. Updating it.")
                        del obs['qpos']
                    if arm_name == 'all':
                        qpos_data = np.array(data_dict['/observations/qpos'])
                        print(f"Saving qpos, shape: {qpos_data.shape}")
                        obs.create_dataset(
                            'qpos',
                            data=qpos_data,
                            dtype='float32'
                        )
                    elif arm_name == 'left_arm':
                        qpos_data = np.array(data_dict['/observations/qpos'])[:,:8]
                        print(f"Saving qpos, shape: {qpos_data.shape}")
                        obs.create_dataset(
                            'qpos',
                            data=qpos_data,
                            dtype='float32'
                        )
                    elif arm_name == 'right_arm':
                        qpos_data = np.array(data_dict['/observations/qpos'])[:,8:]
                        print(f"Saving qpos, shape: {qpos_data.shape}")
                        obs.create_dataset(
                            'qpos',
                            data=qpos_data,
                            dtype='float32'
                        )
                # 写入 action 数据
                if '/action' in data_dict:
                    if 'action' in root:
                        print("Dataset 'action' already exists. Updating it.")
                        del root['action']
                    if arm_name == 'all':
                        
                        action_data = np.array(data_dict['/action'])
                        print(f"Saving action, shape: {action_data.shape}")
                        root.create_dataset(
                            'action',
                            data=action_data,
                            dtype='float32'
                        )
                    elif arm_name == 'left_arm':
                        action_data = np.array(data_dict['/action'])[:,:8]
                        print(f"Saving action, shape: {action_data.shape}")
                        root.create_dataset(
                            'action',
                            data=action_data,
                            dtype='float32'
                        )
                    elif arm_name == 'right_arm':
                        action_data = np.array(data_dict['/action'])[:,8:]
                        print(f"Saving action, shape: {action_data.shape}")
                        root.create_dataset(
                            'action',
                            data=action_data,
                            dtype='float32'
                        )
                # 保存 gpstate, gppos, gpforce 数据
                if '/gp/gppos' in data_dict:
                    if 'gppos' in gp:
                        print("Dataset 'gppos' already exists. Updating it.")
                        del gp['gppos']
                    try:
                        gppos_data = np.array([int(x, 16) for x in data_dict['/gp/gppos']], dtype='int32')
                        print(f"Saving gppos, length: {len(gppos_data)}")
                        gp.create_dataset(
                            'gppos',
                            data=gppos_data
                        )
                    except Exception as e:
                        print(f"Error saving gppos data: {e}")
                        return

                if '/gp/gpstate' in data_dict:
                    if 'gpstate' in gp:
                        print("Dataset 'gpstate' already exists. Updating it.")
                        del gp['gpstate']
                    try:
                        gpstate_data = np.array([int(x, 16) for x in data_dict['/gp/gpstate']], dtype='int32')
                        print(f"Saving gpstate, length: {len(gpstate_data)}")
                        gp.create_dataset(
                            'gpstate',
                            data=gpstate_data
                        )
                    except Exception as e:
                        print(f"Error saving gpstate data: {e}")
                        return

                if '/gp/gpforce' in data_dict:
                    if 'gpforce' in gp:
                        print("Dataset 'gpforce' already exists. Updating it.")
                        del gp['gpforce']
                    try:
                        gpforce_data = np.array([int(x, 16) for x in data_dict['/gp/gpforce']], dtype='int32')
                        print(f"Saving gpforce, length: {len(gpforce_data)}")
                        gp.create_dataset(
                            'gpforce',
                            data=gpforce_data
                        )
                    except Exception as e:
                        print(f"Error saving gpforce data: {e}")
                        return

                # 强制刷新文件，确保数据写入
                root.flush()

        except Exception as e:
            print(f"Error during saving hdf5 file: {e}")
        
        print(f"\033[92mData saved to {self.dataset_path}\033[0m")

class TemporalFilter:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        if self.previous_frame is None:
            self.previous_frame = frame
            return frame
        result = cv2.addWeighted(frame, self.alpha, self.previous_frame, 1 - self.alpha, 0)
        self.previous_frame = result
        return result

class run_main_windows(QWidget):
    def __init__(self):
        super().__init__()
        self.robot=ROBOT()
        self.camera=CAMERA_HOT_PLUG()
        self.gpcontrol = GPCONTROL()
        self.generator_hdf5=GENERATOR_HDF5()
        self.action_plan=ACTION_PLAN()
        self.gpcontrol.gp_control_state_signal.connect(self.handle_feedback)  # 连接信号到槽函数
        self.gpcontrol.error_signal.connect(self.handle_error_signal)
        self.action_plan.action_signal.connect(self.handle_action_signal)
        self.action_plan.complete_signal.connect(self.handle_complete_signal)
        self.action_plan.traja_reverse_signal.connect(self.handle_traja_reverse_signal)
        self.action_plan.close_signal.connect(self.handle_close_signal)
        self.stop_render=True
        self.image:dict[str,np.array]={}
        self.images_dict = {cam_name: [] for cam_name in camera_names}  # 用于存储每个相机的图片
        self.qpos_list=[]
        self.action_list=[]
        self.gpstate_list=[]
        self.gppos_list=[]
        self.gpforce_list=[]
        self.gpstate=[]
        self.max_episode_len=5000

        self.start_index = 0    
        self.index=self.start_index
        self.task_complete_step = 0
        self.end_index=50
        self.start_pos,self.local_desktop_pos,self.goal_pos=[-9.19798, -84.536, 631.215, 1.42741, -0.0901151, 2.83646],[-98.2562, -0.131311, 30.7511, 3.35265, -26.1317, 72.2164],[-99.3707, -82.4347, 65.0209, 8.46747, -53.2068, 69.0324]
        self.complete_sign = False
        self.traja_reverse_signal = False
        self.close_signal = False
        self.save_signal = False
        self.auto_collect = True
        self.create_widget()

    def create_widget(self):
        self.setWindowTitle("ACT GET DATA")
        screen = app.primaryScreen()  # 获取主屏幕
        size = screen.geometry()  # 获取屏幕几何信息
        screen_width = size.width()
        screen_height = size.height()
        window_width = 1920
        window_height = 1080

        # 创建定时器
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.updata_frame)
        self.task_timer = QTimer()
        self.task_timer.timeout.connect(self.updata_collect_task)

        # 主布局管理器
        layout = QVBoxLayout(self)

        # 设置窗口可调整大小
        self.setGeometry(0, 0, window_width, window_height)
        self.setMinimumSize(800, 600)  # 最小窗口大小可选

        # 创建视频显示区域
        self.label = QLabel(' ', self)
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # 关键！
        self.label.setStyleSheet("background-color: black")  # 可选：背景色
        layout.addWidget(self.label, stretch=3)  # 给更大权重

        # 创建选择框与输入框布局
        form_layout = QFormLayout()
        self.input_box = QLineEdit()
        self.input_box.setPlaceholderText("Input position")
        self.input_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        form_layout.addRow("Data:", self.input_box)
        layout.addLayout(form_layout)

        # 设置按钮布局
        button_layout = QHBoxLayout()
        buttons = [
            ("Set Point", self.on_setting_btn_click),
            ("Get Robot State", self.on_get_robot_state_btn_click),
            ("Start Camera", self.start_camera),
            ("Close Camera", self.stop_camera),
            ("修改位置开关", self.on_push_change_mode_btn_click),
            ("使能", self.on_push_enable_btn_click),
            ("上电", self.on_push_enable_power_btn_click),
        ]

        for text, handler in buttons:
            btn = QPushButton(text)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(handler)
            button_layout.addWidget(btn)

        layout.addLayout(button_layout, stretch=1)

        # Task 按钮部分
        for text, handler in [
            ("Start Task", self.on_start_task_btn_click),
            ("Stop Task", self.on_stop_task_btn_click),
            ("START COLLECTION", self.on_start_collection_btn_click),
            ("STOP COLLECTION", self.on_stop_collection_btn_click),
        ]:
            btn = QPushButton(text)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(handler)
            layout.addWidget(btn)

        # 进度控制与紧急按钮
        self.episode_index_box = QLineEdit()
        self.episode_index_box.setPlaceholderText(f"Set start episode_idx.., default: {self.start_index },{self.end_index}")
        self.episode_index_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(QLabel("Index:"))
        layout.addWidget(self.episode_index_box)

        self.set_index = QPushButton("Set Index")
        self.set_index.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.set_index.clicked.connect(self.on_set_index_btn_click)
        layout.addWidget(self.set_index)

        self.stop_emergency = QPushButton("STOP", self)
        self.stop_emergency.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.stop_emergency.clicked.connect(self.on_stop_emergency_btn_click)
        layout.addWidget(self.stop_emergency)

        # 结果显示与进度条
        self.result_label = QLabel("")
        self.result_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.result_label)

        self.progress_bar = QProgressBar(self)
        self.progress_bar.setValue(0)
        self.progress_bar.setRange(0, 500)
        self.progress_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.progress_bar)

        # 设置窗口的布局
        self.setLayout(layout)

        print("控件初始化完成")

    # def on_add_point_btn_click(self):
    #     point_count = self.combo_box.count() + 1  # 计算新点的索引
    #     self.combo_box.addItem(f"Point {point_count}")  # 添加新点
    #     print(f"添加新的点位: Point {point_count}")
    def on_start_collection_btn_click(self):
        self.gpcontrol.start()
        self.progress_value = 0  # 复位进度
        self.task_timer.start(50)
    def on_stop_collection_btn_click(self):
        self.robot.stop(robot_num=1)
        self.robot.stop(robot_num=2)
        self.complete_sign = True
        time.sleep(1)
        self.task_timer.stop()

    def on_push_change_mode_btn_click(self):
        global action_play
        if action_play == False:
            action_play = True
            self.change_mode_btn.setText("action_play:True")  # 直接修改文本
        else:
            action_play = False
            self.change_mode_btn.setText("action_play:False")
        print(action_play)
    def on_push_enable_power_btn_click(self):
        self.robot.enable_power()
    def on_push_enable_btn_click(self):
        self.action_plan.start()
    def on_setting_btn_click(self):
        # global action_play
        # 读取 QLineEdit 输入的点位数据
        input_text = self.input_box.text().strip()
        try:
            # 将输入文本转换为列表，例如 "[1.0, 2.0, 3.0]"
            position = eval(input_text)[0]  # 直接解析为 Python 列表
            robot_num = eval(input_text)[1] 
            if not isinstance(position, list) or len(position) < 3:
                raise ValueError  # 确保是合法的坐标格式
        except:
            QMessageBox.warning(self, "输入错误", "请输入正确的坐标格式，如：[1.0, 2.0, 3.0]")
            return
        print(position,robot_num)
        self.action_plan.point = position  # 更新指定索引的点位
        self.action_plan.robot_num = robot_num
        print(position)
        # action_play = True
    def on_get_robot_state_btn_click(self):
        self.pose = self.robot.get_state('pose',robot_num=1)
        print(self.pose)
        self.input_box.setText(json.dumps(self.pose))
    def on_start_task_btn_click(self):
        global action_play
        # if self.action_plan.points is None:
        #     self.result_label.setText("points is None")
        # elif self.local_desktop_pos is None:
        #     self.result_label.setText("local_desktop_pos is None")
        # elif self.goal_pos is None:
        #     self.result_label.setText("goal_pos is None")
        # else:
            # self.action_plan.val(self.start_pos,self.goal_pos,self.local_desktop_pos)
        self.gpcontrol.start()
        if self.action_plan.isRunning():
            print("Task already running")
        else:
            self.action_plan.start()
        action_play = False
        self.progress_value = 0  # 复位进度
        self.task_timer.start(50)
        self.result_label.setText("task start")
        time.sleep(1)
    def on_stop_task_btn_click(self):
        self.robot.stop(robot_num=1)
        self.robot.stop(robot_num=2)
        self.task_timer.stop()
        # self.camera_timer.stop()
        self.gpcontrol.stop()
        self.action_plan.stop()
        self.result_label.setText("task stop")
    def on_stop_emergency_btn_click(self):
        self.task_timer.stop()
        self.camera_timer.stop()
        self.gpcontrol.stop()
        self.action_plan.stop()
    def updata_collect_task(self):
        # print(self.traja_reverse_signal)
        # if not self.traja_reverse_signal:2,
        start_time = time.time()
        # time.sleep(5)
        if not False:
            
            self.progress_value += 1
            # print(f"当前进度: {self.progress_value}")
            self.progress_bar.setValue(self.progress_value)
   
            angle_qpos_robot_num_1=self.robot.get_state(model='joint',robot_num=1)
            angle_qpos_robot_num_2=self.robot.get_state(model='joint',robot_num=2)
        
            # if self.robot.get_state('pose'):
            #     self.robot.set_state(self.robot.get_state('pose'))
            # print(time.time()-start)

            radius_qpos_robot_num_1 = [math.radians(j) for j in angle_qpos_robot_num_1]
            radius_qpos_robot_num_2 = [math.radians(j) for j in angle_qpos_robot_num_2]
            # 处理 gpstate
            # time.sleep(10)
            # print(self.gpstate)
            if self.gpstate:
                gpstate, gppos, gpforce = map(lambda x: str(x) if not isinstance(x, str) else x, self.gpstate[0])
                radius_qpos_robot_num_1.extend([int(gppos, 16), int(gpforce, 16)])
                gpstate, gppos, gpforce = map(lambda x: str(x) if not isinstance(x, str) else x, self.gpstate[1])
                radius_qpos_robot_num_2.extend([int(gppos, 16), int(gpforce, 16)])      
            # else:
            #     raise ValueError("error in gpstate")
            # 记录 qpos 数据
            gp_data = [x for row in [radius_qpos_robot_num_1,radius_qpos_robot_num_2] for x in row]
            # print(f"gp_data shape:{np.array(gp_data).shape}")
            # print(f"qpos_list:{radius_qpos_robot_num_1},{radius_qpos_robot_num_2},{[radius_qpos_robot_num_1,radius_qpos_robot_num_2]}")
            self.qpos_list.append(gp_data)
            # 记录图像数据
            if self.image:
                for camera_name in camera_names:
                    self.images_dict[camera_name].append(self.image.get(camera_name))
            if self.auto_collect:
                if self.close_signal:
                    # time.sleep(0.1)
                    pass
                else:
                    # time.sleep(0.95)

                    pass
                if self.traja_reverse_signal is True and self.task_complete_step == 0:
                    self.task_complete_step = self.progress_value
        end_time = time.time()
        # print(f"一帧时间：{end_time - start_time}")
            # 任务完成检查
        # if self.progress_value:
            # if self.progress_value >= 100:
            #     self.complete_sign = True
            #     self.task_complete_step = self.progress_value/2
            #     print(self.task_complete_step)
            #     self.result_label.setText("task complete")
        if self.complete_sign:
            # print(self.index - self.start_index)
            if self.index >= self.end_index :
                print('task completed')
                print("stop")
                if self.auto_collect:
                    self.action_plan.stop() 
                self.task_timer.stop()
                self.save_data()

                self.complete_sign=False
                print("stop_collect_data")
            else:
                self.save_data()
                self.complete_sign=False
                print("completed data collection")

    def handle_feedback(self,feed_back):
        self.gpstate=feed_back
        # print("feed_back:",self.gpstate)
    def handle_traja_reverse_signal(self,traja_feed_back):
        self.traja_reverse_signal = traja_feed_back
        print(f"traja_reverse_signal :{traja_feed_back}")
    def handle_action_signal(self,action_feed_back):
        # print(action_feed_back[0],action_feed_back[1])
        self.gpcontrol.set_state_flag(action_feed_back)
    def handle_complete_signal(self,complete_feed_back):
        self.complete_sign =complete_feed_back
        print(f"complete_sign :{complete_feed_back}")
    def handle_error_signal(self,error_feed_back):
        print(error_feed_back)
        self.result_label.setText(error_feed_back)
    def handle_close_signal(self,close_signal):
        self.close_signal = close_signal
    # def save_data(self):

    #     try:
    #         global save_signal
    #         data_dict:dict[str,np.array]={}
    #         data_dict_add:dict[str,np.array]={}
    #         save_signal = True
    #         self.action_list = self.qpos_list
    #         self.max_episode_len=self.progress_value
    #         if self.qpos_list is not None:
    #             # self.qpos_list = np.vstack([self.qpos_list[0], self.qpos_list])
    #             self.qpos_array = np.vstack([self.qpos_list[0], self.qpos_list])  # 存入一个新变量
    #         else:
    #             raise "qpos is none"
    #         data_dict = {
    #             '/observations/qpos': self.qpos_array[:self.max_episode_len],
    #             '/action': self.action_list[:self.max_episode_len],
    #         }
            
    #         data_dict_add = {
    #             '/observations/qpos': self.qpos_array[:self.max_episode_len],
    #             '/action': self.action_list[:self.max_episode_len],
    #         }
    #         self.progress_value = 0  # 复位进度
    #         self.qpos_list.clear()
    #         self.action_list.clear()
    #         for cam_name in camera_names:
    #             data_dict[f'/observations/images/{cam_name}'] = self.images_dict[cam_name][:self.max_episode_len]
            
    #         self.generator_hdf5.save_hdf5(data_dict,"./hdf5_file",self.index)
    #         data_dict:dict[str,np.array]={}

    #         if self.task_complete_step != 0:
    #             for cam_name in camera_names:
    #                 images = self.images_dict[cam_name][:self.max_episode_len]
    #                 colored_images = []
    #                 square_size = 100

    #                 for i, img in enumerate(images):
    #                     img_copy = [row[:] for row in img]  # 深拷贝，防止改到原图

    #                     height = len(img_copy)
    #                     width = len(img_copy[0])
    #                     square_color = [0, 0, 255] if i < self.task_complete_step else [0, 255, 0]  # 蓝或绿（RGB）

    #                     # 左下角：行范围 [height - square_size, height)
    #                     for row in range(height - square_size, height):
    #                         for col in range(square_size):
    #                             if 0 <= row < height and 0 <= col < width:
    #                                 img_copy[row][col] = square_color

    #                     colored_images.append(img_copy)
    #                 self.images_dict[cam_name] = colored_images
    #                 data_dict_add[f'/observations/images/{cam_name}'] = self.images_dict[cam_name][:self.max_episode_len]
    #                 self.images_dict[cam_name].clear()
    #         self.generator_hdf5.save_hdf5(data_dict_add,"./hdf5_file_add",self.index)
    #         self.task_complete_step = 0
    #         data_dict_add:dict[str,np.array]={}

    #         self.index+=1
    #         if self.index>self.end_index:
    #             self.result_label.setText(f"task over please restart or close")
    #         save_signal = False
    #     except Exception as e:
    #         print(f"Save data error: {e}")
    #         self.result_label.setText(f"保存出错：{str(e)}")
    def save_data(self):
        try:
            global save_signal,traj_signal
            save_signal = True
            print("save_data signal",save_signal)
            data_dict = {}
            data_dict_add = {}
            self.action_list = self.qpos_list
            max_episode_len = self.progress_value-1

            if self.qpos_list is not None:
                self.qpos_array = np.vstack([self.qpos_list[0], self.qpos_list])  # 一次性拼接
            else:
                raise ValueError("qpos_list is None")
            self.action_array = np.array(self.action_list)
            # 保存动作和关节状态
            data_dict = {
                '/observations/qpos': self.qpos_array[:max_episode_len],
                '/action': self.action_list[:max_episode_len],
            }

            # 保存图像
            for cam_name in camera_names:
                # 确保图像是np.array
                images_np = np.stack(self.images_dict[cam_name][:max_episode_len], axis=0)  # (N, H, W, C)
                data_dict[f'/observations/images/{cam_name}'] = images_np
            data_dict['/action'] = np.array(data_dict['/action'])
            data_dict['/observations/qpos'][:,:6] = np.degrees(data_dict['/observations/qpos'][:,:6])
            data_dict['/observations/qpos'][:,8:14] = np.degrees(data_dict['/observations/qpos'][:,8:14])
            data_dict['/action'][:,:6] = np.degrees(data_dict['/action'][:,:6])
            data_dict['/action'][:,8:14] = np.degrees(data_dict['/action'][:,8:14])
            
            # 保存主文件
            if traj_signal == 1:
                self.generator_hdf5.save_hdf5(data_dict, f"./hdf5_file_exchange_{today_str}", self.index,arm_name='all')
            if traj_signal == 2:
                self.generator_hdf5.save_hdf5(data_dict, f"./hdf5_file_duikong_{today_str}", self.index,arm_name='left_arm')

            # data_dict['/observations/qpos'][1:, 8:14] = data_dict['/observations/qpos'][1:, 8:14] - data_dict['/observations/qpos'][:-1, 8:14]
            # print(type(data_dict['/action']))
            

            # 增量转换
            # data_dict['/observations/qpos'][0, :] = 0
            # data_dict['/observations/qpos'][1:, :] = data_dict['/observations/qpos'][1:, :] - data_dict['/observations/qpos'][:-1, :]
            # # 增量转换
            # data_dict['/action'][1, :] = 0
            # data_dict['/action'][1:, :] = data_dict['/action'][1:, :] - data_dict['/action'][:-1, :]
            # # data_dict['/action'][1:, 8:14] = data_dict['/action'][1:, 8:14] - data_dict['/action'][:-1, 8:14]
            # # print(data_dict['/action'])
            # if traj_signal == 1:
            #     self.generator_hdf5.save_hdf5(data_dict, "./hdf5_file_exchange_5-9_degrees", self.index,arm_name='all')
            # if traj_signal == 2:
            #     self.generator_hdf5.save_hdf5(data_dict, "./hdf5_file_duikong_5-9_degrees", self.index,arm_name='left_arm')
            # qpos_angle_list  = [math.radians(j) for j in data_dict['/observations/qpos'][:,:6]]

            # 清理原始数据
            self.progress_value = 0
            self.qpos_list.clear()
            self.action_list.clear()

            # 如果需要带有色块的图像版本（标注 task_complete_step）
            if self.task_complete_step != 0:
                square_size = 100
                square_color_pre = np.array([0, 0, 255], dtype=np.uint8)   # 蓝
                square_color_post = np.array([0, 255, 0], dtype=np.uint8)  # 绿

                for cam_name in camera_names:
                    images = np.stack(self.images_dict[cam_name][:max_episode_len], axis=0)  # (N, H, W, C)
                    height, width = images.shape[1:3]

                    # 应用色块：直接修改图像像素
                    for i in range(len(images)):
                        color = square_color_pre if i < self.task_complete_step else square_color_post
                        images[i, height - square_size:, :square_size] = color

                    data_dict_add[f'/observations/images/{cam_name}'] = images

                    # 清理图像缓存
                    self.images_dict[cam_name].clear()

            # 保存带标注图像的版本
            if data_dict_add:
                data_dict_add['/observations/qpos'] = self.qpos_array[:max_episode_len]
                data_dict_add['/action'] = self.action_array[:max_episode_len]
                self.generator_hdf5.save_hdf5(data_dict_add, "./hdf5_file_add", self.index, compressed=False)
            for cam_name in camera_names:
                self.images_dict[cam_name].clear()

            # 更新索引和状态
            self.task_complete_step = 0
            self.index += 1

            if self.index > self.end_index:
                self.result_label.setText("task over please restart or close")
            # time.sleep(10)
            
            save_signal = False
            
        except Exception as e:
            print(f"Save data error: {e}")
            self.result_label.setText(f"保存出错：{str(e)}")

    def on_set_index_btn_click(self):
        index=self.episode_index_box.text()
        # print(index)
        if not re.match(r"^\d+,\d+$", index):
            self.result_label.setText(f"index error")
        else:
            index_len = list(map(int, index.split(',')))
            self.end_index=index_len[1]
            self.index=index_len[0]
            self.start_index =index_len[0]

    def start_camera(self):
        """启动摄像头"""
        self.stop_render =False
        self.camera_timer.start(10)

    def updata_frame(self):
        """更新摄像头图像"""
        global multi_device_sync_config
        # if self.camera.change_signal:
            # self.camera = CAMERA_HOT_PLUG()
        #     frame_data, color_width, color_height = self.camera.rendering_frame()

        # else:
        multi_device_sync_config = self.camera.multi_device_sync_config
        color_image_dict,depth_image_dict,color_width, color_height = self.camera.rendering_frame()
        frame_data = color_image_dict
        # print(color_height,color_width)
        serial_number_list = self.camera.serial_number_list
        camera_index_map = {device['config']['camera_name']: serial_number_list.index(device["serial_number"]) for device in multi_device_sync_config.values() if device["serial_number"] in serial_number_list}

        # 初始化结果图像
        result_image = None
        # print(multi_device_sync_config.values())
        for device in multi_device_sync_config.values():
            cam_name, sn = device['config']['camera_name'], device["serial_number"]
            # print(cam_name,sn)
            if sn in frame_data:
                # print(sn)
                img = frame_data[sn]
                if result_image is None:
                    result_image = img  # 第一个摄像头的图像
                else:
                    result_image = np.hstack((result_image, img))  # 按水平方向拼接图像
        # print(result_image.shape)
        if result_image is not None:
            self.display_image(result_image)
            # print(type(np.array(frame_data.get(str(serial_number_list[camera_index_map['top']]), None))))
            if self.task_timer.isActive():
                # print(f"camera_index_map: {camera_index_map}")
                for cam_name in camera_names:
                    # if cam_name == 'top':.
                    self.image[cam_name]=np.array(frame_data.get(str(serial_number_list[camera_index_map[cam_name]]), None))
                    # elif cam_name == 'right_wrist':
                    #     self.image[cam_name]=frame_data.get(str(serial_number_list[camera_index_map[cam_name]]), None)
                    # elif cam_name == 'left_wrist':
                    #     self.image[cam_name]=frame_data.get(str(serial_number_list[camera_index_map[cam_name]]), None)

    def display_image(self,result_image):
        # **显示图像**
        # print(f"display_result_image")
        if self.stop_render is False:
            color_height, color_width, ch = result_image.shape
            # print(result_image.shape)
            qt_img = QImage(result_image, color_width, color_height, ch * color_width, QImage.Format_RGB888)
            qimage = qt_img.rgbSwapped()
            self.label.setPixmap(QPixmap.fromImage(qimage))

    def stop_camera(self):
        """关闭摄像头"""
        self.stop_render=True
        # self.timer.stop()
        self.label.clear()
        # self.camera.stop_straems()
    def start_collect(self):
        """开始采集数据"""
        self.gpcontrol.start(10)
        self.task_timer.start(10)
        self.progress_value = 0  # 复位进度
    def stop_collect(self):
        """停止采集数据"""
 
        print(self.progress_value)
        self.save_data()
        self.task_timer.stop()
        self.result_label.setText("采集已停止")
       
    def closeEvent(self, event):
        """关闭窗口时释放摄像头"""
        print("关闭窗口")
        # self.robot.Client.close()
        self.action_plan.stop()
        # self.camera.stop()
        # print("camera-stop")
        self.camera_timer.stop()
        self.stop_camera()
        # self.camera.stop_streams()
        # print("camera-timer-stop")
        self.task_timer.stop()
        self.gpcontrol.close()
        self.gpcontrol.stop()
        event.accept()

# if __name__ == '__main__':
app = QApplication([])
# 创建并显示窗口
window = run_main_windows()
window.show()
# 启动事件循环
app.exec_()