import time
import math
from backend.robot import ROBOT
from backend.action_plan import ACTION_PLAN
from backend.generator_hdf5 import GENERATOR_HDF5
from backend.serial_control import GripperCANController
from backend.camera import MultiCameraStreamer
import numpy as np
import threading
from datetime import datetime
camera_names = ['left_wrist','top','right_wrist']
today_str = datetime.today().strftime("%m-%d-%H")  # 输出例如 "07-28-14"

class TaskManager():
    def __init__(self,camera:MultiCameraStreamer,gripper:GripperCANController):
        self._running = threading.Event()  # 正确的运行标志
        self._thread = None
        self.robot=ROBOT()
        self.camera_frame_buffer=camera.frame_buffer
        self.gpcontrol = gripper
        self.generator_hdf5=GENERATOR_HDF5()
        self.action_plan=ACTION_PLAN(self.gpcontrol)
        self.qpos_list = []  # 用于存储qpos数据
        self.images_dict = {camera_name: [] for camera_name in camera_names}  # 用于存储图像数据
        self.progress_value = 0  # 进度值
        self.index = 0  # 当前索引
        self.note = ""  # 备注信息
        self.task_is_start = False  # 任务是否开始
        self.thread_started = False
    def run(self):
        """线程运行函数"""
        while self._running.is_set():  # 如果线程被标记为运行状态            
            self._update_data()
            print(f"数据更新中，当前images_dict长度: {len(self.images_dict['left_wrist'])}, 进度值: {self.progress_value}")
            time.sleep(0.1)
        else:
            print("更新线程未启动")
            time.sleep(0.1)
    def stop_thread(self):
        self._running.clear()  # 通知线程退出
        # 避免线程在自己里面 join 自己
        if self._thread is not None and threading.current_thread() != self._thread:
            self._thread.join()
        self._thread = None
    def stop_task(self):
        """停止任务"""
        self.stop_thread()  # 设置停止事件，退出线程循环
        self.robot.close()
        self.action_plan.stop_thread()  # 停止动作计划线程
        self.gpcontrol.stop_thread()  # 停止夹爪控制线程
        self.images_dict = {camera_name: [] for camera_name in camera_names}  # 用于存储图像数据
        self.qpos_list = []  # 用于存储qpos数据
        self.progress_value = 0  # 重置进度值
        self.task_is_start = False
    def start_thread(self):
        if self._thread is None or not self._thread.is_alive():
            self._running.set()
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()
        else:
            print("更新线程已启动")
    def start_task(self):
        """启动任务"""
        try:
            if not self.task_is_start:
                self.robot.enable_power()
                self.gpcontrol.connect_ACM_port()
                self.gpcontrol.start_thread()  # 启动夹爪控制线程
                self.start_thread()
                self.task_is_start = True  # 设置任务已开始
                self.action_plan.start_thread()  # 启动动作计划线程
            else:
                print("任务正在运行")
        except Exception as e:
            
            print(f"启动任务失败: {e}")
            self.stop_task()
    def _update_data(self):
        """更新数据"""
        # print(f"exchange_task_over_signal:{self.action_plan.exchange_task_over_signal}\nsave_exchange_file:{self.action_plan.save_exchange_file}\nsave_over_exchnage:{self.action_plan.save_over_exchnage}")
        # print(f"duikong_task_over_signal:{self.action_plan.duikong_task_over_signal}\nsave_duikong_file:{self.action_plan.save_duikong_file}\nsave_duikong_over:{self.action_plan.save_duikong_over}")
        angle_qpos_robot_num_1=self.robot.get_state(model='joint',robot_num=1)
        angle_qpos_robot_num_2=self.robot.get_state(model='joint',robot_num=2)

        radius_qpos_robot_num_1 = [math.radians(j) for j in angle_qpos_robot_num_1]
        radius_qpos_robot_num_2 = [math.radians(j) for j in angle_qpos_robot_num_2]
            
        if self.gpcontrol.gpstate:
            gpstate, gppos, gpforce = map(lambda x: str(x) if not isinstance(x, str) else x, self.gpcontrol.gpstate[0])
            
            radius_qpos_robot_num_1.extend([int(gppos, 16), int(gpforce, 16)])
            gpstate, gppos, gpforce = map(lambda x: str(x) if not isinstance(x, str) else x, self.gpcontrol.gpstate[1])
            radius_qpos_robot_num_2.extend([int(gppos, 16), int(gpforce, 16)])      

        gp_data = [x for row in [radius_qpos_robot_num_1,radius_qpos_robot_num_2] for x in row]

        self.qpos_list.append(gp_data)
        # 记录图像数据
        if self.camera_frame_buffer:
            for camera_name in camera_names:
                self.images_dict[camera_name].append(self.camera_frame_buffer.get(camera_name))
        self.progress_value += 1  # 增加进度值
        
        if self.action_plan.traj_signal == 1 and self.action_plan.save is True:
            print(f"保存hdf5-1")
            self.save_data()
            self.action_plan.save = False
        elif self.action_plan.traj_signal == 2 and self.action_plan.save is True:
            print(f"保存hdf5-2")
            self.save_data()
            self.action_plan.traj_signal = 0
            self.stop_task()
        time.sleep(1 / 30)

    def save_data(self):
        """保存数据"""
        try:
            data_dict = {}
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
                # 清理图像数据
            

            data_dict['/action'] = np.array(data_dict['/action'])
            data_dict['/observations/qpos'][:,:6] = np.degrees(data_dict['/observations/qpos'][:,:6])
            data_dict['/observations/qpos'][:,8:14] = np.degrees(data_dict['/observations/qpos'][:,8:14])
            data_dict['/action'][:,:6] = np.degrees(data_dict['/action'][:,:6])
            data_dict['/action'][:,8:14] = np.degrees(data_dict['/action'][:,8:14])
            
            # 保存主文件
            if self.action_plan.traj_signal == 1:
                self.generator_hdf5.save_hdf5(data_dict, f"./hdf5_exchange_{today_str}_{self.note}", self.index,arm_name='all')
            if self.action_plan.traj_signal == 2:
                self.generator_hdf5.save_hdf5(data_dict, f"./hdf5_duikong_{today_str}_{self.note}", self.index,arm_name='left_arm')

         
            # 清理原始数据
            self.progress_value = 0
            self.qpos_list.clear()
            self.action_list.clear()
            for cam_name in camera_names:
                self.images_dict[cam_name].clear()
            # 清理摄像头帧缓存
            # 更新索引和状态
            self.task_complete_step = 0

        except Exception as e:
            print(f"Save data error: {e}")
        finally :
            if self.action_plan.traj_signal == 2:
                print("任务结束")
                self.robot.close()
                self._running.clear()  # 通知线程停止
                self.stop_task()