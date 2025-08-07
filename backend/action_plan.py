import threading
import time
import random
from backend.robot import ROBOT  # 假设 ROBOT 类在这个模块中
# from backend.serial_control import GripperCANController
class ACTION_PLAN(threading.Thread):
    def __init__(self,gpcontrol):
        super().__init__()
        self._running = threading.Event()  # 正确的运行标志
        self._stop_event = threading.Event()
        self.Robot = ROBOT()
        self.velocity = 15
        self.point = None
        self.goal_point = None
        self.local_desktop_point = None
        self.loop_len = 1
        self.complete = False
        self.robot_num = 1
        self.task_name = 'exchange'
        self.traj_signal = 0
        self.Robot.Client.set_speed(1,30)
        self.Robot.Client.set_speed(2,30)
        self.change_points_right = self.Robot.load_json('backend/point.json')['change_points_right']
        self.change_points_left = self.Robot.load_json('backend/point.json')['change_points_left']
        self.exchange_task_over_signal = False
        self.save_over_exchnage = True
        self.save_exchange_file = False
        self.save_duikong_file = False
        self.duikong_task_over_signal = False
        self.save_duikong_over = True
        self.gpcontrol = gpcontrol  # GripperCANController 实例
        self.stop_signal = False
        self.start_signal = False
    def move(self, position, robot_num=1):
        if self.stop_signal:
            return
        self.Robot.set_state(position, 'pose', robot_num)
    def start(self):
        """启动后台采集线程"""
        self._running.set()
        self.stop_signal = False
        print("动作计划线程已启动")
        if self.start_signal is False:
            super().start()
            self.start_signal = True

    def stop(self):
        """停止线程"""
        if self.is_alive():
            self.stop_signal = True
            self._running.clear()  # 设置停止事件，退出线程循环

    def run(self):
        while not self._stop_event.is_set():
            if self._running.is_set():
                if self.exchange_task_over_signal is False:
                    self.exchange_task()
                time.sleep(2)
                if self.save_exchange_file:
                    continue
                self.duikong()
                # time.sleep(100)
                
                self.exchange_task_over_signal = False
                self.stop()
            else:
                time.sleep(0.1)
    def exchange_task(self):
        self.gpcontrol.set_value_func(1, 100)
        time.sleep(2)
        self.gpcontrol.set_value_func(2, 0)
        
        self.traja_task_move(self.change_points_right, robot_num=2)
        if self.stop_signal:
                return
        self.traja_task_move(self.change_points_left[:3], robot_num=1)
        if self.stop_signal:
                return
        self.gpcontrol.set_value_func(1, 0)
        time.sleep(2)
        self.gpcontrol.set_value_func(2, 100)
        time.sleep(2)
        self.traja_reverse_task_move(self.change_points_right[3:], robot_num=2)
        if self.stop_signal:
                return
        self.gpcontrol.set_value_func(2, 0)
        self.traja_reverse_task_move(self.change_points_right[:3], robot_num=2)
        if self.stop_signal:
                return
        self.traja_task_move(self.change_points_left[3:7], robot_num=1)
        if self.stop_signal:
                return
        self.traj_signal = 1
        self.exchange_task_over_signal = True
        self.save_exchange_file = True
        self.save_over_exchnage = False
    def duikong(self):
        self.traja_task_move(self.change_points_left[7:11], robot_num=1)
        if self.stop_signal:
                return
        self.gpcontrol.set_value_func(1, 100)
        time.sleep(4)
        self.traja_task_move(self.change_points_left[11:], robot_num=1)
        if self.stop_signal:
                return
        self.traj_signal = 2
        self.duikong_task_over_signal = True
        self.save_duikong_file = True
        self.save_duikong_over = False
    def traja_task_move(self, points, robot_num):
        for idx, point in enumerate(points):
            if self.stop_signal:
                return  # 立即返回
            if self.is_close(self.Robot.get_state(model='pose'), point):
                continue
            print(f"Moving to point {idx + 1}/{len(points)}: {point}")
            if point is not None:
                if idx >= len(points) - 2:
                    self.move(point, robot_num=robot_num)
                else:
                    self.move(point, robot_num=robot_num)
            if self.stop_signal:
                return
        time.sleep(1)

    def traja_reverse_task_move(self, points, robot_num):
        for point in reversed(points):
            if self.stop_signal:
                return
            if self.is_close(self.Robot.get_state(model='pose'), point):
                continue
            if point is not None:
                self.move(point, robot_num=robot_num)
            if self.stop_signal:
                return
        time.sleep(1)

    def is_close(self, actual, target, tolerance=0.1):
        if actual is None or target is None or len(actual) != len(target):
            return False
        return all(abs(a - t) <= tolerance for a, t in zip(actual, target))

    def random_positon(self, point, a=25, b=30):
        return [
            point[0] + random.uniform(a, b),
            point[1] + random.uniform(a, b),
            point[2] + random.uniform(a, b),
            *point[3:]
        ]

    def set_loop_len(self, value):
        self.loop_len = value