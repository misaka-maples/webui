import threading
import time
import random
from backend.robot import ROBOT  # 假设 ROBOT 类在这个模块中
from backend.serial_control import GripperCANController  
class ACTION_PLAN():
    def __init__(self,gpcontrol):
        self._running = threading.Event()  # 正确的运行标志
        self._thread = None
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
        self.origin_point_right = self.Robot.load_json('backend/point.json')['origin_point_right'][0]
        self.exchange_task_over_signal = False
        self.save_over_exchnage = True
        self.save_exchange_file = False
        self.save_duikong_file = False
        self.duikong_task_over_signal = False
        self.save_duikong_over = True
        self.gpcontrol = gpcontrol  # GripperCANController 实例
        self.stop_signal = False
        self.start_signal = False
        self.save = True
    def move(self, position, robot_num=1):
        if self.stop_signal:
            return
        self.Robot.set_state(position, 'pose', robot_num)

    def start_thread(self):
        if self._thread is None or not self._thread.is_alive():
            self._running.set()
            self._thread = threading.Thread(target=self.run, daemon=True)
            self._thread.start()


    def stop_thread(self):
        """停止线程采集"""
        self._running.clear()  # 通知线程退出
        if self._thread is not None:
            self._thread.join()  # 等待线程安全退出
            self._thread = None
            self.close()
    def run(self):
        while self._running.is_set():
            self.exchange_task()
            while self.save:    
                time.sleep(2)
            self.duikong()                
            self.save = True
            self.stop_thread()
        else:
            time.sleep(0.1)
    def exchange_task(self):
        self.gpcontrol.set_value_func(1, 80)
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
        self.gpcontrol.set_value_func(2, 80)
        time.sleep(2)
        self.traja_reverse_task_move(self.change_points_right[3:], robot_num=2)
        if self.stop_signal:
                return
        self.gpcontrol.set_value_func(2, 0)
        self.traja_reverse_task_move(self.change_points_right[1:3], robot_num=2)
        self.move(self.origin_point_right, robot_num=2)
        if self.stop_signal:
                return
        self.traja_task_move(self.change_points_left[3:7], robot_num=1)
        if self.stop_signal:
                return
        self.traj_signal = 1
    def duikong(self):
        self.traja_task_move(self.change_points_left[7:11], robot_num=1)
        if self.stop_signal:
                return
        self.gpcontrol.set_value_func(1, 80)
        time.sleep(4)
        self.traja_task_move(self.change_points_left[11:], robot_num=1)
        if self.stop_signal:
                return
        self.traj_signal = 2
        time.sleep(2)
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


if __name__ == "__main__":
    gp = GripperCANController()
    gp.connect_ACM_port()
    gp.start_thread()
    action_plan = ACTION_PLAN(gpcontrol=gp)
    action_plan.start_thread()
    i = 0
    while i<20:
        print(i)
        i += 1
        time.sleep(1)
        if i == 3:
            action_plan.stop_thread()