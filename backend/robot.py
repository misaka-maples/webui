from backend.tcp_tx import PersistentClient
import time
import json
class ROBOT:
    def __init__(self):
        self.joint_state_right=None
        self.Client = PersistentClient('192.168.3.15', 8001)

    def get_state(self, model='joint',robot_num=1):#pose
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
        self.Client.set_close(2)
        time.sleep(2)
        self.Client.set_clear(1)
        self.Client.set_clear(2)
        self.Client.set_open(2)
        self.Client.set_open(1)
    def stop(self,robot_num):
        self.Client.set_stop(robot_num)
        self.Client.set_reset(robot_num)
    def load_json(self, file_path):
        """加载JSON文件"""
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data