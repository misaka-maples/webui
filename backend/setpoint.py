from backend.robot import ROBOT
import time
class temp:
    def __init__(self):
        self.robot = ROBOT()
        self.right_point = self.robot.load_json('backend/point.json')['change_points_right']
        self.left_point = self.robot.load_json('backend/point.json')['change_points_left']
        self.origin_point_right = self.robot.load_json('backend/point.json')['origin_point_right']
    def set(self,value,robot_num):
        self.robot.set_state(action=value,model='pose',robot_num=robot_num)
    def get(self):
        pose1 = self.robot.get_state('pose',1)
        print(f"左臂pose:{pose1}")
        joint1 = self.robot.get_state('joint',1)
        print(f"左臂joint:{joint1}")
        pose2 = self.robot.get_state('pose',2)
        print(f"右臂pose:{pose2}")
        joint2 = self.robot.get_state('joint',2)
        print(f"右臂joint:{joint2}")
    def enable_power(self):
        self.robot.enable_power()
if __name__ == "__main__":
    temp_ = temp()
    temp_.get()
    temp_.enable_power()
    time.sleep(0.5)   
    inx = 0
    temp_.set(temp_.origin_point_right[0],robot_num=2)
    temp_.set(temp_.left_point[inx],robot_num=1)
    # temp_.set(temp_.right_point[inx],robot_num=2)
    


