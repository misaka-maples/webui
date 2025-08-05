from backend.robot import ROBOT
class temp:
    def __init__(self):
        self.robot = ROBOT()
        self.right_point = self.robot.load_json('backend/point.json')['change_points_right']
        self.left_point = self.robot.load_json('backend/point.json')['change_points_left']

    def set(self,value,robot_num):
        self.robot.set_state(action=value,model='pose',robot_num=robot_num)

if __name__ == "__main__":
    temp_ = temp()
    temp_.set(temp_.right_point[0],robot_num=2)
    temp_.set(temp_.left_point[0],robot_num=1)


