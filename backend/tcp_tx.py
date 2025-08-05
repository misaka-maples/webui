import socket
import threading
import re
import time

class PersistentClient:
    HEADER = b'&'
    FOOTER = b'^'
    ENCODING = 'utf-8'

    RECV_TIMEOUT = 10         # 🟢 读取超6时时间

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None  # 连接对象
        self.connected = False  # 连接状态
        self.recive_data = None  # 接收数据
        # 建立初始连接
        self.connect()

        #初始参数
        self.vel = 100  #速度
        self.acc = 100  #加速度
        self.dcc = 100  #减速度
        self.velocity = 30
        # self._receive_thread = threading.Thread(target=self._receive_data, daemon=True)
        # self._receive_thread.start()  # 启动接收线程  
          
    def _frame_data(self, data):
        """封装数据包（增加协议头和尾部）"""
        if not isinstance(data, bytes):
            data = data.encode(self.ENCODING)
        return self.HEADER + data + self.FOOTER

    def connect(self):
        """建立长连接"""
        if self.sock:
            self.sock.close() 

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)  # 连接超时

        try:
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(self.RECV_TIMEOUT)  # 🟢 设置 `recv()` 超时
            self.connected = True
            print("[INFO] 成功建立长连接")
        except (ConnectionRefusedError, TimeoutError) as e:
            print(f"[ERROR] 连接失败: {e}")
            self.connected = False

    def send_message(self, message):
        """发送数据（仅发送，写入线程）"""
        if not self.connected:
            print("[WARNING] 连接已断开，正在尝试重新连接...")
            self.connect()

        try:
            framed_data = self._frame_data(message)
            # print(framed_data)
            self.sock.sendall(framed_data)

            return True
        except (BrokenPipeError, ConnectionResetError) as e:
            print(f"[ERROR] 连接断开: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"[ERROR] 未知错误: {e}")
            return False

    def _receive_data(self,robotnum):
        """实时接收数据（独立线程，不影响写入）"""
        data = None
        while True:
            # start_time = time.time()  # 记录开始时间
            # print("开始接收数据")
            if self.connected:
                try:
                    data = self.sock.recv(1024)
                    data = data.decode(self.ENCODING)
                    if not data:
                        print("[WARNING] 接收数据为空，继续监听...")
                        continue
                    # print(f"接收到的数据：{data}")
                    if "readyToNext" not in data :
                        match = re.search(r'robotId:(\d+)', data)
                        id_num = int(match.group(1)) if match else None
                        if id_num != robotnum:
                            print(f"接收数据的ID号不匹配：{id_num} != {robotnum}")
                            continue
                    else:
                        # print(f"接收到readyToNext数据")
                        match = re.search(r"readyToNext,(\d+)", data)  # 匹配逗号后面的数字
                        id_num_ready = int(match.group(1)) if match else None
                        if id_num_ready != robotnum:
                            print(f"接收数据的ID号不匹配：{id_num_ready} != {robotnum}")
                            continue
                except socket.timeout:
                    print("[WARNING] 接收超时，继续监听...")
                    # break  # 超时后继续监听
                except (ConnectionResetError, BrokenPipeError):
                    print("[WARNING] 服务器断开连接，正在重连...")
                    self.connected = False
                    self.connect()
                self.recive_data = data
                # time.sleep(0.01)  # 🟢 等待0.01秒，避免过快循环
                return self.recive_data
            
            
    def close(self):
        """关闭连接"""
        if self.sock:
            self.sock.close()
            self.connected = False
            print("[INFO] 连接已关闭")
    def set_speed(self,robotnum,velocity):
        message = f"speed,{robotnum},{velocity}"
        self.send_message(message)
    def set_open(self,robotnum):
        message = f"open,{robotnum}"
        self.send_message(message)


    def set_stop(self,robotnum):
        message = f"stop,{robotnum}"
        self.send_message(message)

    def set_clear(self,robotnum):
        message = f"clear,{robotnum}"
        self.send_message(message)
    def set_close(self,robotnum):
        message = f"close,{robotnum}"
        self.send_message(message)
    def set_reset(self,robotnum):
        message = f"reset,{robotnum}"
        self.send_message(message)
    def is_close(self,actual,target,tolerance=1):
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
            # print(actual, target, abs(a - t))
            if abs(a - t) > tolerance:
                return False
        return True

    def set_arm_position(self, value:list, model:str,robotnum:str, timeout=5):
        """
        设置机械臂位置（关节模式或位姿模式），阻塞直到收到完成信号

        :param value: 目标位置列表（关节角或位姿坐标）
        :param model: 模式选择，可选 "joint"（关节） 或 "pose"（位姿）
        :param timeout: 超时时间（秒）
        :return: 成功返回True，超时或失败返回False
        """
        # 参数校验
        if not isinstance(value, list) or len(value) != 6:
            print("[ERROR] 输入必须是包含6个数值的列表")
            return False
        # print(f"id:{robotnum}")
        if model not in ["joint", "pose"]:
            print("[ERROR] 模式参数必须是 'joint' 或 'pose'")
            return False
        # print("进入近似点判断")
        if self.is_close(self.get_arm_position_pose(robotnum),value):
            print('[INFO] 当前位置与目标位置相等')
            return False
        try:
            # 构造命令字符串
            value_str = ",".join(f"{x:.4f}" for x in value)
            cmd_type = "ACS" if model == "joint" else "PCS"
            command = f"set,{robotnum},{self.vel},{cmd_type},0,0,{value_str},0,{self.acc},{self.dcc}"

            # 发送命令
            if not self.send_message(command):
                return False
            # finsh = 0
            # print(f"发送数据：{command}")
            # 阻塞等待响应
            while True:
                # 读取接收数据（假设有非阻塞读取方法）
                data = self._receive_data(robotnum)
                # print(f"接收数据:{data}")
                # time.sleep(0.01)
                # print(f'wating data')
                if "readyToNext" in data:  # 根据实际的返回数据格式修改
                    
                    # print(f"data:{data}")
                    break
        except Exception as e:
            print(f"[ERROR] 设置位置失败: {e}")
            return False

    def get_arm_position_joint(self,robotnum):
            """
            获取机械臂位姿：
            发送请求后等待缓冲区中出现响应数据，最多等待5秒。
            :return: 返回接收到的字符串响应数据，或None（超时）
            """
            while True:

                message = f"get,{robotnum},ACS"
                # print("发送数据：", message)
                self.send_message(message)
                # print("发送数据成功")
                # response = self.recive_data
                response = self._receive_data(robotnum)
                # print("get_arm_position_joint接收到的数据：", response)
                if response == None:
                    print("[ERROR] get超时，继续等待数据...")
                    continue
                if  "readyToNext" in response:
                    continue
                else:
                    match = re.search(r'getPos:"([^"]+)"', response)
                    # print("匹配结果：", match)
                    if match:
                        # 提取并解析六个浮动数据
                        data_string = match.group(1)  # 获取 "2,0,0,0,-7.2092,133.368,500.813,-1.63063,-0.0261585,-1.57236,0"
                        data_list = data_string.split(',')[4:10]  # 获取从第5到第10个数据（索引从0开始）
                        # 将数据转换为浮动数并返回
                        return [float(i) for i in data_list]
                    else:
                        print("[ERROR] 无法解析位置数据")
                        break
    def get_arm_position_pose(self, robotnum):
        """
        获取机械臂位姿：
        发送请求后等待缓冲区中出现响应数据，最多等待5秒。
        :return: 返回接收到的字符串响应数据，或None（超时）
        """
        while True:
            message = f"get,{robotnum},PCS"
            message = message.strip()
            self.send_message(message)
            # print("发送数据成功")
            response = self._receive_data(robotnum)
            # response = self.recive_data
            # print("接收到的数据：", response)
            match = re.search(r'getPos:"([^"]+)"', response)
            if match:
                # 提取并解析六个浮动数据
                data_string = match.group(1)  # 获取 "2,0,0,0,-7.2092,133.368,500.813,-1.63063,-0.0261585,-1.57236,0"
                data_list = data_string.split(',')[4:10]  # 获取从第5到第10个数据（索引从0开始）
                # 将数据转换为浮动数并返回
                # print(f"匹配到的数据：{data_list}")
                return [float(i) for i in data_list]
            else:
                print("[ERROR] 无法解析位置数据")
                # return None
        # 返回接收到的响应字符串

if __name__ == "__main__":
    client = PersistentClient('192.168.3.15', 8001)
    client.set_close(2)
    client.set_close(1)
    time.sleep(2)

    client.set_clear(2)
    client.set_open(2)  
    
    client.set_clear(1)
    client.set_open(1) 
    change_points_left = [
                   
                    [-127.205, -584.918, -238.196, 2.8373, -0.121059, 1.07089],#左手，先开爪
                   [-122.71, -575.238, -306.627, 2.51576, -0.107987, 0.866255],
                    [-124.89, -608.516, -345.528, 2.41849, 0.0261467, 0.859578],#关闭左，开右夹爪
                   [-126.071, -676.698, -177.695, 2.5211, -0.176558, 1.30888],
                   [-124.581, -752.295, -212.366, 2.47183, -0.00239102, 2.28396],
                    [-129.127, -810.615, -288.951, 2.4716, -0.00248988, 2.28385],
                     [-109.0, -811.015, -288.718, 2.47157, -0.00245327, 2.28383],
                    [-129.127, -810.615, -288.951, 2.4716, -0.00248988, 2.28385],
                   [-124.961, -813.379, -286.805, 2.47164, -0.00239282, 2.28384],
                    [-41.0751, -734.438, 207.143, 2.28154, 0.145922, 2.31647],
                    [629.137, -161.689, 590.811, 1.6477, 1.38221, 2.1665],
                    [-41.0751, -734.438, 207.143, 2.28154, 0.145922, 2.31647],
                    [-127.205, -584.918, -238.196, 2.8373, -0.121059, 1.07089]
                       ]  # 存储所有点位
    while True:
        try:
            message = input("> ")  # 🟢 从命令行获取输入1
            if message.lower() == "exit":
                print("[INFO] 退出客户端...")
                client.close()
                break
            if message == "up charge":
                client.set_close(2)
                client.set_close(1)
                time.sleep(2)

                client.set_clear(2)
                client.set_open(2)  
                
                client.set_clear(1)
                client.set_open(1)       
            if message == "get1":
                joint_pose = client.get_arm_position_joint(1)
                pose = client.get_arm_position_pose(1)
                print(joint_pose,"\n",pose)
                time.sleep(0.1)
            if message == "get2":
                joint_pose = client.get_arm_position_joint(2)
                pose = client.get_arm_position_pose(2)
                print(joint_pose,"\n",pose)
                time.sleep(0.1)

            if message == "1,4":
                client.set_arm_position([-104.292, -14.1806, 50.2702, -8.32077, -96.7339, -40.532] ,"joint",1)
                client.set_arm_position([93.7621, 3.73672, 50.7112, 10.4171, -32.9198, 3.26294] ,"joint",2)
            if message == "1,5":
                client.set_arm_position([-129.127, -810.615, -288.951, 2.4716, -0.00248988, 2.28385],"pose",1)
            if message == "set speed":
                client.set_speed(1,30)
                time.sleep(2)
                client.set_speed(2,30)
            if message == "2":
                client.set_arm_position( [-127.205, -584.918, -238.196, 2.8373, -0.121059, 1.07089],"pose",1)
            if message == "2,2":
                deta = client.set_arm_position([320.871, 8.02429, 569.908, -2.77289, 1.54682, -0.36409],"pose",2)
            if message == "2,3":
                deta = client.set_arm_position([-77.3069, 482.59, 523.98, -1.74372, -0.200726, -1.37602],"pose",2)



            if message == "3":
                client.set_arm_position( [-132.427, 620.664, -172.781, -2.37856, -0.125904, -3.09914], "pose", 2) 
            if message == "4,2":
                client.set_arm_position([-118.023, 668.517, -105.539, -2.20681, -0.119449, -2.58369],"pose",2)
            if message == "4,4":
                for _ in range(400):
                    client.set_arm_position([320.871, 8.02429, 569.908, -2.77289, 1.54682, -0.36409],"pose",2)
                    client.set_arm_position([-77.3069, 482.59, 523.98, -1.74372, -0.200726, -1.37602],"pose",2)
                    client.set_arm_position([-134.846, 623.718, -169.467, -2.37875, -0.126138, -3.0992],"pose",2) 
                    client.set_arm_position([-77.3069, 482.59, 523.98, -1.74372, -0.200726, -1.37602],"pose",2)
                    # client.set_arm_position([320.871, 8.02429, 569.908, -2.77289, 1.54682, -0.36409],"pose",2)
            if message == "setspeed":
                client.set_speed(1,15)
                client.set_speed(2,15)

            if message == "4":
                client.set_close(1)
                time.sleep(1)
                client.set_clear(1)
                client.set_open(1) 
                # client.get_arm_position_pose(2)
                # deta = client.set_arm_position([-124.602, -752.314, -212.362, 2.47183, -0.00238126, 1.49759], "pose", 1)
            if message == "4.1":
                deta = client.set_arm_position([-122.668, -810.366, -283.547, 2.47194, -0.00226187, 1.49765], "pose", 1)
                print(deta)
            if message == "4.2":
                deta = client.set_arm_position([-104.753, -812.254, -283.736, 2.47097, 0.048749, 1.49756], "pose", 1)
                print(deta)
            if message == "4.3":
                deta = client.set_arm_position([-112.935, -749.491, -200.365, 2.50606, 0.0133301, 1.49744], "pose", 1)
                print(deta)
            if message == "4.4":
                deta = client.set_arm_position([-41.108, -734.451, 207.087, 2.28157, 0.145945, 1.33637], "pose", 1)
                print(deta)
            if message == "0":
                deta = client.get_arm_position_pose(2)
                print(deta)
            if message == "5":
                deta = client.send_message("stop,1")
            if message == "6":
                deta = client.send_message("open,1")
    
            if message == "7":
                for point in change_points_left:
                    print(point)
                    client.set_arm_position(point,"pose",1)
    
        except KeyboardInterrupt:
            print("\n[INFO] 终止客户端...")
            # client.set_stop(1)
            client.close()
            break
