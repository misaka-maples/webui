from flask import Flask, render_template, request, jsonify, Response
from backend.camera import MultiCameraStreamer
from backend.serial_control import GripperCANController
from backend.task import TaskManager 
import sys
GripperCANcontroller = GripperCANController()
camera = MultiCameraStreamer()
taskmanager = TaskManager(camera)
camera_running = False
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed/<camera_name>')
def video_feed(camera_name):
    global camera_running
    with camera.lock:
        if not camera_running:
            return jsonify({"error": "Camera is not running"}), 400
        return Response(camera.generate_mjpeg(camera_name), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/submit_note', methods=['POST'])
def submit_note():
    data = request.get_json()
    note = data.get('note', '')
    taskmanager.note = note  # 保存备注到任务管理器
    print(f"收到备注：{note}")

    return jsonify({'message': '已收到备注'})
@app.route('/set_index', methods=['POST'])
def set_index():
    data = request.get_json()
    index = data.get('index', '')
    taskmanager.index = index  # 保存index到任务管理器
    taskmanager.index = int(index) if index.isdigit() else 0

    print(f"收到index：{index}")

    return jsonify({'message': '已收到index'})


@app.route('/action/<action>', methods=['POST'])
def handle_action(action):
    global camera_running
    data = request.get_json()
    user_input = data.get("input", "")

    if action == "start_task":
        print(f"Starting task with index: {user_input}")
        taskmanager.start_task()
        return jsonify({"message": "任务已开始"})
    elif action == "stop_task":
        print("Stopping task...")
        taskmanager.stop_task()
        return jsonify({"message": "任务已停止"})
    elif action == "start_camera":
        with camera.lock:
            if not camera_running:
                camera_running = True
        print("Starting camera...")
        return jsonify({f"message": camera.message})
    elif action == "stop_camera":
        with camera.lock:
            camera_running = False
        print("Stopping camera...")
        return jsonify({"message": "摄像头已关闭"})
    else:
        return jsonify({"message": "未知操作"}), 400

@app.route('/serial/ports')
def get_serial_ports():
    # print(GripperCANcontroller.get_serial_ports_list())
    return jsonify({"ports": GripperCANcontroller.get_serial_ports_list()})

@app.route('/serial/open', methods=['POST'])
def open_serial():
    port = request.json.get("port")
    success, msg = GripperCANcontroller.open_serial(port)
    return jsonify({"message": msg}), 200 if success else 500

@app.route('/serial/reconnect', methods=['POST'])
def reconnect_serial():
    success, msg = GripperCANcontroller.reconnect_serial_connection()
    return jsonify({"message": msg}), 200 if success else 500

@app.route('/gripper/start', methods=['POST'])
def start_gripper():
    try:        
        GripperCANcontroller.start_thread()
        return jsonify({"message": "夹爪线程已启动"})
    except Exception as e:
        return jsonify({"message": f"启动夹爪线程失败: {str(e)}"}), 500

@app.route('/gripper/stop', methods=['POST'])
def stop_gripper():
    try:
        GripperCANcontroller.pause_thread()
        # GripperCANcontroller.close()
        return jsonify({"message": "夹爪线程已停止"})
    except Exception as e:
        return jsonify({"message": f"停止夹爪线程失败: {str(e)}"}), 500

@app.route('/gripper/<int:gripper_id>', methods=['POST'])
def control_gripper(gripper_id):
    value = request.json.get("value")
    # print(f"控制夹爪 {gripper_id}，设置值: {value}")
    try:
        if not GripperCANcontroller.is_open():
            return jsonify({"message": "串口未连接"}), 400
        GripperCANcontroller.set_value_func(gripper_id, int(value))
        return jsonify({"message": f"夹爪 {gripper_id} 设置为 {value}"})
    except Exception as e:
        return jsonify({"message": f"发送失败: {str(e)}"}), 500

if __name__ == '__main__':
    # try:
        camera.start()
        GripperCANcontroller.start()
        # GripperCANcontroller.start_thread()
        app.run(debug=False)
    # except KeyboardInterrupt:
        
    #     print("\n应用程序被中断 (Ctrl+C)，正在停止...")
    #     # 停止摄像头和夹爪控制器等资源
    #     camera.stop()
    #     GripperCANcontroller.stop_thread() 
    #     sys.exit(1)  # 退出程序
    # except Exception as e :
    #     print(f"启动应用失败: {e}")
    #     camera.stop()
        # sys.exit(1)  # 错误退出