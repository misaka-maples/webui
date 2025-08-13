import cv2
import threading
import time
import numpy as np
from backend.camera_hot_plug import CAMERA_HOT_PLUG
global_color_width, global_color_height = 640,480  # 设定默认分辨率

class MultiCameraStreamer:
    def __init__(self):
        self.camera = CAMERA_HOT_PLUG()
        self._running = threading.Event()  # 正确的运行标志
        self._thread = None
        print("Camera initialized")
        self.frame_buffer = {
            "left_wrist": None,
            "top": None,
            "right_wrist": None,
        }
        self.camera_sn_list = {
            "top": 'CP1Z842000J1',
            "left_wrist": 'CP253530006L',
            "right_wrist": 'CP3294Y0002T',
        }
        self.running = True
        self.message = "相机已启动"

    def generate_random_color_frame(self, width=640, height=480):
        """生成一张随机颜色图像"""
        return np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

    def _update_camera_frames(self):
        """后台线程：采集图像帧并存入 frame_buffer"""
        while self.running:
            try:
                # 获取图像
                color_image_dict, depth_image_dict, color_width, color_height = self.camera.get_images()
            except Exception as e:
                # 相机无法获取图像，生成随机图像代替
                color_image_dict = {}
                # print(f"[WARN] 相机图像获取失败，使用随机图像替代: {e}")  # <--- 修改处

            for cam_name in self.frame_buffer.keys():
                camera_sn = self.camera_sn_list.get(cam_name)
                frame = None

                if (
                    camera_sn in color_image_dict and
                    color_image_dict[camera_sn] is not None and
                    color_image_dict[camera_sn].size != 0
                ):
                    frame = np.array(color_image_dict[camera_sn], dtype=np.uint8)
                    frame = cv2.resize(frame, (global_color_width, global_color_height))
                    self.message = "相机图像获取成功"
                if frame is None or frame.size == 0:
                    frame = self.generate_random_color_frame()  # <--- 修改处
                    self.message = "相机图像获取失败，使用随机图像替代"
                self.frame_buffer[cam_name] = frame

            time.sleep(1 / 30)


    def start_thread(self):
        if self._thread is None or not self._thread.is_alive():
            self._running.set()
            self._thread = threading.Thread(target=self._update_camera_frames, daemon=True)
            self._thread.start()
        return True, "控制线程已启动"


    def stop_thread(self):
        """停止线程采集"""
        self._running.clear()  # 通知线程退出
        if self._thread is not None:
            self._thread.join()  # 等待线程安全退出
            self._thread = None
    def get_frame(self, camera_name):
        """获取某个摄像头的最新图像帧"""
        return self.frame_buffer.get(camera_name)

    def generate_mjpeg(self, camera_name):
        """生成 MJPEG 数据流"""
        while True:
            frame = self.frame_buffer.get(camera_name)
            if frame is None:
                time.sleep(0.01)
                continue
            resized_frame = cv2.resize(frame, (320, 240))
            ret, jpeg = cv2.imencode('.jpg', resized_frame)
            if not ret:
                time.sleep(0.01)
                continue
            time.sleep(1 / 15)
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n'
            )
