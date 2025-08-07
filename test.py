import threading
import time

class ManagedThread(threading.Thread):
    def __init__(self, thread_name, interval=1):
        super().__init__(name=thread_name)
        self._running = threading.Event()
        self._running.set()  # 线程启动时，设定为运行状态
        self._interval = interval  # 控制线程执行的时间间隔
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():  # 只要没有收到停止信号就执行
            if self._running.is_set():  # 如果线程被标记为运行状态
                print(f"{self.name} is running.")
                time.sleep(self._interval)  # 模拟任务执行
            else:
                time.sleep(0.1)  # 如果线程暂停，休眠一会避免浪费CPU资源

    def stop(self):
        self._stop_event.set()  # 设置停止事件，退出线程循环

    def start_running(self):
        self._running.set()  # 设置为运行状态

    def pause_running(self):
        self._running.clear()  # 设置为暂停状态
def test(thread1):
    print("This is a test function.")
    print("This is a test function.")
    print("This is a test function.")
    print("This is a test function.")
    if thread1.is_alive():
        print("Stopping thread1 from test function.")
        thread1.pause_running()
        return
    print("This is a test function.")
    print("This is a test function.")
def test1():
    print("This is a test function.")
    print("This is a test function.")
    print("This is a test function.")
    print("This is a test function.")
    print("This is a test function.")
    print("This is a test function.")
def test2(thread1):
    test(thread1)
    test1()

# 使用示例
def main():
    # 创建一个线程并启动
    thread1 = ManagedThread("Thread-1")
    thread1.start()

    # 主线程模拟切换操作
    time.sleep(2)
    print("Pausing thread")
    # thread1.stop()
    thread1.pause_running()
    # test2(thread1)
    time.sleep(2)
    print("Resuming thread")
    thread1.start_running()

    time.sleep(5)
    print("Stopping thread")
    thread1.stop()

    # 等待线程结束
    thread1.join()

if __name__ == "__main__":
    main()
