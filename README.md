# Flask 多摄像头与夹爪控制系统

本项目是一个基于 **Flask** 的 Web 控制界面，主要用于多摄像头视频流展示、夹爪控制、串口管理以及任务流程管理。前端采用 HTML/JS，后端通过 Flask 提供 RESTful API，支持多线程任务与硬件交互。

***

## 📁 项目目录结构

```
├── app.py                  # Flask 主应用入口
├── backend/                 # 后端核心逻辑与硬件控制模块
│   ├── camera.py            # 多摄像头采集与 MJPEG 视频流输出
│   ├── camera_hot_plug.py   # 摄像头热插拔支持
│   ├── serial_control.py    # 夹爪 CAN 控制与串口管理
│   ├── task.py              # 任务调度、数据采集与保存
│   ├── robot.py             # 机器人底层动作控制接口
│   ├── action_plan.py       # 动作/任务规划与执行
│   ├── setpoint.py          # 点位设置与获取功能
│   ├── utils.py             # 工具函数封装
│   ├── generator_hdf5.py    # 实验数据 HDF5 格式保存
│   ├── act_gp_qt.py         # Qt 界面、动作规划与夹爪控制集成
│   ├── tcp_tx.py            # TCP 通信模块
│   ├── config/              # 系统配置文件目录
│   └── point.json           # 机器人点位配置
├── static/                 # 前端静态资源（JS/CSS/图片）
│   ├── omnipicker.js       # 前端夹爪交互脚本
│   ├── camera.js           # 前端相机和task交互脚本
│   └── style.css           # 网页风格配置
├── templates/              # 前端模板（HTML）
│   └── index.html          
├── pyorbbecsdk/            # 相机编译依赖库
└── ...
```

---

## 📦 安装依赖

建议使用 **Python 3.8+**

基础依赖（最小可运行）：

pip install flask
```

如需串口、摄像头等功能，请根据实际硬件环境安装额外依赖，例如：

pip install pyserial opencv-python pillow hdf5;

## 🚀 启动方式

在项目根目录下运行：

python app.py

默认启动地址：<http://127.0.0.1:5000/>

## 🔧 主要功能

*   🎥 **多摄像头视频流**

    *   支持多路摄像头 MJPEG 实时预览
*   🤖 **夹爪控制**

    *   支持多夹爪独立控制，串口控制开合
*   🔌 **串口管理**

    *   串口端口枚举、连接、重连等操作
*   📋 **任务流程管理**

    *   启动/停止任务，自动采集数据

***

## 🌐 前端交互接口

前端通过 `omnipicker.js` 与后端 API 通信，常用接口包括：

### 📦 任务与摄像头控制

GET /action/&lt;action&gt;

🔌 串口管理

GET /serial/ports         # 枚举串口
POST /serial/open         # 打开串口
POST /serial/reconnect    # 重连串口

🤖 夹爪控制

POST /gripper/start
POST /gripper/stop
POST /gripper/\<gripper\_id>  # 控制指定夹爪开合

***

## 🤝 贡献与反馈

欢迎反馈问题与贡献代码！

* 提交 Issue 或建议
* 提交 Pull Request 合并代码

***

## 📄 License

本项目遵循 MIT License 开源协议。

***

