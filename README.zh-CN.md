# Piper VR 遥操作

<p align="center">
  <a href="./README.md"><img alt="English" src="https://img.shields.io/badge/English-README-0969da"></a>
  <a href="./README.zh-CN.md"><img alt="简体中文" src="https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-README-d73a49"></a>
</p>

这是一个面向两台松灵 Piper 六自由度机械臂的 WebXR 遥操作系统。系统通过 WebSocket 接收 VR 手柄位姿与按键状态，将手柄增量运动映射为机械臂末端目标，完成逆运动学求解后，通过 Piper SDK 控制左右机械臂。

<p align="center">
  <img src="media/piper_demo.gif" alt="Piper 双臂 VR 遥操作演示" width="600">
</p>

## 项目亮点

- 基于浏览器的 WebXR 客户端，头显无需安装原生应用
- 两台 Piper 机械臂可独立、同步控制
- Grip 侧键作为运动安全开关，Trigger 控制夹爪
- 基于 CasADi/Pinocchio 的逆运动学及关节跳变检测
- 支持低通滤波、IK 超时回退以及上电/回零插值
- 通过局域网 HTTPS 与安全 WebSocket 通信

## 系统架构

~~~mermaid
flowchart LR
    V[VR 头显<br/>WebXR 手柄] -->|HTTPS :8443| W[Web 控制界面]
    W -->|位姿与按键<br/>WSS :8442| S[VR WebSocket 服务]
    S --> C[手柄状态<br/>Grip / Trigger / A / X]
    C --> M[增量位姿映射<br/>VR 坐标系到机器人坐标系]
    M --> I[双臂逆运动学<br/>CasADi + Pinocchio]
    I --> F[安全与平滑处理<br/>跳变检测 / 超时回退 / 低通滤波]
    F --> P[Piper SDK<br/>MIT 关节与夹爪指令]
    P --> L[左臂 CAN<br/>arm_l]
    P --> R[右臂 CAN<br/>arm_r]
    L --> LA[左侧 Piper]
    R --> RA[右侧 Piper]
    LA -->|关节及位姿反馈| H[状态与回零插值]
    RA -->|关节及位姿反馈| H
    H --> M
    C -->|松开 Grip 后按 A 或 X| H
~~~

按住 Grip 时，系统记录当前手柄与机械臂位姿；之后仅映射相对运动，因此松开再按下 Grip 的效果类似离合器。左右臂目标分别转换到机器人坐标系并独立求解 IK。有效关节目标经低通滤波后通过 Piper SDK 的 MIT 模式发送；IK 超时或失败时使用上一次成功指令。上电和 A/X 回零过程采用关节插值，避免突然大范围运动。

## 环境要求

- 两台松灵 Piper 机械臂
- 支持 WebXR 的 VR 头显及手柄
- 与头显处于同一局域网的 Linux PC
- 名为 arm_l 和 arm_r 的两个 CAN 接口
- 推荐 Python 3.10.18

主要依赖包括 [LeRobot](https://github.com/huggingface/lerobot)、[Piper SDK](https://github.com/agilexrobotics/piper_sdk)、NumPy、SciPy、CasADi、Pinocchio、WebSockets 和 PyYAML。

## 安装

安装 69901b9b 提交版本的 LeRobot：

~~~bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout 69901b9b
pip install -e .
~~~

安装 0.4.3 版本的 Piper SDK：

~~~bash
git clone https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk
git checkout 0.4.3
pip install .
~~~

安装本项目依赖：

~~~bash
cd telegrip
pip install -r requirements.txt
~~~

Piper IK 模块还会导入 CasADi 和 Pinocchio；如果当前环境没有提供，请额外安装相互兼容的版本。

## CAN 配置

按照 Piper SDK 文档配置并激活两个适配器。如果软件源中的 can-utils 版本导致通信失败，可手动编译较新版本：

~~~bash
cd /tmp
git clone https://github.com/linux-can/can-utils.git
cd can-utils
make
sudo make install
~~~

启动 VR 遥操作前，建议先运行 telegrip/telegrip/piper_demo_V2/ 下的示例，确认两台机械臂均可正常通信和控制。

## 配置与运行

修改 telegrip/telegrip/main_vr_control_piper_V8_mit_interpolation.py 中的 URDF 绝对路径 urdf_path，并确认 URDF 内引用的网格路径均可解析。然后运行：

~~~bash
cd telegrip
python telegrip/main_vr_control_piper_V8_mit_interpolation.py
~~~

将头显和 PC 接入同一局域网，在头显浏览器中打开：

~~~text
https://<PC-IP>:8443
~~~

如浏览器提示本地证书风险，请确认地址后允许访问，再点击开始控制。手柄数据通过 WSS 8442 端口传输。如果中途摘下头显，建议刷新页面后再继续。

## 手柄映射

| 手柄输入 | 功能 |
| --- | --- |
| 按住左/右 Grip | 启用对应机械臂运动 |
| 松开 Grip | 停止更新对应机械臂目标 |
| 按住 Grip 时扣住 Trigger | 闭合对应夹爪 |
| 松开 Grip 后按左手 X | 左臂插值返回初始位姿 |
| 松开 Grip 后按右手 A | 右臂插值返回初始位姿 |

> [!CAUTION]
> 本软件会直接控制真实机器人。使能前请核对 CAN 映射、URDF 路径、工作空间和急停装置；首次运行时应降低速度、安排人员监护，并确保机械臂运动范围内无人。

## VR 手柄数据

<p align="center">
  <img src="vr_info.jpg" alt="VR 手柄信息" width="600">
</p>

## 开源协议

详见 [LICENSE](./LICENSE)。
