# Piper VR Teleoperation

<p align="center">
  <a href="./README.md"><img alt="English" src="https://img.shields.io/badge/English-README-0969da"></a>
  <a href="./README.zh-CN.md"><img alt="简体中文" src="https://img.shields.io/badge/%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87-README-d73a49"></a>
</p>

A WebXR teleoperation system for two AgileX Piper 6-DoF robot arms. It streams VR controller poses and buttons over WebSocket, maps incremental hand motion to end-effector targets, solves inverse kinematics, and drives both arms through the Piper SDK.

<p align="center">
  <img src="media/piper_demo.gif" alt="Piper dual-arm VR teleoperation demo" width="600">
</p>

## Highlights

- Browser-based WebXR client with no native headset app
- Independent and simultaneous control of two Piper arms
- Grip dead-man switches and trigger-controlled grippers
- CasADi/Pinocchio IK with joint-jump detection
- Low-pass filtering, IK timeout fallback, and interpolated startup/home motion
- Local HTTPS and secure WebSocket communication

## System architecture

~~~mermaid
flowchart LR
    V[VR headset<br/>WebXR controllers] -->|HTTPS :8443| W[Web UI]
    W -->|pose and buttons<br/>WSS :8442| S[VR WebSocket server]
    S --> C[Controller state<br/>grip / trigger / A / X]
    C --> M[Incremental pose mapping<br/>VR frame to robot frame]
    M --> I[Dual-arm IK<br/>CasADi + Pinocchio]
    I --> F[Safety and smoothing<br/>jump check / timeout fallback / low-pass filter]
    F --> P[Piper SDK<br/>MIT joint and gripper commands]
    P --> L[Left CAN<br/>arm_l]
    P --> R[Right CAN<br/>arm_r]
    L --> LA[Left Piper arm]
    R --> RA[Right Piper arm]
    LA -->|joint and pose feedback| H[State / home interpolation]
    RA -->|joint and pose feedback| H
    H --> M
    C -->|A or X with grip released| H
~~~

Holding grip captures the current hand and robot poses. Subsequent hand motion is applied incrementally, so releasing and pressing grip again works like a clutch. Each target is transformed into the robot frame and solved independently for the left or right arm. Valid joint targets are filtered and sent in MIT mode; slow or failed IK falls back to the last successful command. Startup and A/X home motions use joint interpolation to avoid abrupt movement.

## Requirements

- Two AgileX Piper arms
- A WebXR-capable VR headset and controllers
- A Linux PC on the same LAN as the headset
- Two CAN interfaces named arm_l and arm_r
- Python 3.10.18 recommended

Core dependencies include [LeRobot](https://github.com/huggingface/lerobot), [Piper SDK](https://github.com/agilexrobotics/piper_sdk), NumPy, SciPy, CasADi, Pinocchio, WebSockets, and PyYAML.

## Installation

Install LeRobot at commit 69901b9b:

~~~bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout 69901b9b
pip install -e .
~~~

Install Piper SDK version 0.4.3:

~~~bash
git clone https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk
git checkout 0.4.3
pip install .
~~~

Install this project's dependencies:

~~~bash
cd telegrip
pip install -r requirements.txt
~~~

The Piper IK module also imports CasADi and Pinocchio; install compatible builds if your environment does not already provide them.

## CAN setup

Configure and activate both adapters according to the Piper SDK documentation. If the packaged can-utils version causes communication failures, build a newer version:

~~~bash
cd /tmp
git clone https://github.com/linux-can/can-utils.git
cd can-utils
make
sudo make install
~~~

Test both arms with examples under telegrip/telegrip/piper_demo_V2/ before starting VR control.

## Configuration and run

Update the absolute urdf_path in telegrip/telegrip/main_vr_control_piper_V8_mit_interpolation.py and ensure the mesh paths inside the URDF resolve correctly. Then run:

~~~bash
cd telegrip
python telegrip/main_vr_control_piper_V8_mit_interpolation.py
~~~

Connect the headset and PC to the same LAN and open the following address in the headset browser:

~~~text
https://<PC-IP>:8443
~~~

Accept the local certificate warning if prompted and select the start-control button. Controller data uses WSS port 8442. If the headset is removed during a session, refresh the page before resuming.

## Controller mapping

| Input | Action |
| --- | --- |
| Hold left/right grip | Enable motion for the corresponding arm |
| Release grip | Stop updating that arm's target |
| Hold trigger while gripping | Close the corresponding gripper |
| Left X with grip released | Interpolate the left arm home |
| Right A with grip released | Interpolate the right arm home |

> [!CAUTION]
> This software commands physical robots. Verify CAN mapping, URDF paths, workspace clearance, and emergency-stop access before enabling the arms. Start at low speed with an observer and keep people outside the operating area.

## VR controller data

<p align="center">
  <img src="vr_info.jpg" alt="VR controller information" width="600">
</p>

## License

See [LICENSE](./LICENSE).
