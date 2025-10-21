#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
import time
from piper_sdk import *

# 测试代码
if __name__ == "__main__":
    piper = C_PiperInterface_V2("can_arm1")
    piper.ConnectPort()
    while True:
        end_pose_data = piper.GetArmEndPoseMsgs()
        print(f"6D位姿(单位为mm和度):  x-->{end_pose_data.end_pose.X_axis/1000}  y-->{end_pose_data.end_pose.Y_axis/1000}  z-->{end_pose_data.end_pose.Z_axis/1000}  rx-->{end_pose_data.end_pose.RX_axis/1000}  ry-->{end_pose_data.end_pose.RY_axis/1000}  rz-->{end_pose_data.end_pose.RZ_axis/1000}")
        time.sleep(0.01)
    