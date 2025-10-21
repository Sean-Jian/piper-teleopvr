#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("arm_r")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    factor = 57295.7795 #1000*180/3.1415926
    position = [0,0,0,0,0,0,0]
    
    joint_0 = round(position[0]*factor)
    joint_1 = round(position[1]*factor)
    joint_2 = round(position[2]*factor)
    joint_3 = round(position[3]*factor)
    joint_4 = round(position[4]*factor)
    joint_5 = round(position[5]*factor)
    joint_6 = round(position[6]*1000*1000)
    piper.ModeCtrl(1,1,100,0)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    time.sleep(3)
    print("start...")
    # current_joints = piper.GetArmJointMsgs().joint_state#读取当前关节角
    # #只动J5,J6
    # piper.JointCtrl(current_joints.joint_1,
    #                 current_joints.joint_2,
    #                 current_joints.joint_3,
    #                 current_joints.joint_4,
    #                 -45*1000,
    #                 +45*1000,
    #                 )
    currEndPose = piper.GetArmEndPoseMsgs().end_pose
    print(f"x:{currEndPose.X_axis/1000/10}cm,y:{currEndPose.Y_axis/1000/10}cm,z:{currEndPose.Z_axis/1000/10}cm,rx:{currEndPose.RX_axis/1000}度,ry:{currEndPose.RY_axis/1000}度,rz:{currEndPose.RZ_axis/1000}度")
    piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
    piper.EndPoseCtrl(
        currEndPose.X_axis+80*1000,
        currEndPose.Y_axis+40*1000,
        currEndPose.Z_axis+80*1000, 
        currEndPose.RX_axis, 
        currEndPose.RY_axis, 
        currEndPose.RZ_axis
                        )
    time.sleep(1)
    currEndPose = piper.GetArmEndPoseMsgs().end_pose
    print(f"x:{currEndPose.X_axis/1000/10}cm,y:{currEndPose.Y_axis/1000/10}cm,z:{currEndPose.Z_axis/1000/10}cm,rx:{currEndPose.RX_axis/1000}度,ry:{currEndPose.RY_axis/1000}度,rz:{currEndPose.RZ_axis/1000}度")
