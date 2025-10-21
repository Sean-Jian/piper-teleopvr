#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
# 使能机械臂
import time
from piper_sdk import *
        
# 测试代码
if __name__ == "__main__":
    pos = [177*1000, -2.4*1000, 216.1*1000, -176.1*1000, 16.8*1000, -161.8*1000]


    # piper = C_PiperInterface_V2("can_arm_left")
    # piper.ConnectPort()
    # time.sleep(0.1)
    # while( not piper.EnablePiper()):
    #     time.sleep(0.01)


    

    piper1 = C_PiperInterface_V2("can0")
    piper1.ConnectPort()
    time.sleep(0.1)
    while( not piper1.EnablePiper()):
        time.sleep(0.01)

    print("使能成功!!!!")
    # piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
    # piper.EndPoseCtrl(pos[0],pos[1],pos[2],pos[3],pos[5],pos[5])

    # piper1.MotionCtrl_2(0x01, 0x00, 100, 0x00)
    # piper1.EndPoseCtrl(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])
    
    