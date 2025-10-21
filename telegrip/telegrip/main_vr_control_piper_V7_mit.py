"""
在V6-opti代码基础上 改用mit模式 尝试优化双臂跟随慢的问题
"""
import asyncio
import time
import logging
import sys
import os
import threading
import http.server
import ssl
import socket
import json
import numpy as np
from piper_sdk import *
from scipy.linalg import inv
from scipy.spatial.transform import Rotation as R

# 添加项目路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
sys.path.insert(0, project_root)
from telegrip.config import TelegripConfig
from telegrip.inputs.vr_ws_server import VRWebSocketServer

# 导入逆解库
from URDF.piper.piper_ik_demo import ArmIK
# from URDF.piper.piper_ik_demo_tcp_offset import ArmIK_tcp
# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class APIHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for the teleoperation API."""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()
    
    def log_message(self, format, *args):
        pass  # Disable default HTTP logging
    
    def do_GET(self):
        if self.path == '/api/status':
            self.handle_status_request()
        elif self.path == '/' or self.path == '/index.html':
            self.serve_file('web-ui/index.html', 'text/html')
        elif self.path.endswith('.css'):
            self.serve_file(f'web-ui{self.path}', 'text/css')
        elif self.path.endswith('.js'):
            self.serve_file(f'web-ui{self.path}', 'application/javascript')
        elif self.path.endswith('.ico'):
            self.serve_file(self.path[1:], 'image/x-icon')
        elif self.path.endswith(('.jpg', '.jpeg')):
            self.serve_file(f'web-ui{self.path}', 'image/jpeg')
        elif self.path.endswith('.png'):
            self.serve_file(f'web-ui{self.path}', 'image/png')
        elif self.path.endswith('.gif'):
            self.serve_file(f'web-ui{self.path}', 'image/gif')
        else:
            self.send_error(404, "Not found")
    
    def do_POST(self):
        if self.path == '/api/vrdata':
            self.handle_vr_data_request()
        else:
            self.send_error(404, "Not found")
    
    def handle_status_request(self):
        try:
            if hasattr(self.server, 'api_handler') and self.server.api_handler:
                system = self.server.api_handler
                vr_connected = False
                if system.vr_server and system.vr_server.is_running:
                    vr_connected = len(system.vr_server.clients) > 0
                
                status = {
                    "vrConnected": vr_connected,
                    "clients": len(system.vr_server.clients) if vr_connected else 0
                }
                
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                response = json.dumps(status)
                self.wfile.write(response.encode('utf-8'))
            else:
                self.send_error(500, "System not available")
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            self.send_error(500, str(e))
    
    def handle_vr_data_request(self):
        try:
            if hasattr(self.server, 'api_handler') and self.server.api_handler:
                system = self.server.api_handler
                left_controller = system.vr_server.left_controller
                right_controller = system.vr_server.right_controller
                
                vr_data = {
                    "leftController": {
                        "position": left_controller.current_position or {},
                        "gripActive": left_controller.grip_active,
                        "triggerActive": left_controller.trigger_active,
                        "zAxisRotation": left_controller.z_axis_rotation,
                        "xAxisRotation": left_controller.x_axis_rotation
                    },
                    "rightController": {
                        "position": right_controller.current_position or {},
                        "gripActive": right_controller.grip_active,
                        "triggerActive": right_controller.trigger_active,
                        "zAxisRotation": right_controller.z_axis_rotation,
                        "xAxisRotation": right_controller.x_axis_rotation
                    },
                    "clients": len(system.vr_server.clients)
                }
                
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                response = json.dumps(vr_data)
                self.wfile.write(response.encode('utf-8'))
            else:
                self.send_error(500, "System not available")
        except Exception as e:
            logger.error(f"Error handling VR data request: {e}")
            self.send_error(500, str(e))
    
    def serve_file(self, filename, content_type):
        from telegrip.utils import get_absolute_path
        try:
            abs_path = get_absolute_path(filename)
            with open(abs_path, 'rb') as f:
                file_content = f.read()
            
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', len(file_content))
            self.end_headers()
            self.wfile.write(file_content)
        except FileNotFoundError:
            self.send_error(404, f"File {filename} not found")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            logger.debug(f"Client disconnected while serving {filename}")
        except Exception as e:
            logger.error(f"Error serving file {filename}: {e}")
            try:
                self.send_error(500, "Internal server error")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass


class SimpleVRSystem:
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.command_queue = asyncio.Queue()
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.is_running = False
        self.display_task = None
        self.https_server = None
        self.https_thread = None
        
        # 左右机械臂控制实例
        self.left_piper = None
        self.right_piper = None
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # 逆解器实例
        self.left_ik = None
        self.right_ik = None
        
        # 机械臂初始位置 (单位: mm和度)
        self.left_initial_position = [177.0, -2.4, 216.1, -176.1, 16.8, -161.8, 0]
        self.right_initial_position = [177.0, -2.4, 216.1, -176.1, 16.8, -161.8, 0]
        
        # 机械臂初始关节角 (单位: 度) - 这些将从初始位置计算得出
        self.left_initial_joints = None
        self.right_initial_joints = None
        
        # 控制频率限制
        self.last_control_time = {"left": 0.0, "right": 0.0}
        self.control_interval = 0.005  # 控制频率：200 Hz

        # 夹爪位置限制 (mm)
        self.gripper_limits = {
            "min": 0,
            "max": 80  # 大多数夹爪的最大开合范围约为100mm
        }
        
        # 齐次变换矩阵相关 - 分左右臂
        self.A0 = {"left": None, "right": None}  # 机器人初始位姿 (4x4)
        self.T = None   # 从VR坐标系到机器人坐标系的变换矩阵 (4x4) - 共享
        self.m_offset = {"left": None, "right": None}  # 偏移矩阵 (4x4)
        
        # 初始化T矩阵矩阵（VR世界到机器人世界的变换矩阵，只考虑旋转，需要根据实际情况修改）
        self.T = np.array([[0,0,-1,0],
                           [-1,0,0,0],
                           [0,1,0,0],
                           [0,0,0,1]])

        # 用于跟踪grip状态变化
        self.previous_grip_state = {"left": False, "right": False}
        
        # 存储控制器初始数据（用于grip释放时使用）
        self.grip_origin_data = {"left": None, "right": None}
        
        # 实时控制相关变量
        self.realtime_control_active = {"left": False, "right": False}

        # 低通滤波器相关参数
        self.filter_alpha = 0.3  # 滤波系数 (0-1, 越小越平滑)
        self.filtered_pose = {"left": None, "right": None}  # 滤波后的位姿

        self._cached_transform = {"left": None, "right": None}  # 为每个机械臂维护独立的矩阵计算中间结果缓存

        # IK超时计数
        self.ik_timeout_count = {"left": 0, "right": 0}
        # 添加用于缓存上一次成功IK结果的属性
        self.last_successful_joints = {"left": None, "right": None}
        self.last_successful_gripper = {"left": None, "right": None}

        #tcp偏移(米)
        self.tcp_offset = np.array([0.0, 0.0, 0.145])
    
    # 添加四元数到旋转矩阵的转换方法
    def quaternion_to_rotation_matrix(self, quaternion: np.ndarray) -> np.ndarray:
        """
        将四元数转换为旋转矩阵
        
        参数:
            quaternion: 四元数数组 [x, y, z, w]
        
        返回:
            3x3 旋转矩阵
        """
        if quaternion is None or len(quaternion) != 4:
            raise ValueError("Invalid quaternion: must be an array of 4 elements [x, y, z, w]")
        
        # 使用 scipy 创建旋转对象
        rotation = R.from_quat(quaternion)
        
        # 转换为旋转矩阵
        rotation_matrix = rotation.as_matrix()
        
        return rotation_matrix
    
    async def start(self):
        self.is_running = True
        
        # 初始化机械臂
        await self._init_robot_arms()
        
        # 启动HTTPS服务器
        await self._start_https_server()
        
        # 启动VR WebSocket服务器
        await self.vr_server.start()
        
        # 启动数据显示循环
        self.display_task = asyncio.create_task(self._display_loop())
        
        try:
            await self.display_task
        except asyncio.CancelledError:
            pass
            
    async def stop(self):
        self.is_running = False
        if self.display_task:
            self.display_task.cancel()
            try:
                await self.display_task
            except asyncio.CancelledError:
                pass
        await self.vr_server.stop()
        self._stop_https_server()
        self._disconnect_robot_arms()
    
    async def _init_robot_arms(self):
        """初始化左右机械臂"""
        try:
            # 初始化左机械臂
            self.left_piper = C_PiperInterface_V2("arm_l")
            self.left_piper.ConnectPort()
            while not self.left_piper.EnablePiper():
                await asyncio.sleep(0.01)
            self.left_arm_connected = True
            
            # 初始化右机械臂
            self.right_piper = C_PiperInterface_V2("arm_r")
            self.right_piper.ConnectPort()
            while not self.right_piper.EnablePiper():
                await asyncio.sleep(0.01)
            self.right_arm_connected = True
            
            # 初始化逆解器
            urdf_path = '/home/robotgym/jxj/qijia-teleopvr/telegrip/URDF/piper/piper_description.urdf'
            self.left_ik = ArmIK(urdf_path)
            self.right_ik = ArmIK(urdf_path)
            # self.left_ik = ArmIK_tcp(urdf_path,self.tcp_offset)
            # self.right_ik = ArmIK_tcp(urdf_path,self.tcp_offset)
            
            # 计算初始关节角
            self._calculate_initial_joints()
            
            self._move_arm_to_position("left", self.left_initial_position)
            self._move_arm_to_position("right", self.right_initial_position)
            print("左右机械臂连接成功  移动到初始位置")
            
            # 构造初始位姿矩阵A0 (单位: mm)
            # 左臂
            self.A0["left"] = np.eye(4)
            self.A0["left"][:3, :3] = R.from_euler('xyz', self.left_initial_position[3:-1], degrees=True).as_matrix()
            self.A0["left"][:3, 3] = self.left_initial_position[:3]
            
            # 右臂
            self.A0["right"] = np.eye(4)
            self.A0["right"][:3, :3] = R.from_euler('xyz', self.right_initial_position[3:-1], degrees=True).as_matrix()
            self.A0["right"][:3, 3] = self.right_initial_position[:3]

            print("左臂初始位姿矩阵 A0 (单位: mm):")
            print(self.A0["left"])
            print("右臂初始位姿矩阵 A0 (单位: mm):")
            print(self.A0["right"])
            print("变换矩阵 T:")
            print(self.T)
            
        except Exception as e:
            logger.error(f"机械臂连接失败: {e}")
            self.left_arm_connected = False
            self.right_arm_connected = False
    
    def _calculate_initial_joints(self):
        """从初始位姿计算初始关节角"""
        try:
            # 计算左臂初始关节角
            left_target_pose = np.eye(4)
            left_target_pose[:3, :3] = R.from_euler('xyz', self.left_initial_position[3:-1], degrees=True).as_matrix()
            left_target_pose[:3, 3] = self.left_initial_position[:3]  # mm
            left_joints = self.left_ik.ik(left_target_pose)
            self.left_initial_joints = left_joints if left_joints is not None else [0, 0, 0, 0, 0, 0]
            
            # 计算右臂初始关节角
            right_target_pose = np.eye(4)
            right_target_pose[:3, :3] = R.from_euler('xyz', self.right_initial_position[3:-1], degrees=True).as_matrix()
            right_target_pose[:3, 3] = self.right_initial_position[:3]  # mm
            right_joints = self.right_ik.ik(right_target_pose)
            self.right_initial_joints = right_joints if right_joints is not None else [0, 0, 0, 0, 0, 0]
            
            print(f"左臂初始关节角: {self.left_initial_joints}")
            print(f"右臂初始关节角: {self.right_initial_joints}")
        except Exception as e:
            logger.error(f"计算初始关节角失败: {e}")
            # 如果计算失败，使用默认值
            self.left_initial_joints = [0, 0, 0, 0, 0, 0]
            self.right_initial_joints = [0, 0, 0, 0, 0, 0]
    
    def _disconnect_robot_arms(self):
        """断开机械臂连接"""
        # 断开左机械臂
        if self.left_piper and self.left_arm_connected:
            try:
                self._move_arm_to_position("left", self.left_initial_position)
                self.left_piper.GripperCtrl(0, 1000, 0x01, 0)
                self.left_piper.DisconnectPort()
                self.left_arm_connected = False
                logger.info("左机械臂已断开连接 回到初始位置")
            except Exception as e:
                logger.error(f"断开左机械臂连接时出错: {e}")
        
        # 断开右机械臂
        if self.right_piper and self.right_arm_connected:
            try:
                self._move_arm_to_position("right", self.right_initial_position)
                self.right_piper.GripperCtrl(0, 1000, 0x01, 0)
                self.right_piper.DisconnectPort()
                self.right_arm_connected = False
                logger.info("右机械臂已断开连接 回到初始位置")
            except Exception as e:
                logger.error(f"断开右机械臂连接时出错: {e}")
    
    def _move_arm_to_position(self, arm: str, position):
        """通过关节角控制移动机械臂到指定位置"""
        piper = self.left_piper if arm == "left" else self.right_piper
        connected = self.left_arm_connected if arm == "left" else self.right_arm_connected
        ik_solver = self.left_ik if arm == "left" else self.right_ik
        
        if not piper or not connected or not ik_solver:
            return
            
        try:
            # 构造目标位姿矩阵 (平移部分单位为毫米，旋转部分由欧拉角转换)
            target_pose = np.eye(4)
            target_pose[:3, :3] = R.from_euler('xyz', position[3:-1], degrees=True).as_matrix()
            target_pose[:3, 3] = position[:3]  # mm
            
            # 计算逆解 (返回关节角单位为度)
            joint_angles_deg = ik_solver.ik(target_pose)
            
            if joint_angles_deg is not None:
                # 转换关节角单位为0.001° (度转千分之一度)
                joint_1 = round(joint_angles_deg[0] * 1000)  # degree to 0.001°
                joint_2 = round(joint_angles_deg[1] * 1000)  # degree to 0.001°
                joint_3 = round(joint_angles_deg[2] * 1000)  # degree to 0.001°
                joint_4 = round(joint_angles_deg[3] * 1000)  # degree to 0.001°
                joint_5 = round(joint_angles_deg[4] * 1000)  # degree to 0.001°
                joint_6 = round(joint_angles_deg[5] * 1000)  # degree to 0.001°

                # 第7关节(夹爪)是位置单位(千分之一毫米)
                gripper_pos = round(position[6] * 1000)  # mm to 0.001mm
                t1 = time.time()
                piper.MotionCtrl_2(0x01, 0x01, 100, 0xad)       # MIT模式
                piper.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
                piper.GripperCtrl(abs(gripper_pos), 1000, 0x01, 0)
                t2 = time.time()
                print(f"piperAPI调用耗时1:{(t2-t1)*1000}")
            else:
                logger.error(f"{arm}机械臂逆解失败")
        except Exception as e:
            logger.error(f"{arm}机械臂移动出错: {e}")
    
    def _get_arm_current_joints(self, arm: str):
        """获取机械臂当前关节角 (单位: 度)"""
        piper = self.left_piper if arm == "left" else self.right_piper
        if not piper:
            return None
            
        try:
            joint_data = piper.GetArmJointMsgs()
            joints = [
                joint_data.joint_state.joint_1 / 1000.0,  # 0.001° to degree
                joint_data.joint_state.joint_2 / 1000.0,  # 0.001° to degree
                joint_data.joint_state.joint_3 / 1000.0,  # 0.001° to degree
                joint_data.joint_state.joint_4 / 1000.0,  # 0.001° to degree
                joint_data.joint_state.joint_5 / 1000.0,  # 0.001° to degree
                joint_data.joint_state.joint_6 / 1000.0   # 0.001° to degree
            ]
            return joints
        except Exception as e:
            logger.error(f"获取{arm}机械臂当前关节角失败: {e}")
            return None
    
    def _get_arm_current_pose(self, arm: str):
        """获取机械臂当前末端位姿"""
        piper = self.left_piper if arm == "left" else self.right_piper
        if not piper:
            return None
            
        try:
            end_pose_data = piper.GetArmEndPoseMsgs()
            x = end_pose_data.end_pose.X_axis / 1000.0  # mm
            y = end_pose_data.end_pose.Y_axis / 1000.0  # mm
            z = end_pose_data.end_pose.Z_axis / 1000.0  # mm
            rx = end_pose_data.end_pose.RX_axis / 1000.0  # degree
            ry = end_pose_data.end_pose.RY_axis / 1000.0  # degree
            rz = end_pose_data.end_pose.RZ_axis / 1000.0  # degree
            
            # 创建4x4齐次变换矩阵
            A = np.eye(4)
            rotation = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
            A[:3, :3] = rotation
            A[:3, 3] = [x, y, z]
            
            return A
        except Exception as e:
            logger.error(f"获取{arm}机械臂当前位姿失败: {e}")
            return None
    
    async  def _start_https_server(self):
        """启动HTTPS服务器提供Web界面"""
        try:
            self.https_server = http.server.HTTPServer(
                (self.config.host_ip, self.config.https_port), 
                APIHandler
            )
            self.https_server.api_handler = self
            
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            context.load_cert_chain(cert_path, key_path)
            self.https_server.socket = context.wrap_socket(self.https_server.socket, server_side=True)
            
            self.https_thread = threading.Thread(target=self.https_server.serve_forever, daemon=True)
            self.https_thread.start()
            
            logger.info(f"HTTPS server started on {self.config.host_ip}:{self.config.https_port}")
        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
    
    def _stop_https_server(self):
        """停止HTTPS服务器"""
        if self.https_server:
            self.https_server.shutdown()
            if self.https_thread:
                self.https_thread.join(timeout=5)
            logger.info("HTTPS server stopped")
    
    async def _display_loop(self):
        """循环显示VR控制器数据"""
        print("VR控制器数据监视器启动...")
        print(f"Web界面地址: https://{self._get_local_ip()}:{self.config.https_port}/")
        print(f"WebSocket地址: wss://{self._get_local_ip()}:{self.config.websocket_port}/")
        print("等待VR头显连接...")
        print("-" * 50)
        
        while self.is_running:
            # 处理手柄按键的更新
            self._print_controller_data() 
            
            # 计算新的手柄位姿 --> 计算机械臂目标位姿 --> IK --> 控制机械臂
            await self._process_realtime_control()
            
            await asyncio.sleep(0.001)  # 主循环 1000 Hz
    
    
    async def _process_realtime_control(self):
        """处理实时控制机械臂 - 并行版本"""
        # 检查命令队列中的A键命令
        try:
            while True:
                goal = self.command_queue.get_nowait()
                if goal.metadata and goal.metadata.get("action") == "return_to_home":
                    await self._return_to_home_pose(goal.arm)
        except asyncio.QueueEmpty:
            pass

        # 收集需要处理的机械臂
        arms_to_process = []
        
        # 检查左手
        left_controller = self.vr_server.left_controller
        left_relative_pos = self.vr_server.relative_pos_left
        
        # 检查是否有X键按下请求回到初始位姿
        if hasattr(left_controller, 'x_button_active') and left_controller.x_button_active:
            print("检测到左手柄X键按下")
            await self._return_to_home_pose("left")
            left_controller.x_button_active = False
        # 否则检查是否需要进行实时控制
        elif (self.realtime_control_active["left"] and self.grip_origin_data["left"] is not None and 
              left_controller.accumulated_rotation_quat is not None):
            current_time = time.time()
            if current_time - self.last_control_time["left"] >= self.control_interval:
                arms_to_process.append(("left", left_controller, left_relative_pos))

        # 检查右手
        right_controller = self.vr_server.right_controller
        right_relative_pos = self.vr_server.relative_pos_right
        
        # 检查是否有A键按下请求回到初始位姿
        if hasattr(right_controller, 'a_button_active') and right_controller.a_button_active:
            print("检测到右手柄A键按下")
            await self._return_to_home_pose("right")
            right_controller.a_button_active = False
        # 否则检查是否需要进行实时控制
        elif (self.realtime_control_active["right"] and self.grip_origin_data["right"] is not None and 
              right_controller.accumulated_rotation_quat is not None):
            current_time = time.time()
            if current_time - self.last_control_time["right"] >= self.control_interval:
                arms_to_process.append(("right", right_controller, right_relative_pos))

        # 并行处理所有需要控制的机械臂
        if arms_to_process:
            tasks = [self._process_single_arm(arm, controller, relative_pos) 
                     for arm, controller, relative_pos in arms_to_process]
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _process_single_arm(self, arm: str, controller, relative_pos):
        """处理单个机械臂的控制"""
        try:
            current_time = time.time()
            
            # 构造当前T1矩阵 (VR坐标系，单位: 毫米) origin_position
            T1_rot = self.quaternion_to_rotation_matrix(controller.accumulated_rotation_quat)
            T1_pos = np.array([
                (controller.origin_position['x'] + relative_pos[0]) * 1000,
                (controller.origin_position['y'] + relative_pos[1]) * 1000,
                (controller.origin_position['z'] + relative_pos[2]) * 1000
            ])
            
            # 预计算常量部分以提高性能
            if self._cached_transform[arm] is None:
                if self.A0[arm] is not None and self.m_offset[arm] is not None:
                    self._cached_transform[arm] = self.A0[arm] @ inv(self.T @ self.grip_origin_data[arm]["T0"] @ self.m_offset[arm])
                    print(f"更新{arm}臂矩阵缓存")
            
            T1 = np.eye(4)
            T1[:3, :3] = T1_rot
            T1[:3, 3] = T1_pos
            
            if self._cached_transform[arm] is not None and self.m_offset[arm] is not None:
                # 计算机械臂的目标位姿A1(mm)
                A1 = self._cached_transform[arm] @ (self.T @ T1 @ self.m_offset[arm])
            else:
                A1 = self.A0[arm] @ inv(self.T @ self.grip_origin_data[arm]["T0"] @ self.m_offset[arm]) @ (self.T @ T1 @ self.m_offset[arm])
            
            # 通过逆解计算关节角
            ik_solver = self.left_ik if arm == "left" else self.right_ik
            ik_start_time = time.time()
            joint_angles_deg = ik_solver.ik(A1)
            ik_end_time = time.time()
            ik_time = ik_end_time - ik_start_time
            
            if ik_time > 0.007:  # 超过7ms
                self.ik_timeout_count[arm] += 1
                print(f"{arm}IK超过7ms,耗时{ik_time * 1000:.2f} ms")

                # 如果IK超时，使用上一次成功的关节角度
                if self.last_successful_joints[arm] is not None and self.last_successful_gripper[arm] is not None:
                    logger.warning(f"{arm}IK计算超时，使用上一次成功的结果")
                    self._move_arm_to_joints(arm, self.last_successful_joints[arm], self.last_successful_gripper[arm])
                    self.last_control_time[arm] = current_time
                    return  # 直接返回，不继续执行下面的逻辑
                else:
                    logger.error(f"{arm}IK计算超时且无历史数据可用，跳过此次控制")
                    return
            else:
                print("IK计算成功")
            # print(f"IK超时次数: {self.ik_timeout_count[arm]}")

            if joint_angles_deg is not None:
                # 通过trigger按钮控制夹爪
                gripper_position = self.gripper_limits["min"]
                if controller.trigger_active:
                    gripper_position = self.gripper_limits["max"]
                
                # 应用滤波和优化
                filtered_joints = self._apply_lowpass_filter_joints(arm, joint_angles_deg)
                self.last_control_time[arm] = current_time
                self._move_arm_to_joints(arm, filtered_joints, gripper_position)
                # 保存上次成功的数据
                self.last_successful_joints[arm] = filtered_joints
                self.last_successful_gripper[arm] = gripper_position
            else:
                logger.error(f"{arm}机械臂逆解失败,使用上一次成功的结果")
                if self.last_successful_joints[arm] is not None and self.last_successful_gripper[arm] is not None:
                    self._move_arm_to_joints(arm, self.last_successful_joints[arm], self.last_successful_gripper[arm])
                    self.last_control_time[arm] = current_time
                else:
                    logger.error(f"{arm}无历史关节角度可用，跳过此次控制")
        except Exception as e:
            logger.error(f"实时控制{arm}机械臂时出错: {e}")

    def _move_arm_to_joints(self, arm: str, joint_angles_deg: list, gripper_position: float):
        """通过关节角控制机械臂"""
        piper = self.left_piper if arm == "left" else self.right_piper
        connected = self.left_arm_connected if arm == "left" else self.right_arm_connected
        
        if not piper or not connected:
            return
            
        try:
            # 转换关节角单位为0.001° (度转千分之一度)
            joint_1 = round(joint_angles_deg[0] * 1000)  # degree to 0.001°
            joint_2 = round(joint_angles_deg[1] * 1000)  # degree to 0.001°
            joint_3 = round(joint_angles_deg[2] * 1000)  # degree to 0.001°
            joint_4 = round(joint_angles_deg[3] * 1000)  # degree to 0.001°
            joint_5 = round(joint_angles_deg[4] * 1000)  # degree to 0.001°
            joint_6 = round(joint_angles_deg[5] * 1000)  # degree to 0.001°
            # 夹爪位置单位为千分之一毫米
            gripper_pos = round(gripper_position * 1000)  # mm to 0.001mm
            
            # 发送控制指令
            piper.MotionCtrl_2(0x01, 0x01, 100, 0xad)   #MIT模式
            piper.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
            piper.GripperCtrl(abs(gripper_pos), 1000, 0x01, 0)
        except Exception as e:
            logger.error(f"{arm}机械臂关节控制出错: {e}")

    # 添加回到A0姿态的方法（必须松开离合按钮）
    async def _return_to_home_pose(self, arm: str):
        """让机械臂回到初始位姿"""
        print(f"🎮 {arm} 手柄按键按下 - 机械臂回到初始位姿")
        
        try:
            # 获取对应的初始关节角 - 使用从初始位置计算得出的关节角
            initial_joints = self.left_initial_joints if arm == "left" else self.right_initial_joints
            initial_gripper = self.left_initial_position[6] if arm == "left" else self.right_initial_position[6]
            
            # 确保机械臂已连接
            piper = self.left_piper if arm == "left" else self.right_piper
            connected = self.left_arm_connected if arm == "left" else self.right_arm_connected
            
            if not piper or not connected:
                print(f"⚠️ {arm} 机械臂未连接，无法执行回初始位姿操作")
                return
            
            # 转换关节角单位为0.001° (度转千分之一度)
            joint_1 = round(initial_joints[0] * 1000)  # degree to 0.001°
            joint_2 = round(initial_joints[1] * 1000)  # degree to 0.001°
            joint_3 = round(initial_joints[2] * 1000)  # degree to 0.001°
            joint_4 = round(initial_joints[3] * 1000)  # degree to 0.001°
            joint_5 = round(initial_joints[4] * 1000)  # degree to 0.001°
            joint_6 = round(initial_joints[5] * 1000)  # degree to 0.001°
            # 夹爪位置单位为千分之一毫米
            gripper_pos = round(initial_gripper * 1000)  # mm to 0.001mm
            
            # 发送控制指令
            t1 = time.time()
            piper.MotionCtrl_2(0x01, 0x01, 100, 0xad)       #MIT模式
            piper.JointCtrl(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)
            piper.GripperCtrl(abs(gripper_pos), 1000, 0x01, 0)
            t2 = time.time()
            print(f"piperAPI调用耗时3(ms),{(t2-t1)*1000}")
        except Exception as e:
            logger.error(f"{arm} 机械臂回到初始位姿时出错: {e}")
            
    def _apply_lowpass_filter_joints(self, arm: str, current_joints_deg: list) -> list:
        """
        应用一阶低通滤波器到关节角
        
        参数:
            arm: 手臂标识 ("left" 或 "right")
            current_joints_deg: 当前目标关节角 [j1, j2, j3, j4, j5, j6] (单位: 度)
        
        返回:
            list: 滤波后的关节角
        """
        current_joints_deg = np.array(current_joints_deg)
        
        # 如果是第一次滤波，直接使用当前值
        if self.filtered_pose[arm] is None:
            self.filtered_pose[arm] = current_joints_deg.copy()
            return current_joints_deg.tolist()
        
        # 应用一阶低通滤波: y[n] = α*x[n] + (1-α)*y[n-1]
        filtered_joints = self.filter_alpha * current_joints_deg + (1 - self.filter_alpha) * self.filtered_pose[arm]
        
        # 更新滤波后的关节角
        self.filtered_pose[arm] = filtered_joints.copy()
        
        return filtered_joints.tolist()
    
    def _get_local_ip(self):
        """获取本机IP地址"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except:
            return "localhost"
    
    def _print_controller_data(self):
        """打印控制器数据到控制台"""
        # 检查是否有客户端连接
        if not self.vr_server.clients:
            return
            
        # 获取左右手控制器状态
        left_controller = self.vr_server.left_controller
        right_controller = self.vr_server.right_controller
        
        # 检查左手grip状态变化
        if not self.previous_grip_state["left"] and left_controller.grip_active:
            # grip刚按下，保存初始数据
            print("左手 grip 按下")
            if left_controller.origin_position is not None and left_controller.origin_quaternion is not None:
                # 获取机械臂当前位姿作为新的A0 - 与V5版本保持一致
                current_arm_pose = self._get_arm_current_pose("left")
                if current_arm_pose is not None:
                    self.A0["left"] = current_arm_pose
                
                # 构造T0矩阵
                T0_rot = self.quaternion_to_rotation_matrix(left_controller.origin_quaternion)
                T0_pos = np.array([
                    left_controller.origin_position['x'] * 1000,  # 米转毫米
                    left_controller.origin_position['y'] * 1000,  # 米转毫米
                    left_controller.origin_position['z'] * 1000   # 米转毫米
                ])
                T0 = np.eye(4)
                T0[:3, :3] = T0_rot
                T0[:3, 3] = T0_pos
                
                self.grip_origin_data["left"] = {
                    "origin_position": left_controller.origin_position.copy(),
                    "origin_quaternion": left_controller.origin_quaternion.copy(),
                    "T0": T0
                }
                
                # 重新计算偏移矩阵
                if self.A0["left"] is not None:
                    self.m_offset["left"] = inv(self.T @ T0) @ self.A0["left"]
                    self.m_offset["left"][:3,3] = 0  # 将偏移矩阵的平移部分设置为零
                
                # 激活实时控制
                self.realtime_control_active["left"] = True
                
                # 重置滤波器状态
                self.filtered_pose["left"] = None
                
                # 重置按键状态
                left_controller.x_button_active = False

                # 清除矩阵结果缓存，将在下次控制时重新计算
                self._cached_transform["left"] = None
                print("清除左臂矩阵缓存")
                
        elif self.previous_grip_state["left"] and not left_controller.grip_active:
            # grip刚释放
            print("左手 grip 释放")
            self.realtime_control_active["left"] = False
            self.grip_origin_data["left"] = None
        
        # 检查右手grip状态变化
        if not self.previous_grip_state["right"] and right_controller.grip_active:
            # grip刚按下，保存初始数据
            print("右手 grip 按下")
            if right_controller.origin_position is not None and right_controller.origin_quaternion is not None:
                # 获取机械臂当前位姿作为新的A0 - 与V5版本保持一致
                current_arm_pose = self._get_arm_current_pose("right")
                if current_arm_pose is not None:
                    self.A0["right"] = current_arm_pose
                
                # 构造T0矩阵
                T0_rot = self.quaternion_to_rotation_matrix(right_controller.origin_quaternion)
                T0_pos = np.array([
                    right_controller.origin_position['x'] * 1000,  # 米转毫米
                    right_controller.origin_position['y'] * 1000,  # 米转毫米
                    right_controller.origin_position['z'] * 1000   # 米转毫米
                ])
                T0 = np.eye(4)
                T0[:3, :3] = T0_rot
                T0[:3, 3] = T0_pos
                self.grip_origin_data["right"] = {
                    "origin_position": right_controller.origin_position.copy(),
                    "origin_quaternion": right_controller.origin_quaternion.copy(),
                    "T0": T0
                }
                
                # 重新计算偏移矩阵
                if self.A0["right"] is not None:
                    self.m_offset["right"] = inv(self.T @ T0) @ self.A0["right"]
                    self.m_offset["right"][:3,3] = 0  # 将偏移矩阵的平移部分设置为零
                
                # 激活实时控制
                self.realtime_control_active["right"] = True
                
                # 重置滤波器状态
                self.filtered_pose["right"] = None
                
                # 重置按键状态
                right_controller.a_button_active = False

                # 清除矩阵结果缓存，将在下次控制时重新计算
                self._cached_transform["right"] = None
                print("清除右臂矩阵缓存")
                
        elif self.previous_grip_state["right"] and not right_controller.grip_active:
            # grip刚释放
            print("右手 grip 释放")
            self.realtime_control_active["right"] = False
            self.grip_origin_data["right"] = None
        
        # 更新前一状态
        self.previous_grip_state["left"] = left_controller.grip_active
        self.previous_grip_state["right"] = right_controller.grip_active
 

def create_vr_config():
    """创建VR专用配置"""
    config = TelegripConfig()
    
    # 启用VR功能
    config.enable_vr = True
    config.enable_robot = False
    config.enable_pybullet = False
    config.enable_keyboard = False
    config.enable_pybullet_gui = False
    config.autoconnect = False
    
    # 网络配置
    config.host_ip = "0.0.0.0"
    config.https_port = 8443
    config.websocket_port = 8442
    
    # 确保证书存在
    try:
        config.ensure_ssl_certificates()
    except Exception as e:
        logger.warning(f"SSL证书生成警告: {e}")
    
    return config

async def main():
    # 创建VR专用配置
    config = create_vr_config()
    
    print("启动VR控制器监视器...")
    print(f"HTTPS服务器将在端口 {config.https_port} 上运行")
    print(f"WebSocket服务器将在端口 {config.websocket_port} 上运行")
    
    # 获取本机IP地址用于显示
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
    except:
        local_ip = "localhost"
    
    print(f"请在VR头显中打开: https://{local_ip}:{config.https_port}/")
    print("等待VR头显连接...")
    print("-" * 50)
    
    # 创建并启动系统
    system = SimpleVRSystem(config)
    
    try:
        await system.start()
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭...")
        await system.stop()
        print("系统已关闭")
    except Exception as e:
        logger.error(f"系统错误: {e}")
        await system.stop()
        print(f"系统错误: {e}")

if __name__ == "__main__":
    asyncio.run(main())