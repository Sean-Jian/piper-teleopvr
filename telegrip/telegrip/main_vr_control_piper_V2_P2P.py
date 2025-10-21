"""
该版本实现了VR手柄姿态到机械臂末端姿态的正确映射   但仅仅是点对点移动 
上电给机械臂一个初始位姿A0 之后手柄按住离合并移动到另一个位姿并松开离合 基于手柄按下离合时的位姿T0以及松开离合时的位姿T1  
来计算机械臂末端的目标位姿A1  然后让机械臂从A0移动到A1  主要是为了验证考虑坐标映射后的正确性
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
        
        # 机械臂初始位置 (单位: mm和度)
        self.left_initial_position = [152.0, 15.0, 319.0, -155.0, 43.0, -139.0, 0]
        self.right_initial_position = [152.0, 15.0, 319.0, -155.0, 43.0, -139.0, 0]
        
        # 控制频率限制
        self.last_control_time = {"left": 0.0, "right": 0.0}
        self.control_interval = 0.1  # 100ms控制间隔

        # 夹爪位置限制 (mm)
        self.gripper_limits = {
            "min": 0,
            "max": 50  # 大多数夹爪的最大开合范围约为100mm
        }
        
        # 保存机械臂末端位姿的变量
        self.last_arm_pose = {"left": None, "right": None}
        self.arm_control_active = {"left": False, "right": False}
        
        # 保存初始姿态
        # self.initial_orientation = {"left": None, "right": None}
        # scale: vr-->arm
        self.scale = 0.5
        
        # 齐次变换矩阵相关
        self.A0 = None  # 机器人初始位姿 (4x4)
        self.T = None   # 从VR坐标系到机器人坐标系的变换矩阵 (4x4)
        self.m_offset = None  # 偏移矩阵 (4x4)
        self.m_offset_computed = False  # 标记是否已计算偏移矩阵
        
        # 初始化T矩阵为单位矩阵（您需要根据实际情况修改）
        self.T = np.array([[0,0,-1,0],
                           [-1,0,0,0],
                           [0,1,0,0],
                           [0,0,0,1]])
        
        # 用于控制机械臂移动的标志
        self.move_pending = {"left": False, "right": False}
        self.pending_move_data = {"left": None, "right": None}
        
        # 用于跟踪grip状态变化
        self.previous_grip_state = {"left": False, "right": False}
        
        # 存储相对位置数据
        self.last_relative_pos = {"left": None, "right": None}
        
        # 存储控制器初始数据（用于grip释放时使用）
        self.grip_origin_data = {"left": None, "right": None}
    
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
            # 初始化左机械臂 (假设使用can0)
            # self.left_piper = C_PiperInterface_V2("can0")
            # self.left_piper.ConnectPort()
            # while not self.left_piper.EnablePiper():
            #     await asyncio.sleep(0.01)
            # self.left_arm_connected = True
            # logger.info("左机械臂连接成功")
            
            # 初始化右机械臂 (假设使用can1)
            self.right_piper = C_PiperInterface_V2("can_arm1")
            self.right_piper.ConnectPort()
            # self.right_piper.MotionCtrl_1(0x02,0,0)#恢复
            while not self.right_piper.EnablePiper():
                await asyncio.sleep(0.01)
            self.right_arm_connected = True
            
            self._move_arm_to_position("right", self.right_initial_position)
            print("右机械臂连接成功  移动到初始位置")
            # 获取初始姿态并构造A0矩阵
            end_pose_data = self.right_piper.GetArmEndPoseMsgs()
            # self.initial_orientation["right"] = {
            #     'rx': end_pose_data.end_pose.RX_axis / 1000.0,  # 从 0.001° 转换为 °
            #     'ry': end_pose_data.end_pose.RY_axis / 1000.0,
            #     'rz': end_pose_data.end_pose.RZ_axis / 1000.0
            # }
            # print(f"initial_orientation-right已赋值: {self.initial_orientation['right']}")
            
            # 构造初始位姿矩阵A0 (单位: mm)
            x = end_pose_data.end_pose.X_axis / 1000.0  # mm
            y = end_pose_data.end_pose.Y_axis / 1000.0  # mm
            z = end_pose_data.end_pose.Z_axis / 1000.0  # mm
            rx = end_pose_data.end_pose.RX_axis / 1000.0  # degree
            ry = end_pose_data.end_pose.RY_axis / 1000.0  # degree
            rz = end_pose_data.end_pose.RZ_axis / 1000.0  # degree
            
            # 创建4x4齐次变换矩阵
            self.A0 = np.eye(4)
            rotation = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
            # self.A0[:3, :3] = rotation  
            # self.A0[:3, 3] = [x, y, z] 
            
            self.A0[:3, :3] = R.from_euler('xyz',self.right_initial_position[3:-1],degrees=True).as_matrix()
            self.A0[:3, 3] = self.right_initial_position[:3]

            print("初始位姿矩阵 A0 (单位: mm):")
            print(self.A0)
            print("变换矩阵 T:")
            print(self.T)
            
        except Exception as e:
            logger.error(f"机械臂连接失败: {e}")
            self.left_arm_connected = False
            self.right_arm_connected = False
    
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
        """通过末端控制移动机械臂到指定位置"""
        piper = self.left_piper if arm == "left" else self.right_piper
        connected = self.left_arm_connected if arm == "left" else self.right_arm_connected
        
        if not piper or not connected:
            return
            
        try:
            # 应用运动范围限
            # limited_position = self._limit_arm_position(position)
            limited_position = position
            # 转换位置单位为0.001mm (毫米转千分之一毫米)
            X = round(limited_position[0] * 1000)  # mm to 0.001mm
            Y = round(limited_position[1] * 1000)  # mm to 0.001mm
            Z = round(limited_position[2] * 1000)  # mm to 0.001mm
            
            # 转换角度单位为0.001° (度转千分之一度)
            RX = round(limited_position[3] * 1000)  # degree to 0.001°
            RY = round(limited_position[4] * 1000)  # degree to 0.001°
            RZ = round(limited_position[5] * 1000)  # degree to 0.001°
            print(f"EndPoseCtrl()参数(单位:cm,度)--> x:{X/1000/10}cm,y:{Y/1000/10}cm,z:{Z/1000/10}cm,RX:{RX/1000}度,RY:{RY/1000}度,RZ:{RZ/1000}度")
            # 第7关节(夹爪)也是位置单位(千分之一毫米)
            joint_6 = round(limited_position[6] * 1000)  # mm to 0.001mm
            piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
            piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
            piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        except Exception as e:
            logger.error(f"{arm}机械臂移动出错: {e}")
    
    
    async def _start_https_server(self):
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
        
        last_display_time = 0
        display_interval = 0.05  # 每50ms显示一次
        
        while self.is_running:
            current_time = time.time()
            
            # 控制显示频率
            if current_time - last_display_time >= display_interval:
                self._print_controller_data()
                last_display_time = current_time
            
            # 处理待执行的机械臂移动
            await self._process_pending_moves()
            
            await asyncio.sleep(0.01)  # 10ms检查间隔
    
    async def _process_pending_moves(self):
        """处理待执行的机械臂移动"""
        for arm in ["left", "right"]:
            if self.move_pending[arm] and self.pending_move_data[arm] is not None:
                # 短暂延时
                time.sleep(0.5)
                
                # 执行移动
                target_position = self.pending_move_data[arm]
                self._move_arm_to_position(arm, target_position)
                
                # 标记移动完成
                self.move_pending[arm] = False
                self.pending_move_data[arm] = None
                
                print("机械臂移动完成")

    
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
        
        # 保存相对位置数据
        if self.vr_server.relative_pos_left is not None:
            self.last_relative_pos["left"] = self.vr_server.relative_pos_left.copy()
        if self.vr_server.relative_pos_right is not None:
            self.last_relative_pos["right"] = self.vr_server.relative_pos_right.copy()
        
        # 检查左手grip状态变化
        if not self.previous_grip_state["left"] and left_controller.grip_active:
            # grip刚按下，保存初始数据
            print("左手 grip 按下")
            if left_controller.origin_position is not None and left_controller.origin_quaternion is not None:
                self.grip_origin_data["left"] = {
                    "origin_position": left_controller.origin_position.copy(),
                    "origin_quaternion": left_controller.origin_quaternion.copy(),
                    "accumulated_rotation_quat": left_controller.accumulated_rotation_quat.copy() if left_controller.accumulated_rotation_quat is not None else left_controller.origin_quaternion.copy()
                }
        # 更新手柄当前的姿态（四元数）
        elif self.previous_grip_state["left"] and left_controller.grip_active:
            self.grip_origin_data["left"]["accumulated_rotation_quat"] = left_controller.accumulated_rotation_quat.copy()
        elif self.previous_grip_state["left"] and not left_controller.grip_active:
            # grip刚释放
            print("左手 grip 释放")
            self._handle_grip_release("left")
        
        # 检查右手grip状态变化
        if not self.previous_grip_state["right"] and right_controller.grip_active:
            # grip刚按下，保存初始数据
            print("右手 grip 按下")
            if right_controller.origin_position is not None and right_controller.origin_quaternion is not None:
                self.grip_origin_data["right"] = {
                    "origin_position": right_controller.origin_position.copy(),
                    "origin_quaternion": right_controller.origin_quaternion.copy(),
                    "accumulated_rotation_quat": right_controller.accumulated_rotation_quat.copy() if right_controller.accumulated_rotation_quat is not None else right_controller.origin_quaternion.copy()
                }

        # 更新手柄当前的姿态（四元数）
        elif self.previous_grip_state["right"] and right_controller.grip_active:
            self.grip_origin_data["right"]["accumulated_rotation_quat"] = right_controller.accumulated_rotation_quat.copy()
        elif self.previous_grip_state["right"] and not right_controller.grip_active:
            # grip刚释放
            print("右手 grip 释放")
            self._handle_grip_release("right")
        
        # 更新前一状态
        self.previous_grip_state["left"] = left_controller.grip_active
        self.previous_grip_state["right"] = right_controller.grip_active
 
        # 显示左手控制器数据
        if left_controller.grip_active and self.last_relative_pos["left"] is not None and type(left_controller.origin_position) is dict:
            print("左手控制器:")
            print(f"  🏠 初始位置-> x: {left_controller.origin_position['x']*100} cm , y: {left_controller.origin_position['y']*100} cm , z: {left_controller.origin_position['z']*100} cm")
            print(f"  📍 相对位移-> x: {self.last_relative_pos['left'][0]*100} cm , y: {self.last_relative_pos['left'][1]*100} cm , z: {self.last_relative_pos['left'][2]*100} cm")
            
            print(f"  ✋ 抓握状态: {'激活' if left_controller.grip_active else '未激活'}")
            print(f"  🔘 触发器: {'按下' if left_controller.trigger_active else '释放'}")
            print(f"  🔄 Z轴旋转: {left_controller.z_axis_rotation:.1f}°")
            print(f"  📈 X轴旋转: {left_controller.x_axis_rotation:.1f}°")
            print(f"  📈 Y轴旋转: {left_controller.y_axis_rotation:.1f}°")
            
            print()
        
        # 显示右手控制器数据
        if right_controller.grip_active and self.last_relative_pos["right"] is not None and type(right_controller.origin_position) is dict:
            print("右手控制器:")
            print(f"  🏠 初始位置-> x: {right_controller.origin_position['x']*100} cm , y: {right_controller.origin_position['y']*100} cm , z: {right_controller.origin_position['z']*100} cm")
            print(f"  📍 相对位移-> x: {self.last_relative_pos['right'][0]*100} cm , y: {self.last_relative_pos['right'][1]*100} cm , z: {self.last_relative_pos['right'][2]*100} cm")
            
            print(f"  ✋ 抓握状态: {'激活' if right_controller.grip_active else '未激活'}")
            print(f"  🔘 触发器: {'按下' if right_controller.trigger_active else '释放'}")
            print(f"  🔄 Z轴旋转: {right_controller.z_axis_rotation:.1f}°")
            print(f"  📈 X轴旋转: {right_controller.x_axis_rotation:.1f}°")
            print(f"  📈 Y轴旋转: {right_controller.y_axis_rotation:.1f}°")
            
            print("=" * 80)
            print()
    
    def _handle_grip_release(self, arm):
        """处理grip释放事件，计算并执行机械臂移动"""
        piper = self.left_piper if arm == "left" else self.right_piper
        connected = self.left_arm_connected if arm == "left" else self.right_arm_connected
        
        print(f"处理 {arm} 手控制器 grip 释放")
        
        if not piper or not connected:
            print(f"机械臂未连接: {arm}")
            return
            
        # 确保必要的数据存在
        if self.A0 is None or self.T is None or self.grip_origin_data[arm] is None:
            print(f"缺少必要数据，无法执行移动: A0={self.A0 is not None}, T={self.T is not None}, grip_origin_data={self.grip_origin_data[arm] is not None}")
            return
            
        origin_data = self.grip_origin_data[arm]
        origin_position = origin_data["origin_position"]
        origin_quaternion = origin_data["origin_quaternion"]
        accumulated_rotation_quat = origin_data["accumulated_rotation_quat"]
        
        # 获取相对位置数据
        relative_pos = self.last_relative_pos[arm]
        if relative_pos is None:
            print(f"相对位置数据为空，无法执行移动")
            return
            
        try:
            # 构造T0矩阵 (VR坐标系，单位: 毫米)
            # T0使用grip按下时的初始位姿
            T0_rot = self.quaternion_to_rotation_matrix(origin_quaternion)
            T0_pos = np.array([
                origin_position['x'] * 1000,  # 米转毫米
                origin_position['y'] * 1000,  # 米转毫米
                origin_position['z'] * 1000   # 米转毫米
            ])
            T0 = np.eye(4)
            T0[:3, :3] = T0_rot
            T0[:3, 3] = T0_pos
            
            # 构造T1矩阵 (VR坐标系，单位: 毫米)
            # T1使用grip释放时的最终累积旋转和相对位置
            T1_rot = self.quaternion_to_rotation_matrix(accumulated_rotation_quat)
            T1_pos = T0_pos + np.array([
                relative_pos[0] * 1000 if isinstance(relative_pos, (list, tuple, np.ndarray)) and len(relative_pos) > 0 else 0,  # 米转毫米
                relative_pos[1] * 1000 if isinstance(relative_pos, (list, tuple, np.ndarray)) and len(relative_pos) > 1 else 0,  # 米转毫米
                relative_pos[2] * 1000 if isinstance(relative_pos, (list, tuple, np.ndarray)) and len(relative_pos) > 2 else 0   # 米转毫米
            ])
            T1 = np.eye(4)
            T1[:3, :3] = T1_rot
            T1[:3, 3] = T1_pos
            
            print("变换矩阵信息:")
            print(f"T 矩阵:\n{self.T}")
            print(f"A0 矩阵 (mm):\n{self.A0}")
            print(f"T0 矩阵 (mm):\n{T0}")
            print(f"T1 矩阵 (mm):\n{T1}")
            
            # 如果是第一次计算偏移矩阵，则计算m_offset
            if not self.m_offset_computed:
                self.m_offset = inv(self.T @ T0) @ self.A0
                self.m_offset[:3,3] = 0  # 将偏移矩阵的平移部分设置为零
                self.m_offset_computed = True
                print(f"计算偏移矩阵 m_offset:\n{self.m_offset}")
            
            # 计算机械臂的目标位姿A1(mm)
            A1 = self.A0 @ inv(self.T@T0@self.m_offset) @ (self.T@T1@self.m_offset)  
            print(f"A1 矩阵 (mm):\n{A1}")
            # 将A1转换为target_pose (x, y, z, rx, ry, rz) - 单位: mm 和 度
            target_position = A1[:3, 3]  # mm
            target_rotation = R.from_matrix(A1[:3, :3]).as_euler('xyz', degrees=True)  # 度
            
            # 构造目标位置 (单位: mm和度)
            target_pose = [
                target_position[0],      # X (mm)
                target_position[1],      # Y (mm)
                target_position[2],      # Z (mm)
                target_rotation[0],      # RX (度)
                target_rotation[1],      # RY (度)
                target_rotation[2],      # RZ (度)
                0  # 夹爪位置
            ]
            
            # 设置待执行的移动
            self.move_pending[arm] = True
            self.pending_move_data[arm] = target_pose
            
            print(f"准备移动{arm}机械臂到目标位置 (mm, 度): {target_pose}")
            
        except Exception as e:
            logger.error(f"处理{arm}机械臂grip释放时出错: {e}")
            print(f"处理{arm}机械臂grip释放时出错: {e}")
            import traceback
            traceback.print_exc()

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