# main_test.py
"""
实现了通过网页网络获取VR手柄的位姿等数据并打印在控制台
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
# 添加项目路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
sys.path.insert(0, project_root)

from telegrip.config import TelegripConfig
from telegrip.inputs.vr_ws_server import VRWebSocketServer
from telegrip.inputs.vr_ws_server import VRControllerState

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
        
    async def start(self):
        self.is_running = True
        
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
        display_interval = 0.1  # 每100ms显示一次
        
        while self.is_running:
            current_time = time.time()
            
            # 控制显示频率
            if current_time - last_display_time >= display_interval:
                self._print_controller_data()
                last_display_time = current_time
            
            await asyncio.sleep(0.01)  # 10ms检查间隔
    
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
        
        print("=" * 80)
        print(f"时间: {time.strftime('%H:%M:%S')}")
        print(f"连接客户端数: {len(self.vr_server.clients)}")
        print("=" * 80)
        
        # 显示左手控制器数据
        print("左手控制器:")
        if left_controller.grip_active and type(self.vr_server.relative_pos_left) is np.ndarray and type(left_controller.origin_position) is dict:
            print(f"  🏠 初始位置-> x: {left_controller.origin_position['x']*100} cm , y: {left_controller.origin_position['y']*100} cm , z: {left_controller.origin_position['z']*100} cm")
            print(f"  📍 相对位移-> x: {self.vr_server.relative_pos_left[0]*100} cm , y: {self.vr_server.relative_pos_left[1]*100} cm , z: {self.vr_server.relative_pos_left[2]*100} cm")   
        print(f"  ✋ 抓握状态: {'激活' if left_controller.grip_active else '未激活'}")
        print(f"  🔘 触发器: {'按下' if left_controller.trigger_active else '释放'}")
        print(f"  🔄 Z轴旋转: {left_controller.z_axis_rotation:.1f}°")
        print(f"  📈 X轴旋转: {left_controller.x_axis_rotation:.1f}°")
        print(f"  📈 Y轴旋转: {left_controller.y_axis_rotation:.1f}°")
        
        print()
        
        # 显示右手控制器数据
        print("右手控制器:")
        if right_controller.grip_active and type(self.vr_server.relative_pos_right) is np.ndarray and type(right_controller.origin_position) is dict:
            print(f"  🏠 初始位置-> x: {right_controller.origin_position['x']*100} cm , y: {right_controller.origin_position['y']*100} cm , z: {right_controller.origin_position['z']*100} cm")
            print(f"  📍 相对位移-> x: {self.vr_server.relative_pos_right[0]*100} cm , y: {self.vr_server.relative_pos_right[1]*100} cm , z: {self.vr_server.relative_pos_right[2]*100} cm")
            
        print(f"  ✋ 抓握状态: {'激活' if right_controller.grip_active else '未激活'}")
        print(f"  🔘 触发器: {'按下' if right_controller.trigger_active else '释放'}")
        print(f"  🔄 Z轴旋转: {right_controller.z_axis_rotation:.1f}°")
        print(f"  📈 X轴旋转: {right_controller.x_axis_rotation:.1f}°")
        print(f"  📈 Y轴旋转: {right_controller.y_axis_rotation:.1f}°")
        
        print("=" * 80)
        print()

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