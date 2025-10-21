"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import ssl
import websockets
import numpy as np
import math
import logging
from typing import Dict, Optional, Set
from scipy.spatial.transform import Rotation as R
import time
from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TelegripConfig
from ..core.kinematics import compute_relative_position
from scipy.spatial.transform import Rotation as R
logger = logging.getLogger(__name__)


class VRControllerState:
    """State tracking for a VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking for relative movement
        self.origin_position = None
        self.origin_rotation = None
        
        # Quaternion-based rotation tracking (more stable than Euler)
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None  # Accumulated rotation as quaternion
        
        # Rotation tracking for wrist control
        self.z_axis_rotation = 0.0  # For wrist_roll
        self.x_axis_rotation = 0.0  # For wrist_flex (pitch)
        self.y_axis_rotation = 0.0 # yaw
        # Position tracking
        self.current_position = None
        
        # Rotation tracking
        self.origin_wrist_angle = 0.0

        # 添加A X 键状态
        self.a_button_active = False  
        self.x_button_active = False  
    
    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.z_axis_rotation = 0.0
        self.y_axis_rotation = 0.0
        self.x_axis_rotation = 0.0
        self.a_button_active = False  # 重置时也重置A键状态
        self.x_button_active = False  # 重置时也重置X键状态


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Robot state tracking (for relative position calculation)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None

        # 新增
        self.relative_pos_left = None
        self.relative_pos_right = None
    
    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket server."""
        # Automatically generate SSL certificates if they don't exist
        if not self.config.ssl_files_exist:
            logger.info("SSL certificates not found for WebSocket server, attempting to generate them...")
            if not self.config.ensure_ssl_certificates():
                logger.error("Failed to generate SSL certificates for WebSocket server")
                return None
        
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            # Get absolute paths for SSL certificates
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            ssl_context.load_cert_chain(certfile=cert_path, keyfile=key_path)
            logger.info("SSL certificate and key loaded successfully for WebSocket server")
            return ssl_context
        except ssl.SSLError as e:
            logger.error(f"Error loading SSL cert/key: {e}")
            return None
    
    async def start(self):
        """Start the WebSocket server."""
        if not self.config.enable_vr:
            logger.info("VR WebSocket server disabled in configuration")
            return
        
        ssl_context = self.setup_ssl()
        if ssl_context is None:
            logger.error("Failed to setup SSL for WebSocket server")
            return
        
        host = self.config.host_ip
        port = self.config.websocket_port
        
        try:
            self.server = await websockets.serve(
                self.websocket_handler, 
                host, 
                port, 
                ssl=ssl_context
            )
            self.is_running = True
            logger.info(f"VR WebSocket server running on wss://{host}:{port}")
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")
    
    async def stop(self):
        """Stop the WebSocket server."""
        self.is_running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info("VR WebSocket server stopped")
    
    async def websocket_handler(self, websocket, path=None):
        """Handle WebSocket connections from VR controllers."""
        client_address = websocket.remote_address
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)
        
        #测量message更新频率
        # message_count = 0
        # start_time = time.time()

        try:
            async for message in websocket:
                try:
                    # message_count += 1
                    # current_time = time.time()
                    # if message_count%100==0:
                    #     elapsed_time = current_time - start_time
                    #     freq = message_count / elapsed_time if elapsed_time > 0 else 0
                    #     print(f"📊 WebSocket消息频率: {freq:.2f} Hz ({message_count} messages in {elapsed_time:.2f}s)")
                    #     message_count = 0
                    #     start_time = current_time
                    data = json.loads(message)
                    await self.process_controller_data(data)
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON message: {message}")
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}")
        
        except websockets.exceptions.ConnectionClosedOK:
            logger.info(f"VR client {client_address} disconnected normally")
        except websockets.exceptions.ConnectionClosedError as e:
            logger.warning(f"VR client {client_address} disconnected with error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error with VR client {client_address}: {e}")
        finally:
            self.clients.discard(websocket)
            # Handle grip releases when client disconnects
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            logger.info(f"VR client {client_address} cleanup complete")
    
    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""
        
        # Handle new dual controller format
        if 'leftController' in data and 'rightController' in data:
            left_data = data['leftController']
            right_data = data['rightController']
            
            # 处理左手X键事件 - 只在未按住grip时有效
            if 'xButtonPressed' in left_data and left_data.get('xButtonPressed'):
                # 检查是否未按住grip
                if not left_data.get('gripActive', False):
                    await self.handle_x_button_press('left')
            
            # 处理右手A键事件 - 只在未按住grip时有效
            if 'aButtonPressed' in right_data and right_data.get('aButtonPressed'):
                # 检查是否未按住grip
                if not right_data.get('gripActive', False):
                    await self.handle_a_button_press('right')
            
            # Process left controller
            if left_data.get('position') and (left_data.get('gripActive', False) or left_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('left', left_data)
            elif not left_data.get('gripActive', False) and self.left_controller.grip_active:
                await self.handle_grip_release('left')
            
            # Process right controller
            if right_data.get('position') and (right_data.get('gripActive', False) or right_data.get('trigger', 0) > 0.5):
                # start_time = time.time()
                await self.process_single_controller('right', right_data)
                # end_time = time.time()
                # end_time = time.time()
                # print(f"处理时间(ms): {(end_time - start_time) * 1000}")
            elif not right_data.get('gripActive', False) and self.right_controller.grip_active:
                await self.handle_grip_release('right')
                
            return
        
        # Handle legacy single controller format
        hand = data.get('hand')
        
        # 处理A键事件 - 只在未按住grip时有效
        if data.get('aButtonPressed') and data.get('action') == 'return_to_home':
            grip_active = data.get('gripActive', False)
            # 只有在未按住grip时才处理A键
            if not grip_active:
                await self.handle_a_button_press(hand)
            return
        
        # 处理X键事件 - 只在未按住grip时有效
        if data.get('xButtonPressed') and data.get('action') == 'return_to_home':
            grip_active = data.get('gripActive', False)
            # 只有在未按住grip时才处理A键
            if not grip_active:
                await self.handle_a_button_press(hand)
            return
        
        # Handle explicit release messages
        if data.get('gripReleased'):
            await self.handle_grip_release(hand)
            return
        
        if data.get('triggerReleased'):
            await self.handle_trigger_release(hand)
            return
            
        # Process single controller data
        if hand and data.get('position') and (data.get('gripActive', False) or data.get('trigger', 0) > 0.5):
            # start_time = time.time()
            await self.process_single_controller(hand, data)
            # end_time = time.time()
            # print(f"手柄处理时间: {end_time - start_time:.2f}s")
    
    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        quaternion = data.get('quaternion', {})  # Get quaternion data directly
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        
        controller = self.left_controller if hand == 'left' else self.right_controller

        # 如果有按键状态信息，更新它（用于主控制循环检查）
        if hand == 'left':
            if 'xButtonPressed' in data and data.get('xButtonPressed'):
                # 只有在未按住grip时才设置X键状态
                if not grip_active:
                    controller.x_button_active = True  
                else:
                    controller.x_button_active = False
        elif hand == 'right':  # 右手
            if 'aButtonPressed' in data and data.get('aButtonPressed'):
                # 只有在未按住grip时才设置A键状态
                if not grip_active:
                    controller.a_button_active = True
                else:
                    controller.a_button_active = False
        
        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active
            
            # Send gripper control goal - do not specify mode to avoid interfering with position control
            # Reverse behavior: gripper open by default, closes when trigger pressed
            gripper_goal = ControlGoal(
                arm=hand,
                gripper_closed=not trigger_active,  # Inverted: closed when trigger NOT active
                metadata={"source": "vr_trigger"}
            )
            await self.send_goal(gripper_goal)
            
            # logger.info(f"🤏 {hand.upper()} gripper {'OPENED' if trigger_active else 'CLOSED'}")
        
        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                controller.origin_position = position.copy()
                
                # Use quaternion data directly if available, otherwise fall back to Euler conversion
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                    controller.origin_rotation = controller.origin_quaternion  # Store for compatibility
                else:
                    # Fallback to Euler angle conversion
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                    controller.origin_rotation = controller.origin_quaternion
                
                controller.accumulated_rotation_quat = controller.origin_quaternion
                controller.z_axis_rotation = 0.0
                controller.x_axis_rotation = 0.0
                controller.y_axis_rotation = 0.0
                controller.a_button_active = False  # 重置A键状态
                controller.x_button_active = False  # 重置X键状态
                
                # Send reset signal to control loop to reset target position to current robot position
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,  # Keep in position control
                    target_position=None,  # Special signal
                    metadata={
                        "source": f"vr_grip_reset_{hand}",
                        "reset_target_to_current": True  # Signal to reset target to current position
                    }
                )
                await self.send_goal(reset_goal)
                
                # logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm (target reset to current position)")
            
            # Compute target position
            if controller.origin_position:
                relative_delta = compute_relative_position(
                    position, 
                    controller.origin_position, 
                    self.config.vr_to_robot_scale
                )
                if hand == 'left':
                    self.relative_pos_left = relative_delta
                elif hand == 'right':
                    self.relative_pos_right = relative_delta
                # Calculate Z-axis rotation for wrist_roll control
                # Calculate X-axis rotation for wrist_flex control
                if controller.origin_quaternion is not None:
                    # Update quaternion-based rotation tracking
                    if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                        # Use quaternion data directly
                        current_quat = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                        self.update_quaternion_rotation_direct(controller, current_quat)
                    else:
                        # Fallback to Euler angle conversion
                        self.update_quaternion_rotation(controller, rotation)
                    
                    # Get accumulated rotations from quaternion
                    controller.z_axis_rotation = self.extract_roll_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)
                    controller.x_axis_rotation = self.extract_pitch_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)
                    controller.y_axis_rotation = self.extract_yaw_from_quaternion(controller.accumulated_rotation_quat, controller.origin_quaternion)
                # Create position control goal
                # Note: We send relative position here, the control loop will handle
                # adding it to the robot's current position
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=relative_delta,  # Relative position delta  x y z 位置
                    wrist_roll_deg=-controller.z_axis_rotation,
                    wrist_flex_deg=-controller.x_axis_rotation,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "origin_position": controller.origin_position.copy()
                    }
                )
                await self.send_goal(goal)

    async def handle_grip_release(self, hand: str):
        """Handle grip release for a controller."""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        if controller.grip_active:
            controller.reset_grip()
            
            # Send idle goal to stop arm control
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={"source": "vr_grip_release"}
            )
            await self.send_goal(goal)
            
            # logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
    
    async def handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        if controller.trigger_active:
            controller.trigger_active = False
            
            # Send gripper closed goal - reversed behavior: gripper closes when trigger released
            goal = ControlGoal(
                arm=hand,
                gripper_closed=True,  # Close gripper when trigger released
                metadata={"source": "vr_trigger_release"}
            )
            await self.send_goal(goal)
            
            # logger.info(f"🤏 {hand.upper()} gripper CLOSED (trigger released)")
    
    def euler_to_quaternion(self, euler_deg: Dict[str, float]) -> np.ndarray:
        """Convert Euler angles in degrees to quaternion [x, y, z, w]."""
        euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
        rotation = R.from_euler('xyz', euler_rad)
        return rotation.as_quat()
    
    def update_quaternion_rotation(self, controller: VRControllerState, current_euler: dict):
        """Update quaternion-based rotation tracking."""
        if not current_euler:
            return
        
        # Convert current Euler to quaternion
        current_quat = self.euler_to_quaternion(current_euler)
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
    
    def update_quaternion_rotation_direct(self, controller: VRControllerState, current_quat: np.ndarray):
        """Update quaternion-based rotation tracking using quaternion data directly."""
        if current_quat is None:
            return
        
        # Store current quaternion for accumulated rotation calculation
        controller.accumulated_rotation_quat = current_quat
    
    def extract_roll_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """Extract roll rotation around Z-axis from relative quaternion rotation."""
        if current_quat is None or origin_quat is None:
            return 0.0
        
        try:
            # Calculate relative rotation quaternion (from origin to current)
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # Project the relative rotation onto the Z-axis (roll)
            # Get the rotation vector (axis-angle representation)
            rotvec = relative_rotation.as_rotvec()
            
            # The Z-component of the rotation vector represents rotation around Z-axis (roll)
            z_rotation_rad = rotvec[2]
            z_rotation_deg = -np.degrees(z_rotation_rad)
            
            return z_rotation_deg
        except Exception as e:
            logger.warning(f"Error extracting roll from quaternion: {e}")
            return 0.0
    
    def extract_pitch_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """Extract pitch rotation around X-axis from relative quaternion rotation."""
        if current_quat is None or origin_quat is None:
            return 0.0
        
        try:
            # Calculate relative rotation quaternion (from origin to current)
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # Project the relative rotation onto the X-axis (pitch)
            # Get the rotation vector (axis-angle representation)
            rotvec = relative_rotation.as_rotvec()
            
            # The X-component of the rotation vector represents rotation around X-axis (pitch)
            x_rotation_rad = rotvec[0]
            x_rotation_deg = np.degrees(x_rotation_rad)
            
            return x_rotation_deg
        except Exception as e:
            logger.warning(f"Error extracting pitch from quaternion: {e}")
            return 0.0 

    def extract_yaw_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """Extract yaw rotation around Y-axis from relative quaternion rotation."""
        if current_quat is None or origin_quat is None:
            return 0.0
    
        try:
            # Calculate relative rotation quaternion (from origin to current)
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # Project the relative rotation onto the Y-axis (yaw)
            # Get the rotation vector (axis-angle representation)
            rotvec = relative_rotation.as_rotvec()
            
            # The Y-component of the rotation vector represents rotation around Y-axis (yaw)
            y_rotation_rad = rotvec[1]
            y_rotation_deg = np.degrees(y_rotation_rad)
            
            return y_rotation_deg
        except Exception as e:
            logger.warning(f"Error extracting yaw from quaternion: {e}")
            return 0.0
        

    def quaternion_to_rotation_matrix(self,quaternion: np.ndarray) -> np.ndarray:
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
    
    # 添加A键处理方法
    async def handle_a_button_press(self, hand: str):
        """处理A键按下事件"""
        # logger.info(f"🎮 {hand.upper()} A button pressed - returning to home pose")
        
        # 发送回到初始位姿的控制目标
        a0_goal = ControlGoal(
            arm=hand,
            mode=ControlMode.POSITION_CONTROL,
            target_position=None,  # 特殊标记，表示回到初始位姿
            metadata={
                "source": "vr_a_button",
                "action": "return_to_home"
            }
        )
        await self.send_goal(a0_goal)


    # 添加X键处理方法
    async def handle_x_button_press(self, hand: str):
        """处理X键按下事件"""
        # logger.info(f"🎮 {hand.upper()} X button pressed - returning to home pose")
        
        # 发送回到初始位姿的控制目标
        x0_goal = ControlGoal(
            arm=hand,
            mode=ControlMode.POSITION_CONTROL,
            target_position=None,  # 特殊标记，表示回到初始位姿
            metadata={
                "source": "vr_x_button",
                "action": "return_to_home"
            }
        )
        await self.send_goal(x0_goal)