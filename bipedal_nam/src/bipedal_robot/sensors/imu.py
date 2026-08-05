import zmq
import math
import numpy as np
import time
import json
import logging
import threading
from typing import List, Tuple, Dict, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - [IMU_FUSION] - %(message)s'
)
logger = logging.getLogger(__name__)


class IMUController:
    """ZeroMQ controller to read IMU data from a single leg"""
    
    def __init__(self, host: str, port: int, side: str = "LEFT"):
        self.host = host
        self.port = port
        self.side = side
        
        # ZeroMQ socket setup
        self.context = zmq.Context()
        self.socket = None
        self.lock = threading.Lock()  # Thread lock
        
        # Cache cho IMU data
        self.last_imu_data = {
            "quat": [1, 0, 0, 0],
            "gyro": [0, 0, 0],
            "timestamp": 0
        }
        self.last_valid_time = 0
        
        self.connect()
    
    def connect(self):
        """Create a new socket"""
        with self.lock:  #Protect socket creation
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
            
            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, 2000)  # 2s timeout (faster reconnect)
            self.socket.setsockopt(zmq.LINGER, 0)
            
            try:
                self.socket.connect(f"tcp://{self.host}:{self.port}")
                logger.info(f"✓ {self.side} IMU controller connected to {self.host}:{self.port}")
            except Exception as e:
                logger.error(f"✗ {self.side} connection failed: {e}")
                raise
    
    def read_imu(self) -> Optional[Dict]:
        """Read IMU data - THREAD-SAFE VERSION"""
        try:
            with self.lock:  # Protect socket access
                try:
                    # 1. Send request
                    self.socket.send_json({"type": "feedback"}, zmq.NOBLOCK)
                    
                    # 2. Wait for response (with timeout)
                    response = self.socket.recv_json()
                    
                    # 3. Extract IMU data
                    imu_data = {
                        "quat": response.get("quat", [1, 0, 0, 0]),
                        "gyro": response.get("gyro", [0, 0, 0])
                    }
                    
                    # 4. Validate data
                    if len(imu_data["quat"]) == 4 and len(imu_data["gyro"]) == 3:
                        self.last_imu_data = imu_data
                        self.last_valid_time = time.time()
                        return imu_data
                    else:
                        logger.debug(f"{self.side}: Invalid IMU data format, using cache")
                        return self.last_imu_data
                
                except zmq.Again:
                    logger.debug(f"{self.side}: Timeout waiting for IMU, using cache")
                    self.socket.close()  # ✅ FIX: Close broken socket
                    self.socket = self.context.socket(zmq.REQ)  # Create new socket
                    self.socket.setsockopt(zmq.RCVTIMEO, 2000)
                    self.socket.setsockopt(zmq.LINGER, 0)
                    self.socket.connect(f"tcp://{self.host}:{self.port}")
                    return self.last_imu_data
                
                except zmq.error.ZMQError as e:
                    if "Operation cannot be accomplished in current state" in str(e):
                        logger.debug(f"{self.side}: Socket state error, recreating...")
                        self.socket.close()
                        self.socket = self.context.socket(zmq.REQ)
                        self.socket.setsockopt(zmq.RCVTIMEO, 2000)
                        self.socket.setsockopt(zmq.LINGER, 0)
                        self.socket.connect(f"tcp://{self.host}:{self.port}")
                    else:
                        logger.debug(f"{self.side}: ZMQ Error - {e}")
                    return self.last_imu_data
                
        except Exception as e:
            logger.debug(f"{self.side}: Error in read_imu - {e}")
            return self.last_imu_data
    
    def close(self):
        """Close connection"""
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass


class IMUFusion:
    """API to fuse IMU data from 2 leg sensors"""
    
    def __init__(self, left_host: str = "127.0.0.1", left_port: int = 5556,
                 right_host: str = "127.0.0.1", right_port: int = 5555):
        """
        Initialize IMU Fusion
        
        Args:
            left_host: Host of left leg IMU server
            left_port: Port of left leg IMU server
            right_host: Host of right leg IMU server
            right_port: Port of right leg IMU server
        """
        # Create controller for each leg
        self.left = IMUController(left_host, left_port, "LEFT")
        self.right = IMUController(right_host, right_port, "RIGHT")
        
        # IMU positions (from URDF)
        self.left_pos = np.array([-2.3585E-05, -0.015, 0.0851])
        self.right_pos = np.array([0, 0.015, 0.0851])
        
        # Rotation matrices (from URDF rpy)
        roll_left = 90 * math.pi / 180
        pitch_left = 0 * math.pi / 180
        yaw_left = -90 * math.pi / 180
        self.left_rot = self.euler_to_rot(roll_left, pitch_left, yaw_left)

        roll_right = 90 * math.pi / 180
        pitch_right = 0 * math.pi / 180
        yaw_right = 90 * math.pi / 180
        self.right_rot = self.euler_to_rot(roll_right, pitch_right, yaw_right)

        self.left_rot_gyro = self.euler_to_rot_gyro(roll_left, pitch_left, yaw_left)
        self.right_rot_gyro = self.euler_to_rot_gyro(roll_right, pitch_right, yaw_right)
        self.left_q_rot = self.euler_to_quat(roll_left, pitch_left, yaw_left)
        self.right_q_rot = self.euler_to_quat(roll_right, pitch_right, yaw_right)
    
    def quat_normalize(self, q: List[float]) -> List[float]:
        """Normalize quaternion to unit norm"""
        norm = math.sqrt(sum(x**2 for x in q))
        if norm > 1e-10:
            return [x / norm for x in q]
        else:
            return [1, 0, 0, 0]

    def euler_to_rot(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert Euler angles to rotation matrix"""
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        
        return Rz @ Ry @ Rx
    
    def euler_to_rot_gyro(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert Euler angles to rotation matrix"""
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])

        return Rx @ Rz @ Ry

    def euler_to_quat(self, roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]
    
    def quat_mult(self, q1: List[float], q2: List[float]) -> List[float]:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        result = [
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ]
        return self.quat_normalize(result)
    
    def quat_conj(self, q: List[float]) -> List[float]:
        """Quaternion conjugate"""
        return [q[0], -q[1], -q[2], -q[3]]
    
    def transform_quat_to_baselink(self, q_imu: List[float], imu_name: str) -> List[float]:
        """Transform quaternion from IMU frame to Baselink frame"""
        if imu_name == "left":
            angle = -math.pi / 2
            axis = np.array([0, 0, 1])
        else:
            angle = math.pi / 2
            axis = np.array([0, 0, 1])
        
        half_angle = angle / 2
        sin_half = math.sin(half_angle)
        cos_half = math.cos(half_angle)
        
        q_rot = [
            cos_half,
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half
        ]
        
        q_rot = self.quat_normalize(q_rot)
        q_imu = self.quat_normalize(q_imu)
        
        q_temp = self.quat_mult(q_rot, q_imu)
        q_baselink = self.quat_mult(q_temp, self.quat_conj(q_rot))
        
        return self.quat_normalize(q_baselink)
    
    def transform_pos_to_baselink(self, pos: np.ndarray, imu_name: str) -> np.ndarray:
        """Transform position from IMU frame to Baselink frame"""
        if imu_name == "left":
            return self.left_rot @ pos
        else:
            return self.right_rot @ pos
    
    def transform_gyro_to_baselink(self, gyro: np.ndarray, imu_name: str) -> np.ndarray:
        """Transform gyro data from IMU frame to Baselink frame"""
        gyro_array = np.array(gyro, dtype=float)

        if imu_name == "left":
            angle = -math.pi / 2  # -90 độ quanh Z
        else:
            angle = math.pi / 2   # +90 độ quanh Z
        
        # Rotation matrix quanh trục Z
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        Rz = np.array([[cos_a, -sin_a, 0],
                    [sin_a, cos_a,  0],
                    [0,     0,      1]])
        
        # Áp dụng transpose (vì gyro là angular velocity)
        gyro_transformed = Rz.T @ gyro_array
        gyro_transformed[0] *= -1  # Flip GX
        gyro_transformed[1] *= -1  # Flip GY

        return gyro_transformed
    
    def fuse_gyro(self, gyro_left: np.ndarray, gyro_right: np.ndarray, 
                  weight_left: float = 0.5) -> np.ndarray:
        """Fuse gyro data from two IMUs using weighted average"""
        weight_right = 1.0 - weight_left
        gyro_left = np.array(gyro_left, dtype=float)
        gyro_right = np.array(gyro_right, dtype=float)
        fused = weight_left * gyro_left + weight_right * gyro_right
        return fused

    def quat_to_euler(self, q: List[float]) -> Tuple[float, float, float]:
        """Convert quaternion to Euler angles (radians)"""
        w, x, y, z = q
        
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-10:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def fuse_quat(self, q1: List[float], q2: List[float], weight1: float = 0.5) -> List[float]:
        """Fuse two quaternions with weighted average"""
        weight2 = 1 - weight1
        
        q1 = self.quat_normalize(q1)
        q2 = self.quat_normalize(q2)

        dot = sum(q1[i]*q2[i] for i in range(4))
        if dot < 0:
            q2 = [-x for x in q2]
        
        fused = [q1[i]*weight1 + q2[i]*weight2 for i in range(4)]
        
        return self.quat_normalize(fused)
    
    def get_fused_imu(self) -> Optional[Dict]:
        """Read and fuse IMU data from 2 leg sensors"""
        try:
            # Request IMU data (thread-safe now)
            left_raw = self.left.read_imu()
            right_raw = self.right.read_imu()
            
            if not left_raw or not right_raw:
                logger.debug("Failed to read IMU data")
                return None
            
            left_quat = left_raw.get("quat", [1, 0, 0, 0])
            right_quat = right_raw.get("quat", [1, 0, 0, 0])
            left_gyro_raw = left_raw.get("gyro", [0, 0, 0])
            right_gyro_raw = right_raw.get("gyro", [0, 0, 0])
            
            left_q = self.transform_quat_to_baselink(left_quat, "left")
            right_q = self.transform_quat_to_baselink(right_quat, "right")
            fused_q = self.fuse_quat(left_q, right_q)
            
            left_gyro_transformed = self.transform_gyro_to_baselink(left_gyro_raw, "left")
            right_gyro_transformed = self.transform_gyro_to_baselink(right_gyro_raw, "right")
            fused_gyro = self.fuse_gyro(left_gyro_transformed, right_gyro_transformed)
            
            fused_euler = self.quat_to_euler(fused_q)
            
            return {
                'left_quat': left_q,
                'right_quat': right_q,
                'fused_quat': fused_q,
                'fused_euler': fused_euler,
                'left_gyro': left_gyro_transformed.tolist(),
                'right_gyro': right_gyro_transformed.tolist(),
                'fused_gyro': fused_gyro.tolist(),
                'timestamp': time.time()
            }
        
        except Exception as e:
            logger.error(f"Error in get_fused_imu: {e}")
            return None
    
    def close(self):
        """Close all connections"""
        self.left.close()
        self.right.close()