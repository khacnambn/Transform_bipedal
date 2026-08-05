"""
Transformer API for Bipedal Robot Control
Simplified version based on walking_gait_dual.py logic
- IMU: Only Euler angles (orient) like Bimo API
- Servo: Control only 3 joints per leg (hip, knee, foot) like walking_gait_dual.py
- Architecture: Distributed (2 leg servers via ZeroMQ)
"""

import sys
import time
import math
import zmq  # ⭐ THÊM: Import ZeroMQ
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import logging
import threading

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from bipedal_robot.sensors.imu import IMUFusion

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - [TRANSFORMER] - %(message)s'
)
logger = logging.getLogger(__name__)


class TransformerAPI:
    """
    High-level API for Bipedal Robot control using leg servers and IMU fusion
    Similar to Bimo API but distributed with 2 leg servers
    """
    
    def __init__(self, 
                 left_host: str = "mobile2.local", 
                 left_port: int = 5556,
                 right_host: str = "mobile1.local", 
                 right_port: int = 5555):
        """
        Initialize Transformer API
        
        Args:
            left_host: Hostname/IP of left leg server
            left_port: Port of left leg server
            right_host: Hostname/IP of right leg server
            right_port: Port of right leg server
        """
        self.left_host = left_host
        self.left_port = left_port
        self.right_host = right_host
        self.right_port = right_port
        
        # ⭐ THÊM: Initialize socket dictionaries
        self.req_context = None      # ← REQ/REP context
        self.push_context = None     # ← PUSH/PULL context
        self.sockets_req = {}        # ← {leg: socket}
        self.sockets_push = {}       # ← {leg: socket}
        
        # IMU Fusion module
        self.imu_fusion = None
        
        # Robot state (similar to Bimo API structure)
        self.state_data = {
            "orient": (0.0, 0.0, 0.0),  # (roll, pitch, yaw) in radians - LIKE BIMO
            "gyro": [0.0, 0.0, 0.0],
            "servo": {
                "left": {
                    "pos": [0] * 6,      # All 6 positions (but only control 3)
                    "speed": [0] * 6,
                    "load": [0] * 6,
                    "voltage": [0] * 6,
                    "current": [0] * 6,
                    "temp": [0] * 6,
                },
                "right": {
                    "pos": [0] * 6,
                    "speed": [0] * 6,
                    "load": [0] * 6,
                    "voltage": [0] * 6,
                    "current": [0] * 6,
                    "temp": [0] * 6,
                },
            },
        }
        
        # Home positions (from server config - motors 4-9)
        # Indices: 0=motor4, 1=hip(5), 2=motor6, 3=knee(7), 4=foot(8), 5=motor9
        self.home_pos_left = [3366, 1958, 1042, 2317, 1584, 1390] # decrease
        self.home_pos_right = [415, 2642, 2702, 2505, 2185, 2048] # increase
        
        # Initial pose (from walking_gait_dual.py)
        self.initial_hip_deg = 25
        self.initial_knee_deg = -50
        self.initial_foot_deg = 25
        
        # Servo configuration (like walking_gait_dual.py)
        self.servo_config_left = {
            5: {"name": "hip",  "home_ticks": 1988, "min_ticks": 1545, "max_ticks": 2365},
            7: {"name": "knee", "home_ticks": 2287, "min_ticks": 1313, "max_ticks": 3231},
            8: {"name": "foot", "home_ticks": 1614, "min_ticks": 1111, "max_ticks": 2363},
        }
        
        self.servo_config_right = {
            5: {"name": "hip",  "home_ticks": 2612, "min_ticks": 2240, "max_ticks": 3075},
            7: {"name": "knee", "home_ticks": 2535, "min_ticks": 1635, "max_ticks": 3545},
            8: {"name": "foot", "home_ticks": 2155, "min_ticks": 1413, "max_ticks": 2532},
        }
        
        # ✅ THÊM: IMU cache + thread
        self.imu_cache = {
            "orient": (0.0, 0.0, 0.0),
            "gyro": [0.0, 0.0, 0.0],
            "timestamp": 0
        }
        self.imu_lock = threading.Lock()
        self.imu_thread = None
        self.imu_running = False
        
        logger.info("TransformerAPI initialized")
    
    def _imu_background_loop(self):
        """✅ THÊM: IMU thread - 50Hz continuous update"""
        logger.info("🔄 IMU background thread started (50Hz)")
        
        sample_count = 0
        valid_count = 0
        
        while self.imu_running:
            try:
                # Get fresh IMU data
                imu_data = self.imu_fusion.get_fused_imu()
                sample_count += 1
                
                if imu_data:
                    valid_count += 1
                    
                    # ✅ LƯU vào cache atomically
                    with self.imu_lock:
                        roll, pitch, yaw = imu_data["fused_euler"]
                        self.imu_cache["orient"] = (roll, pitch, yaw)
                        self.imu_cache["gyro"] = imu_data["fused_gyro"]
                        self.imu_cache["timestamp"] = time.time()
                    
                    # ✅ Log every 50 samples (1 second)
                    if valid_count % 50 == 0:
                        logger.info(f"✓ IMU: {valid_count} valid samples | "
                                   f"orient=({roll:+.4f}, {pitch:+.4f}, {yaw:+.4f}) | "
                                   f"gyro=({imu_data['fused_gyro'][0]:+.4f}, {imu_data['fused_gyro'][1]:+.4f}, {imu_data['fused_gyro'][2]:+.4f})")
                else:
                    logger.warning(f"⚠️  Sample {sample_count}: IMU returned None")
                
                # 50Hz = 20ms
                time.sleep(0.02)
                
            except Exception as e:
                logger.error(f"IMU thread error: {e}")
                time.sleep(0.02)
    
        logger.info(f"IMU background thread stopped ({valid_count}/{sample_count} valid)")
    
    def initialize(self, warmup_time: float = 5.0) -> bool:
        """Initialize with 2 socket types"""
        logger.info("=" * 80)
        logger.info("TRANSFORMER API INITIALIZATION")
        logger.info("=" * 80)
        
        try:
            # Initialize REQ/REP (data queries)
            logger.info("\nConnecting REQ/REP sockets (data)...")
            self.req_context = zmq.Context()
            for leg in ["left", "right"]:
                socket = self.req_context.socket(zmq.REQ)
                socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5s timeout
                
                host = self.left_host if leg == "left" else self.right_host
                port = self.left_port if leg == "left" else self.right_port
                
                socket.connect(f"tcp://{host}:{port}")
                self.sockets_req[leg] = socket
                logger.info(f"✓ REQ socket connected to {leg} ({host}:{port})")
            
            # Initialize PUSH/PULL (control commands)
            logger.info("\nConnecting PUSH/PULL sockets (control)...")
            self.push_context = zmq.Context()
            for leg in ["left", "right"]:
                socket = self.push_context.socket(zmq.PUSH)
                socket.setsockopt(zmq.LINGER, 0)  # Don't wait on close
                
                host = self.left_host if leg == "left" else self.right_host
                port = (self.left_port + 100) if leg == "left" else (self.right_port + 100)
                
                socket.connect(f"tcp://{host}:{port}")
                self.sockets_push[leg] = socket
                logger.info(f"✓ PUSH socket connected to {leg} ({host}:{port})")
            
            # # Initialize IMU Fusion
            # logger.info("\nInitializing IMU Fusion...")
            # self.imu_fusion = IMUFusion(
            #     left_host=self.left_host,
            #     left_port=self.left_port,
            #     right_host=self.right_host,
            #     right_port=self.right_port
            # )
            # logger.info("IMU Fusion initialized")
            
            # # INITIAL WARMUP (5 giây) - Cho servers sẵn sàng
            # logger.info("\n Waiting for servers to warm up IMU (5.0s)...")
            
            # # ✅ FIX: Dùng print() thay vì logger.info() để có end="\r"
            # for i in range(5, 0, -1):
            #     print(f"   {i}s remaining...", end="\r")
            #     time.sleep(1)
            # print("  Servers ready!                    ")
            
            # Warmup IMU - WITH RETRY LOGIC
            # logger.info(f"\n⏳ Warming up IMU ({warmup_time:.1f}s)...")
            # warmup_start = time.time()
            # imu_samples = 0
            # valid_count = 0
            # consecutive_failures = 0
            # max_consecutive_failures = 5
            
            # while time.time() - warmup_start < warmup_time:
            #     imu_data = self.imu_fusion.get_fused_imu()
            #     time.sleep(0.02)
            #     imu_samples += 1
                
            #     if imu_data:
            #         valid_count += 1
            #         consecutive_failures = 0
                    
            #         if valid_count % 50 == 0:
            #             logger.info(f"  ✓ IMU valid: {valid_count} samples")
            #     else:
            #         consecutive_failures += 1
                    
            #         if consecutive_failures > max_consecutive_failures:
            #             logger.warning(f"  ⚠️  {consecutive_failures} consecutive failures - reconnecting...")
            #             self.imu_fusion.left.connect()
            #             self.imu_fusion.right.connect()
            #             consecutive_failures = 0
            #             logger.info(f"  ✓ Reconnected, continuing warmup...")
                    
            #         if valid_count == 0 and imu_samples % 50 == 0:
            #             logger.debug(f"  ⏳ Waiting for IMU... ({imu_samples*20}ms)")
            
            # # ✅ MOVE THIS OUTSIDE THE WHILE LOOP (but still inside try)
            # if valid_count == 0:
            #     logger.error("❌ IMU data not received after warmup!")
            #     return False
            
            # logger.info(f"✅ Warmup complete: {imu_samples} samples, {valid_count} valid")
            
            # Move both legs to HOME position first
            logger.info("\n🏠 Moving both legs to HOME position...")
            if not self.move_legs_to_home(speed=500):  
                logger.error("❌ Failed to move to home")
                return False
            
            time.sleep(1)
            
            # Then move to INITIAL pose
            logger.info("\n🎯 Moving both legs to INITIAL POSE...")
            if not self.move_legs_to_initial_pose(speed=500):
                logger.error("❌ Failed to move to initial pose")
                return False
            
            # Get initial feedback
            logger.info("\n📊 Reading initial servo feedback...")
            feedback_left = self._get_feedback("left")
            feedback_right = self._get_feedback("right")
            
            if feedback_left and feedback_right:
                logger.info("✅ Servo feedback obtained")
                self._update_state(feedback_left, feedback_right)
            else:
                logger.warning("⚠️  Some servo feedback failed")
            
            logger.info("\n" + "=" * 80)
            logger.info("✅ TRANSFORMER API READY")
            logger.info("=" * 80 + "\n")
            
            # ✅ THÊM: Start IMU background thread AFTER warmup
            logger.info("\n🔄 Starting IMU background thread...")
            self.imu_running = True
            self.imu_thread = threading.Thread(target=self._imu_background_loop, daemon=True)
            self.imu_thread.start()
            logger.info("✓ IMU background thread started\n")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Initialization failed: {e}")
            self.imu_running = False
            return False
    
    def _send_control_command(self, leg: str, positions: list) -> bool:
        """
        Send control command via PUSH (async, fire-and-forget)
        """
        try:
            if leg not in self.sockets_push:
                logger.error(f"❌ No PUSH socket for {leg}")
                return False
            
            socket = self.sockets_push[leg]
            socket.send_json({"type": "move", "positions": positions})
            logger.debug(f"✓ PUSH control sent to {leg}: {positions}")
            return True
            
        except Exception as e:
            logger.error(f"Error sending PUSH command to {leg}: {e}")
            return False
    
    def _send_command(self, leg: str, positions: list = None) -> bool:
        """
        Send query via REQ/REP (sync, wait for response)
        Used for feedback/data queries, not real-time control
        """
        try:
            if leg not in self.sockets_req:
                logger.error(f"❌ No REQ socket for {leg}")
                return False
            
            socket = self.sockets_req[leg]
            
            if positions:
                socket.send_json({"type": "move", "positions": positions})
            else:
                socket.send_json({"type": "feedback"})
            
            response = socket.recv_json()
            
            if response.get("status") == "success":
                logger.debug(f"✓ REQ response from {leg}")
                return True
            else:
                logger.warning(f"⚠️  REQ failed from {leg}: {response.get('message')}")
                return False
            
        except zmq.Again:
            logger.error(f"❌ REQ timeout from {leg}")
            return False
        except Exception as e:
            logger.error(f"Error in REQ command to {leg}: {e}")
            return False
    
    def _get_feedback(self, leg: str, max_retries: int = 3) -> Optional[Dict]:
        """
        ✅ Get feedback via REQ/REP (sync, reliable)
        """
        for attempt in range(max_retries):
            try:
                if leg not in self.sockets_req:
                    logger.error(f"❌ No REQ socket for {leg}")
                    return None
                
                socket = self.sockets_req[leg]
                socket.send_json({"type": "feedback"})
                
                response = socket.recv_json()
                
                if response.get("status") == "success":
                    # logger.info(f"✓ {leg} feedback received")
                    return response
                else:
                    # logger.warning(f"{leg}: Feedback rejected")
                    if attempt < max_retries - 1:
                        time.sleep(0.5)
                    continue
                
            except zmq.Again:
                logger.warning(f"{leg}: Feedback timeout attempt {attempt+1}/{max_retries}")
                if attempt < max_retries - 1:
                    time.sleep(0.5)
                continue
            
            except Exception as e:
                logger.error(f"{leg}: Feedback error: {e}")
                if attempt < max_retries - 1:
                    time.sleep(0.5)
                continue
        
        logger.error(f"❌ {leg}: Feedback failed after {max_retries} attempts")
        return None
    
    def move_legs_to_home(self, speed: int = None) -> bool:
        """
        Move both legs to HOME position with optional custom speed
        
        Args:
            speed: Custom speed for this move (if None, use server default)
        """
        logger.info(f"Moving both legs to HOME...")
        time.sleep(0.5)
        
        try:
            if speed is not None:
                logger.info(f"  Setting custom speed: {speed}")
                self._configure_servo_speed("left", speed)
                self._configure_servo_speed("right", speed)
            
            logger.info("Sending HOME commands (async PUSH)...")
            self._send_control_command("left", self.home_pos_left)
            self._send_control_command("right", self.home_pos_right)
            
            logger.info("✅ HOME commands sent")
            time.sleep(3)
            
            if speed is not None:
                logger.info(f"  Resetting to default speed")
                self._configure_servo_speed("left", None)  # Reset
                self._configure_servo_speed("right", None)
            
            return True
            
        except Exception as e:
            logger.error(f"Error: {e}")
            return False

    def move_legs_to_initial_pose(self, speed: int = None) -> bool:
        """
        Move both legs to INITIAL POSE with optional custom speed
        
        Args:
            speed: Custom speed for this move (if None, use server default)
        """
        logger.info(f"Moving both legs to INITIAL POSE...")
        time.sleep(0.5)
        
        initial_pos_left = self.home_pos_left.copy()
        initial_pos_left[1] = self.degree_to_ticks("left", 5, self.initial_hip_deg)
        initial_pos_left[3] = self.degree_to_ticks("left", 7, self.initial_knee_deg)
        initial_pos_left[4] = self.degree_to_ticks("left", 8, self.initial_foot_deg)
        
        initial_pos_right = self.home_pos_right.copy()
        initial_pos_right[1] = self.degree_to_ticks("right", 5, self.initial_hip_deg)
        initial_pos_right[3] = self.degree_to_ticks("right", 7, self.initial_knee_deg)
        initial_pos_right[4] = self.degree_to_ticks("right", 8, self.initial_foot_deg)
        
        try:
            if speed is not None:
                logger.info(f"  Setting custom speed: {speed}")
                self._configure_servo_speed("left", speed)
                self._configure_servo_speed("right", speed)
            
            logger.info("Sending INITIAL POSE commands (async PUSH)...")
            self._send_control_command("left", initial_pos_left)
            self._send_control_command("right", initial_pos_right)
            
            logger.info("✅ INITIAL POSE commands sent")
            time.sleep(3)
            
            if speed is not None:
                logger.info(f"  Resetting to default speed")
                self._configure_servo_speed("left", None)  # Reset
                self._configure_servo_speed("right", None)
            
            return True
            
        except Exception as e:
            logger.error(f"Error: {e}")
            return False

    def _configure_servo_speed(self, leg: str, speed: int = None) -> bool:
        if speed is None:
            return True  # Skip if no speed specified
        
        try:
            if leg not in self.sockets_req:
                logger.error(f"❌ No REQ socket for {leg}")
                return False
            
            socket = self.sockets_req[leg]
            socket.send_json({"type": "config", "speed": speed})
            
            # ✅ FIX: BỎ timeout parameter - đã set ở init với RCVTIMEO
            response = socket.recv_json()
            
            if response.get("status") == "success":
                logger.debug(f"✓ {leg} servo speed set to {speed}")
                return True
            else:
                logger.warning(f"⚠️  Failed to set speed for {leg}")
                return False
                
        except zmq.Again:
            logger.error(f"❌ Timeout setting speed for {leg}")
            return False
        except Exception as e:
            logger.error(f"Error setting speed for {leg}: {e}")
            return False
    
    def set_joint_angles(self, 
                        left_hip: float, left_knee: float, left_foot: float,
                        right_hip: float, right_knee: float, right_foot: float) -> bool:
        """
        ✅ REAL-TIME: Use PUSH (async) for policy control
        This ensures immediate motor updates without blocking!
        """
        try:
            # Build positions
            pos_left = self.home_pos_left.copy()
            pos_left[1] = self.degree_to_ticks("left", 5, left_hip)
            pos_left[3] = self.degree_to_ticks("left", 7, left_knee)
            pos_left[4] = self.degree_to_ticks("left", 8, left_foot)
            
            pos_right = self.home_pos_right.copy()
            pos_right[1] = self.degree_to_ticks("right", 5, right_hip)
            pos_right[3] = self.degree_to_ticks("right", 7, right_knee)
            pos_right[4] = self.degree_to_ticks("right", 8, right_foot)
            
            # ⭐ Use PUSH (async, non-blocking) for real-time control
            self._send_control_command("left", pos_left)
            self._send_control_command("right", pos_right)
            
            return True
                
        except Exception as e:
            logger.error(f"Error setting joint angles: {e}")
            return False
    
    def _update_state(self, feedback_left: Dict, feedback_right: Dict) -> None:
        """Update internal state from feedback"""
        try:
            # Update left servo data
            if "servo_pos" in feedback_left:
                self.state_data["servo"]["left"]["pos"] = feedback_left["servo_pos"]
            if "servo_speed" in feedback_left:
                self.state_data["servo"]["left"]["speed"] = feedback_left["servo_speed"]
            if "servo_load" in feedback_left:
                self.state_data["servo"]["left"]["load"] = feedback_left["servo_load"]
            
            # Update right servo data
            if "servo_pos" in feedback_right:
                self.state_data["servo"]["right"]["pos"] = feedback_right["servo_pos"]
            if "servo_speed" in feedback_right:
                self.state_data["servo"]["right"]["speed"] = feedback_right["servo_speed"]
            if "servo_load" in feedback_right:
                self.state_data["servo"]["right"]["load"] = feedback_right["servo_load"]
            
        except Exception as e:
            logger.error(f"Error updating state: {e}")
    
    def degree_to_ticks(self, leg: str, servo_id: int, degrees_relative: float) -> int:
        """
        Convert degrees to ticks (like walking_gait_dual.py)
        
        Args:
            leg: "left" or "right"
            servo_id: 5 (hip), 7 (knee), or 8 (foot)
            degrees_relative: Angle relative to home (degrees)
        
        Returns:
            Servo position in ticks
        """
        if leg == "right":
            config = self.servo_config_right[servo_id]
        else:  # left
            config = self.servo_config_left[servo_id]
        
        home_ticks = config["home_ticks"]
        min_ticks = config["min_ticks"]
        max_ticks = config["max_ticks"]
        
        if leg == "right":
            # RIGHT leg: normal (positive degrees = higher ticks)
            min_deg = (min_ticks - home_ticks) / 4096.0 * 360.0
            max_deg = (max_ticks - home_ticks) / 4096.0 * 360.0
            degrees_relative = max(min_deg, min(max_deg, degrees_relative))
            ticks = home_ticks + (degrees_relative / 360.0) * 4096.0
        else:
            # LEFT leg: inverted (positive degrees = lower ticks)
            ticks_range_forward = home_ticks - min_ticks
            ticks_range_backward = max_ticks - home_ticks
            
            if degrees_relative >= 0:
                max_angle_forward = (ticks_range_forward / 4096.0) * 360.0
                clipped_deg = min(degrees_relative, max_angle_forward)
                ticks = home_ticks - (clipped_deg / 360.0) * 4096.0
            else:
                max_angle_backward = (ticks_range_backward / 4096.0) * 360.0
                clipped_deg = max(degrees_relative, -max_angle_backward)
                ticks = home_ticks - (clipped_deg / 360.0) * 4096.0
        
        ticks = max(min_ticks, min(max_ticks, int(ticks)))
        return ticks
    
    def ticks_to_degree(self, leg: str, servo_id: int, ticks: int) -> float:
        """
        Convert ticks to degrees (like walking_gait_dual.py)
        
        Args:
            leg: "left" or "right"
            servo_id: 5 (hip), 7 (knee), or 8 (foot)
            ticks: Servo position in ticks
        
        Returns:
            Angle relative to home (degrees)
        """
        if leg == "right":
            config = self.servo_config_right[servo_id]
        else:  # left
            config = self.servo_config_left[servo_id]
        
        home_ticks = config["home_ticks"]
        
        if leg == "right":
            degrees_relative = (ticks - home_ticks) / 4096.0 * 360.0
        else:
            degrees_relative = (home_ticks - ticks) / 4096.0 * 360.0
        
        return round(degrees_relative, 2)
    
    def get_state(self) -> Dict:
        """Get current robot state (for compatibility)"""
        return self.state_data
    
    def get_state_with_retry(self, max_retries: int = 3) -> Optional[Dict]:
        """
        ✅ THAY ĐỔI: Lấy IMU từ cache (không fetch real-time)
        Real-time updates đã handle bởi background thread
        """
        try:
            # ✅ LẤY từ cache (non-blocking)
            with self.imu_lock:
                orient = self.imu_cache["orient"]
                gyro = self.imu_cache["gyro"]
                timestamp = self.imu_cache["timestamp"]
            
            # Check if cache is stale (older than 200ms)
            age = time.time() - timestamp
            if age > 0.2 and timestamp > 0:  # Allow first 0 timestamp
                logger.warning(f"⚠️  IMU cache stale ({age*1000:.0f}ms old)")
            
            # ✅ UPDATE state_data từ cache
            self.state_data["orient"] = orient
            self.state_data["gyro"] = gyro
            
            # Get servo feedback (keep this as-is, servo updates are okay)
            feedback_left = self._get_feedback("left", max_retries=1)
            feedback_right = self._get_feedback("right", max_retries=1)
            
            if feedback_left and feedback_right:
                self._update_state(feedback_left, feedback_right)
            
            return self.state_data
        
        except Exception as e:
            logger.error(f"Error getting state: {e}")
            return None
    
    def shutdown(self) -> None:
        """Shutdown API"""
        logger.info("Shutting down TransformerAPI...")
        
        # ✅ THÊM: Stop IMU thread
        self.imu_running = False
        if self.imu_thread:
            self.imu_thread.join(timeout=2.0)
            logger.info("IMU thread stopped")
        
        # Close REQ sockets
        for socket in self.sockets_req.values():
            try:
                socket.close()
            except:
                pass
        if self.req_context:
            self.req_context.term()
        
        # Close PUSH sockets
        for socket in self.sockets_push.values():
            try:
                socket.close()
            except:
                pass
        if self.push_context:
            self.push_context.term()
        
        if self.imu_fusion:
            self.imu_fusion.left.close()
            self.imu_fusion.right.close()
        
        logger.info("✅ Shutdown complete")

if __name__ == "__main__":
    # Create API instance
    robot = TransformerAPI(
        left_host="mobile2.local",
        left_port=5556,
        right_host="mobile1.local",
        right_port=5555
    )
    
    # Initialize
    if not robot.initialize():
        print("❌ Failed to initialize")
        exit(1)
    
    try:
        print("\n" + "=" * 80)
        print("🚶 DEMONSTRATION: Policy-controlled gait")
        print("=" * 80)
        
        # Get initial state
        state = robot.get_state()
        print(f"\n📊 Initial state:")
        print(f"  Orient (roll, pitch, yaw): {state['orient']}")
        print(f"  Left servo pos: {state['servo']['left']['pos'][:3]}...")
        print(f"  Right servo pos: {state['servo']['right']['pos'][:3]}...")
        
        # Simulate policy control loop
        print(f"\n🔄 Simulating 3 gait cycles...")
        num_cycles = 3
        steps_per_cycle = 30
        
        for cycle in range(num_cycles):
            print(f"\n  Cycle {cycle + 1}/{num_cycles}")
            
            for step in range(steps_per_cycle):
                t = step / steps_per_cycle  # 0 → 1
                
                # Synchronized swing like walking_gait_dual.py
                # RIGHT leg: normal phase
                swing_r = math.sin(2 * math.pi * t)
                hip_r = robot.initial_hip_deg + swing_r * 15
                knee_r = robot.initial_knee_deg + swing_r * 20
                foot_r = -(hip_r + knee_r)
                
                # LEFT leg: opposite phase
                swing_l = math.sin(2 * math.pi * (t + 0.5))
                hip_l = robot.initial_hip_deg + swing_l * 15
                knee_l = robot.initial_knee_deg + swing_l * 20
                foot_l = -(hip_l + knee_l)
                
                # Apply joint angles
                success = robot.set_joint_angles(
                    hip_l, knee_l, foot_l,
                    hip_r, knee_r, foot_r
                )
                
                if not success:
                    print(f"    ❌ Step {step} failed")
                    break
                
                time.sleep(0.03)
            
            print(f"    ✓ Cycle complete")
        
        # Return to initial pose
        print(f"\n🎯 Returning to initial pose...")
        robot.move_legs_to_initial_pose()
        
        # Return to home
        print(f"\n🏠 Returning to home...")
        robot.move_legs_to_home()
        
        print("\n" + "=" * 80)
        print("✅ Demonstration complete!")
        print("=" * 80)
        
    except KeyboardInterrupt:
        print("\n\n👋 Interrupted by user")
    
    finally:
        robot.shutdown()