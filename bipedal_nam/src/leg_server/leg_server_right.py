import zmq
import json
import struct
import time
import threading
from typing import Dict, List, Optional
import logging
from collections import deque
import sys
from pathlib import Path
import qwiic_icm20948
import math
from ahrs.filters import Madgwick
import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from bipedal_robot import BipedalRobot, BipedalConfig

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - [MCU_SERVER_RIGHT] - %(message)s'
)
logger = logging.getLogger(__name__)

# ==============================
# Load calibration
# ==============================
with open("/home/mobile1/leg1_bipedal/imu/imu_calib.json", "r") as f:
    calib = json.load(f)

ax_bias = calib["ax_bias"]
ay_bias = calib["ay_bias"]
az_bias = calib["az_bias"]

ax_scale = calib["ax_scale"]
ay_scale = calib["ay_scale"]
az_scale = calib["az_scale"]

gx_bias = calib.get("gx_bias", 0)
gy_bias = calib.get("gy_bias", 0)
gz_bias = calib.get("gz_bias", 0)
GYRO_SENSITIVITY = calib.get("gyro_sensitivity", 65.536)  # ±500°/s range

SENSITIVITY = 16384.0
G = 9.81

ACCEL_ALPHA = 0.15
GYRO_ALPHA = 0.15

filtered_ax, filtered_ay, filtered_az = 0.0, 0.0, 0.0
filtered_gx, filtered_gy, filtered_gz = 0.0, 0.0, 0.0

# Initialize IMU
IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    logger.error("IMU not found! Exiting...")
    exit()

IMU.begin()
logger.info("ICM-20948 IMU initialized (calibrated, output in m/s²)")
madgwick = Madgwick(frequency=50, beta=0.1)
madgwick.q0 = np.array([1.0, 0.0, 0.0, 0.0])  # Initialize with identity quaternion


class MCUServer:
    """Main MCU control server for Bimo robot on Raspberry Pi 5."""

    def __init__(self, zmq_port: int = 5555, serial_port: str = "/dev/ttyACM0"):
        """
        Initialize MCU server - CHỈ cho 6 leg motors (servo 4-9).
        Bỏ qua base motors (servo 1-3).

        Args:
            zmq_port: ZeroMQ socket port for receiving commands
            serial_port: Serial port for servo communication
        """
        self.zmq_port = zmq_port
        self.serial_port = serial_port
        self.running = False

        # Robot instance (BipedalRobot from bipedal.py)
        self.config = BipedalConfig(port=serial_port, baudrate=1_000_000)
        self.robot = None

        # ✅ SỬA: Servo mapping CHỈ motor 4-9 (leg motors)
        self.servo_map = {
            4: "bubright_joint",               # leg_bub (sts3095)
            5: "hipright_joint",               # leg_hip (sts3095)
            6: "twistright_joint",             # leg_twist (sts3215)
            7: "kneeright_joint",              # leg_knee (sts3095)
            8: "footright_joint",              # leg_foot (sts3215)
            9: "gripperright_joint",           # leg_gripper (sts3215)
        }

        # Servo control parameters (like micro_bimo.ino)
        self.servo_speed = 3400      # Goal velocity (like WritePosEx speed param)
        self.servo_accel = 254       # Acceleration (like WritePosEx acceleration param)

        # ✅ SỬA: Servo limits CHỈ cho motor 4-9
        self.servo_limits = {
            4: {"min": 300, "max": 2432},
            5: {"min": 2240, "max": 3075},
            6: {"min": 1680, "max": 3307},
            7: {"min": 1635, "max": 3545},
            8: {"min": 1413, "max": 2532},     
            9: {"min": 2044, "max": 2050},       
        }

        # ✅ SỬA: Current positions CHỈ 6 servos (motor 4-9)
        self.current_positions = [0] * 6  # 6 slots
        self.target_positions = [0] * 6   # 6 slots

        # Servo feedback history
        self.feedback_history = deque(maxlen=100)

        # ✅ SỬA: State data CHỈ 6 servos (motor 4-9)
        self.state_data = {
            "imu": [0.0, 0.0, 0.0, 0.0],       # Quaternion [w, x, y, z]
            "distance": [0, 0, 0, 0],          # Distance sensors
            "servo_pos": [0] * 6,              # 6 leg servos
            "servo_speed": [0] * 6,
            "servo_load": [0] * 6,
            "servo_voltage": [0] * 6,
            "servo_current": [0] * 6,
            "servo_temp": [0] * 6,
        }

        # ZeroMQ context and socket
        self.context = zmq.Context()
        self.socket = None

        # Update loop control
        self.control_rate = 50  # Hz (20ms per update like micro_bimo.ino)
        self.last_update_time = time.time()
        self.update_thread = None
        self.imu_thread = None      # ✅ THÊM
        self.servo_thread = None    # ✅ THÊM

        # ✅ SỬA: Tách thành 3 lock - 1 cho read, 1 cho write, 1 cho IMU
        self.read_lock = threading.Lock()      # Chỉ cho servo feedback
        self.write_lock = threading.Lock()     # Chỉ cho servo command
        self.imu_lock = threading.Lock()       # ✅ THÊM: Cho IMU data
        
        logger.info(f"MCUServer initialized (6 leg servos 4-9) on port {zmq_port}")

    def init_zmq(self) -> bool:
        """Initialize ZeroMQ sockets - BOTH REQ/REP and PUSH/PULL"""
        try:
            # ✅ REQ/REP Socket (feedback queries)
            self.socket_rep = self.context.socket(zmq.REP)
            self.socket_rep.setsockopt(zmq.RCVTIMEO, 100)
            self.socket_rep.bind(f"tcp://*:{self.zmq_port}")  # 5555
            logger.info(f"✓ REQ/REP socket bound to port {self.zmq_port} (feedback)")
            
            # ⭐ THÊM: PUSH Socket (async control commands)
            self.socket_pull = self.context.socket(zmq.PULL)
            self.socket_pull.setsockopt(zmq.RCVTIMEO, 100)
            self.socket_pull.bind(f"tcp://*:{self.zmq_port + 100}")  # 5655
            logger.info(f"✓ PUSH/PULL socket bound to port {self.zmq_port + 100} (async commands)")
            
            # ⭐ THÊM: Poller để monitor cả 2 sockets
            self.poller = zmq.Poller()
            self.poller.register(self.socket_rep, zmq.POLLIN)
            self.poller.register(self.socket_pull, zmq.POLLIN)
            
            return True
        except Exception as e:
            logger.error(f"✗ Failed to initialize ZeroMQ: {e}")
            return False

    def init_robot(self) -> bool:
        """Initialize BipedalRobot with initial feedback read"""
        try:
            self.robot = BipedalRobot(self.config)
            self.robot.connect()
            self.robot.configure()
            
            logger.info("✓ Robot connected and configured")
            logger.info(f"✓ Available motors: {list(self.robot.bus.motors.keys())}")
            
            # ✅ THÊM: Initialize feedback immediately
            logger.info("Initializing servo feedback (reading actual positions)...")
            time.sleep(0.5)  # Wait for motors to respond
            self.update_servo_feedback()
            
            # Verify feedback is valid
            with self.read_lock:
                current_pos = self.state_data["servo_pos"].copy()
            
            logger.info(f"Initial positions read: {current_pos}")
            
            if all(p == 0 for p in current_pos):
                logger.warning("⚠️  Initial feedback all zeros!")
                logger.warning("   Retrying after 1 second...")
                time.sleep(1)
                self.update_servo_feedback()
                
                with self.read_lock:
                    current_pos = self.state_data["servo_pos"].copy()
                
                logger.info(f"After retry: {current_pos}")
                
                if all(p == 0 for p in current_pos):
                    logger.error("❌ Feedback still all zeros!")
                    logger.error("   Possible causes:")
                    logger.error("   - USB not connected properly")
                    logger.error("   - Servos not powered")
                    logger.error("   - Serial port mismatch")
                    logger.error("   - Motor names not matching config")
                    return False
            
            logger.info(f"✓ Initial positions: {current_pos}")
            logger.info("✅ Robot initialization complete")
            return True
            
        except Exception as e:
            logger.error(f"✗ Failed to initialize robot: {e}")
            return False

    def update_servo_feedback(self) -> None:
        """
        ✅ OPTIMIZED: Read ONLY servo positions
        """
        try:
            if not self.robot or not self.robot.is_connected:
                logger.debug("Robot not connected")
                return

            for servo_id in range(4, 10):
                motor_name = self.servo_map[servo_id]

                if motor_name not in self.robot.bus.motors:
                    continue

                try:
                    idx = servo_id - 4
                    
                    # ⭐ CHỈ đọc position
                    pos = self.robot.bus.read("Present_Position", motor_name, normalize=False)
                    
                    if pos is not None and pos > 0:
                        with self.read_lock:
                            self.state_data["servo_pos"][idx] = int(pos)
                    else:
                        with self.read_lock:
                            old_value = self.state_data["servo_pos"][idx]
                        logger.debug(f"⚠️  {motor_name}: read failed, keeping {old_value}")

                except Exception as e:
                    logger.error(f"Exception reading {motor_name}: {e}")

        except Exception as e:
            logger.error(f"Error in update_servo_feedback: {e}")

    def update_imu_data(self) -> None:
        """Update IMU data with calibration and store in state_data."""
        try:
            global filtered_ax, filtered_ay, filtered_az, filtered_gx, filtered_gy, filtered_gz
            # logger.debug("🔵 update_imu_data() called")
            if IMU.dataReady():
                # logger.debug("🟢 IMU.dataReady() = True")
                IMU.getAgmt()

                # Read raw data
                ax_raw = IMU.axRaw
                ay_raw = IMU.ayRaw
                az_raw = IMU.azRaw
                
                gx_raw = IMU.gxRaw
                gy_raw = IMU.gyRaw
                gz_raw = IMU.gzRaw
                
                # ✅ Calibrate and convert accelerometer (LSB → m/s²)
                ax_lsb = (ax_raw - ax_bias) / ax_scale
                ay_lsb = (ay_raw - ay_bias) / ay_scale
                az_lsb = (az_raw - az_bias) / az_scale
                
                ax = ax_lsb / SENSITIVITY * G
                ay = ay_lsb / SENSITIVITY * G
                az = az_lsb / SENSITIVITY * G
                
                # ✅ Calibrate and convert gyroscope (LSB → °/s → rad/s)
                gx_deg = (gx_raw - gx_bias) / GYRO_SENSITIVITY
                gy_deg = (gy_raw - gy_bias) / GYRO_SENSITIVITY
                gz_deg = (gz_raw - gz_bias) / GYRO_SENSITIVITY
                
                gx = gx_deg * math.pi / 180
                gy = gy_deg * math.pi / 180
                gz = gz_deg * math.pi / 180
                
                # ✅ Apply low-pass filter for smooth data
                filtered_ax = ACCEL_ALPHA * ax + (1 - ACCEL_ALPHA) * filtered_ax
                filtered_ay = ACCEL_ALPHA * ay + (1 - ACCEL_ALPHA) * filtered_ay
                filtered_az = ACCEL_ALPHA * az + (1 - ACCEL_ALPHA) * filtered_az
                
                filtered_gx = GYRO_ALPHA * gx + (1 - GYRO_ALPHA) * filtered_gx
                filtered_gy = GYRO_ALPHA * gy + (1 - GYRO_ALPHA) * filtered_gy
                filtered_gz = GYRO_ALPHA * gz + (1 - GYRO_ALPHA) * filtered_gz
                
                # ✅ Update Madgwick filter with filtered data
                q_new = madgwick.updateIMU(
                    q=madgwick.q0,
                    gyr=np.array([filtered_gx, filtered_gy, filtered_gz]),
                    acc=np.array([filtered_ax, filtered_ay, filtered_az])
                )
                madgwick.q0 = q_new
                
                # ✅ Store quaternion in state_data
                with self.imu_lock:
                    self.state_data["imu"] = list(q_new)  # [w, x, y, z]
                
                # logger.debug(
                #     f"IMU: ax={ax:+.4f}, ay={ay:+.4f}, az={az:+.4f} | "
                #     f"gx={gx_deg:+.2f}°/s, gy={gy_deg:+.2f}°/s, gz={gz_deg:+.2f}°/s | "
                #     f"q=[{q_new[0]:+.4f}, {q_new[1]:+.4f}, {q_new[2]:+.4f}, {q_new[3]:+.4f}]"
                # )
            # else:
                # logger.debug("🔴 IMU.dataReady() = False")
        except Exception as e:
            logger.error(f"Error updating IMU data: {e}")


    def update_distance_sensors(self) -> None:
        """Update distance sensor data (placeholder)."""
        # TODO: Integrate actual distance sensors
        # Example: self.state_data["distance"] = [front, back, right, left]
        pass

    def get_state_data_bytes(self) -> bytes:
        """Get state data as bytes - CHỈ 6 leg servos"""
        try:
            imu_bytes = struct.pack("<4f", *self.state_data["imu"])
            dist_bytes = struct.pack("<4H", *self.state_data["distance"])
            pos_bytes = struct.pack("<6H", *self.state_data["servo_pos"])
            speed_bytes = struct.pack("<6h", *self.state_data["servo_speed"])
            load_bytes = struct.pack("<6h", *self.state_data["servo_load"])
            voltage_bytes = struct.pack("<6H", *self.state_data["servo_voltage"])
            current_bytes = struct.pack("<6h", *self.state_data["servo_current"])
            temp_bytes = struct.pack("<6B", *self.state_data["servo_temp"])

            state_bytes = (
                imu_bytes + dist_bytes + pos_bytes + speed_bytes +
                load_bytes + voltage_bytes + current_bytes + temp_bytes
            )

            logger.debug(f"State data packed: {len(state_bytes)} bytes")
            return state_bytes

        except Exception as e:
            logger.error(f"Error packing state data: {e}")
            return b""

    def apply_new_positions(self, positions: List[int]) -> bool:
        """
        ✅ SỬA: Ghi vị trí với write_lock (ngắn hơn).
        Ưu tiên ghi (command) hơn đọc (feedback).
        """
        try:
            if not self.robot or not self.robot.is_connected:
                logger.warning("Robot not connected")
                return False

            if len(positions) != 6:
                logger.error(f"Expected 6 positions, got {len(positions)}")
                return False

            success_count = 0
            fail_count = 0

            # ✅ SỬA: Lock chỉ khi ghi
            with self.write_lock:
                for servo_id in range(4, 10):
                    motor_name = self.servo_map[servo_id]
                    pos_idx = servo_id - 4
                    target_pos = positions[pos_idx]

                    if motor_name not in self.robot.bus.motors:
                        logger.warning(f"Motor {motor_name} (ID {servo_id}) not found")
                        fail_count += 1
                        continue

                    limits = self.servo_limits[servo_id]
                    clamped_pos = max(limits["min"], min(limits["max"], target_pos))

                    try:
                        logger.debug(
                            f"Sending to {motor_name} (ID {servo_id}): pos={clamped_pos}, "
                            f"speed={self.servo_speed}, accel={self.servo_accel}"
                        )

                        self.robot.write_pos_ex(
                            motor_name=motor_name,
                            position=clamped_pos,
                            speed=self.servo_speed,
                            acceleration=self.servo_accel,
                            normalize=False
                        )
                        success_count += 1

                    except Exception as e:
                        logger.error(f"Failed to set {motor_name} (ID {servo_id}): {e}")
                        fail_count += 1

            self.target_positions = positions.copy()
            
            result = success_count > 0 and fail_count == 0
            logger.info(f"Applied positions (motor 4-9): {success_count} success, {fail_count} failed")
            return result

        except Exception as e:
            logger.error(f"Error in apply_new_positions: {e}")
            return False

    def process_request(self, msg: int) -> Optional[bytes]:
        """Process incoming request message."""
        try:
            if msg == 1:
                logger.debug("State data requested")
                return self.get_state_data_bytes()

            elif msg == 2:
                logger.debug("Alive status requested")
                response = struct.pack("<i", 0)
                return response

            elif msg == 3:
                logger.info("Calibration requested")
                return struct.pack("<i", 1)

            else:
                logger.warning(f"Unknown request: {msg}")
                return None

        except Exception as e:
            logger.error(f"Error processing request: {e}")
            return None

    def process_command(self, command: Dict) -> Dict:
        """Process incoming command"""
        try:
            cmd_type = command.get("type")

            if cmd_type == "move":
                positions = command.get("positions", [])
                success = self.apply_new_positions(positions)
                return {
                    "status": "success" if success else "error",
                    "current_positions": self.target_positions,
                }

            elif cmd_type == "feedback":
                # ✅ THÊM: Trả về gyro data
                with self.imu_lock:
                    imu_quat = self.state_data["imu"].copy()
                
                return {
                    "status": "success",
                    "quat": imu_quat,
                    "gyro": [filtered_gx, filtered_gy, filtered_gz],  # ✅ THÊM
                    "servo_pos": self.state_data["servo_pos"],
                    "servo_speed": self.state_data["servo_speed"],
                    "servo_load": self.state_data["servo_load"],
                    "servo_voltage": self.state_data["servo_voltage"],
                    "servo_current": self.state_data["servo_current"],
                    "servo_temp": self.state_data["servo_temp"],
                }

            elif cmd_type == "home":
                home_pos = [415, 2642, 2702, 2505, 2185, 2048]
                success = self.apply_new_positions(home_pos)
                return {
                    "status": "success" if success else "error",
                    "message": "Home position reached",
                }

            elif cmd_type == "stop":
                stop_pos = self.target_positions.copy()
                success = self.apply_new_positions(stop_pos)
                return {
                    "status": "success" if success else "error",
                    "message": "Servos (4-9) stopped",
                }

            elif cmd_type == "config":
                if "speed" in command:
                    self.servo_speed = command["speed"]
                if "acceleration" in command:
                    self.servo_accel = command["acceleration"]
                return {
                    "status": "success",
                    "message": f"Config updated: speed={self.servo_speed}, accel={self.servo_accel}",
                }

            else:
                return {
                    "status": "error",
                    "message": f"Unknown command type: {cmd_type}",
                }

        except Exception as e:
            logger.error(f"Error processing command: {e}")
            return {"status": "error", "message": str(e)}

    # ✅ THÊM: IMU loop riêng
    def imu_loop(self) -> None:
        """✅ IMU thread - 50Hz independent loop"""
        logger.info("IMU loop started (50Hz - independent)")
        
        while self.running:
            try:
                self.update_imu_data()
                time.sleep(0.02)  # 50Hz = 20ms
            except Exception as e:
                logger.error(f"Error in IMU loop: {e}")
                time.sleep(0.01)
        
        logger.info("IMU loop stopped")

    # ✅ THÊM: Servo loop riêng
    def servo_loop(self) -> None:
        """✅ Servo thread - 25Hz independent loop"""
        logger.info("Servo loop started (25Hz - independent)")
        
        while self.running:
            try:
                if self.read_lock.acquire(timeout=0.01):
                    try:
                        self.update_servo_feedback()
                    finally:
                        self.read_lock.release()
                else:
                    logger.debug("Read lock busy, skipping servo feedback")
                
                self.update_distance_sensors()
                time.sleep(0.04)  # 25Hz = 40ms
                
            except Exception as e:
                logger.error(f"Error in Servo loop: {e}")
                time.sleep(0.01)
        
        logger.info("Servo loop stopped")

    # ✅ THAY: update_loop() → chỉ deprecated placeholder
    def update_loop(self) -> None:
        """⚠️ KHÔNG DÙNG NỮA"""
        logger.info("⚠️  Main update loop (deprecated)")
        while self.running:
            time.sleep(0.1)
        logger.info("Main update loop stopped")

    # ✅ SỬA: run() để start 2 threads
    def run(self) -> None:
        """Main server loop - Monitor cả REQ/REP và PUSH/PULL"""
        self.running = True

        # ✅ START IMU thread (50Hz)
        self.imu_thread = threading.Thread(target=self.imu_loop, daemon=True)
        self.imu_thread.start()
        logger.info("✓ IMU loop thread started (50Hz)")

        # ✅ START Servo thread (25Hz)
        self.servo_thread = threading.Thread(target=self.servo_loop, daemon=True)
        self.servo_thread.start()
        logger.info("✓ Servo loop thread started (25Hz)")

        # ✅ WARMUP IMU
        logger.info("⏳ Warming up IMU (filter convergence)...")
        warmup_time = 5.0
        warmup_start = time.time()
        imu_samples = 0
        valid_count = 0
        
        while time.time() - warmup_start < warmup_time:
            time.sleep(0.02)
            imu_samples += 1
            
            with self.imu_lock:
                imu_data = self.state_data["imu"].copy()
            
            if imu_data != [0.0, 0.0, 0.0, 0.0]:
                valid_count += 1
                if valid_count % 50 == 0:
                    logger.info(f"  ✓ IMU valid: {valid_count} samples")
        
        logger.info(f"✅ Warmup complete: {imu_samples} samples, {valid_count} valid")
        logger.info("MCU Server started with separate IMU/Servo loops...")

        try:
            while self.running:
                # ⭐ THÊM: Use poller để monitor cả 2 sockets
                socks = dict(self.poller.poll(timeout=10))
                
                # ✅ Process REQ/REP (feedback queries)
                if self.socket_rep in socks:
                    try:
                        message = self.socket_rep.recv_json()
                        logger.debug(f"REQ: {message}")
                        response = self.process_command(message)
                        self.socket_rep.send_json(response)
                    except zmq.Again:
                        pass
                    except Exception as e:
                        logger.error(f"REQ/REP error: {e}")
                
                # ⭐ THÊM: Process PUSH/PULL (async commands)
                if self.socket_pull in socks:
                    try:
                        command = self.socket_pull.recv_json()
                        logger.debug(f"PUSH: {command}")
                        # ✅ SỬA: Chỉ process move commands (không cần response)
                        if command.get("type") == "move":
                            positions = command.get("positions", [])
                            self.apply_new_positions(positions)
                            logger.info(f"✓ Async move applied: {positions}")
                    except zmq.Again:
                        pass
                    except Exception as e:
                        logger.error(f"PUSH/PULL error: {e}")

        except KeyboardInterrupt:
            logger.info("Server interrupted by user")
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self) -> None:
        """Shutdown the server"""
        self.running = False

        # Wait for threads
        if self.imu_thread:
            self.imu_thread.join(timeout=2.0)
            logger.info("IMU thread stopped")

        if self.servo_thread:
            self.servo_thread.join(timeout=2.0)
            logger.info("Servo thread stopped")

        if self.update_thread:
            self.update_thread.join(timeout=2.0)

        # ⭐ THÊM: Close cả 2 sockets
        if self.socket_rep:
            self.socket_rep.close()
        if self.socket_pull:
            self.socket_pull.close()

        if self.robot:
            try:
                self.robot.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting robot: {e}")

        self.context.term()
        logger.info("MCU Server shutdown complete")


def main():
    """Start the MCU server."""
    import argparse

    parser = argparse.ArgumentParser(description="Bimo RIGHT Leg MCU Control Server")
    parser.add_argument("--port", type=int, default=5555, help="ZeroMQ port")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0", help="Servo serial port")
    parser.add_argument("--speed", type=int, default=3400, help="Default servo speed")
    parser.add_argument("--acceleration", type=int, default=254, help="Default servo acceleration")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")

    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        logging.getLogger().setLevel(logging.INFO)

    server = MCUServer(zmq_port=args.port, serial_port=args.serial_port)
    server.servo_speed = args.speed
    server.servo_accel = args.acceleration

    try:
        if not server.init_zmq():
            return
        
        if not server.init_robot():
            return

        # ✅ run() sẽ tự start 2 threads
        server.run()

    except Exception as e:
        logger.error(f"Failed to start server: {e}")
    finally:
        server.shutdown()

if __name__ == "__main__":
    main()