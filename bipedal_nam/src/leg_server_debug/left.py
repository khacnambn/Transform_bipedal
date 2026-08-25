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

# CHÂN TRÁI LÀ LEG SERVER 2
# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from bipedal_robot.bipedal_left import BipedalRobot, BipedalConfig

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - [MCU_SERVER_LEFT] - %(message)s"
)
logger = logging.getLogger(__name__)

# ==============================
# Load calibration
# ==============================
with open("/home/mobile2/Transform_bipedal/bipedal_nam/src/leg_server/calibfull.json", "r") as f:
    calib = json.load(f)

# calib từ accel
accel_SM = np.array(calib["accel"]["SM"])
accel_bias = np.array(calib["accel"]["bias"])

# calib từ gyro
gx_bias = calib["gyro"]["gx_bias"]
gy_bias = calib["gyro"]["gy_bias"]
gz_bias = calib["gyro"]["gz_bias"]
GYRO_SENSITIVITY = calib["gyro"]["gyro_sensitivity"]

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


class MCUServerLeft:
    """MCU control server for LEFT leg on Raspberry Pi 5."""

    def __init__(self, zmq_port: int = 5556, serial_port: str = "/dev/ttyACM0"):
        """
        Initialize LEFT leg server - motor 4-9 (LEFT leg only).

        Args:
            zmq_port: ZeroMQ socket port
            serial_port: Serial port for servo communication
        """
        self.zmq_port = zmq_port
        self.serial_port = serial_port
        self.running = False

        # Robot instance (BipedalRobot from bipedal_left.py)
        self.config = BipedalConfig(port=serial_port, baudrate=1_000_000)
        self.robot = None

        # ✅ Motor mapping - LEFT leg (motor 4-9)
        self.servo_map = {
            4: "bubleft_joint",  # LEFT leg_bub (sts3095)
            5: "hipleft_joint",  # LEFT leg_hip (sts3095)
            6: "twistleft_joint",  # LEFT leg_twist (sts3215)
            7: "kneeleft_joint",  # LEFT leg_knee (sts3095)
            8: "footleft_joint",  # LEFT leg_foot (sts3215)
            9: "gripperleft_joint",  # LEFT leg_gripper (sts3215)
        }

        # Servo control parameters
        self.servo_speed = 3400  # Goal velocity
        self.servo_accel = 254  # Acceleration

        # ✅ Servo limits for LEFT leg
        self.servo_limits = {
            4: {"min": 1311, "max": 3665},
            5: {"min": 1545, "max": 2365},
            6: {"min": 1040, "max": 1050},
            7: {"min": 1313, "max": 3231},
            8: {"min": 1111, "max": 2363},
            9: {"min": 1380, "max": 1398},
        }

        # Current positions (6 servos: motor 4-9)
        self.current_positions = [0] * 6
        self.target_positions = [0] * 6

        # Servo feedback history
        self.feedback_history = deque(maxlen=100)

        # State data (6 servos)
        self.state_data = {
            "imu": [0.0, 0.0, 0.0, 0.0],
            "distance": [0, 0, 0, 0],
            "servo_pos": [0] * 6,
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
        self.control_rate = 50  # Hz
        self.last_update_time = time.time()
        self.update_thread = None
        self.imu_thread = None  # ✅ THÊM
        self.servo_thread = None  # ✅ THÊM

        # Thread locks
        self.read_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.imu_lock = threading.Lock()

        logger.info(f"MCUServerLeft initialized (6 LEFT leg servos 4-9) on port {zmq_port}")

    def init_zmq(self) -> bool:
        """Initialize ZeroMQ sockets - BOTH REQ/REP and PUSH/PULL"""
        try:
            self.socket_rep = self.context.socket(zmq.REP)
            self.socket_rep.setsockopt(zmq.RCVTIMEO, 100)
            self.socket_rep.bind(f"tcp://*:{self.zmq_port}")  # 5555
            logger.info(f"✓ REQ/REP socket bound to port {self.zmq_port} (feedback)")

            self.socket_pull = self.context.socket(zmq.PULL)
            self.socket_pull.setsockopt(zmq.RCVTIMEO, 100)
            self.socket_pull.bind(f"tcp://*:{self.zmq_port + 100}")  # 5655
            logger.info(f"✓ PUSH/PULL socket bound to port {self.zmq_port + 100} (async commands)")

            self.poller = zmq.Poller()
            self.poller.register(self.socket_rep, zmq.POLLIN)
            self.poller.register(self.socket_pull, zmq.POLLIN)

            return True
        except Exception as e:
            logger.error(f"✗ Failed to initialize ZeroMQ: {e}")
            return False

    def init_robot(self) -> bool:
        """Initialize BipedalRobot (LEFT leg) with initial feedback read"""
        try:
            self.robot = BipedalRobot(self.config)
            self.robot.connect()
            self.robot.configure()

            logger.info("✓ Robot (LEFT leg) connected and configured")
            logger.info(f"✓ Available motors: {list(self.robot.bus.motors.keys())}")

            # Initialize feedback
            logger.info("Initializing servo feedback (reading actual positions)...")
            time.sleep(0.5)
            self.update_servo_feedback()

            # Verify feedback
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
                    return False

            logger.info(f"✓ Initial positions: {current_pos}")
            logger.info("✅ Robot initialization complete")
            return True

        except Exception as e:
            logger.error(f"✗ Failed to initialize robot: {e}")
            import traceback

            traceback.print_exc()
            return False

    def update_servo_feedback(self) -> None:
        """
        ✅ OPTIMIZED: Read ONLY servo positions (no speed/load/voltage/current/temp)
        This is 5x faster and enough for policy control!
        """
        try:
            if not self.robot or not self.robot.is_connected:
                logger.debug("Robot not connected")
                return

            for servo_id in range(4, 10):
                motor_name = self.servo_map[servo_id]

                if motor_name not in self.robot.bus.motors:
                    logger.warning(f"❌ Motor {motor_name} (ID {servo_id}) NOT in bus!")
                    continue

                try:
                    idx = servo_id - 4

                    # ⭐ CHỈ đọc position (1 read per motor ~20ms)
                    pos = self.robot.bus.read("Present_Position", motor_name, normalize=False)

                    if pos is not None and pos > 0:
                        with self.read_lock:
                            self.state_data["servo_pos"][idx] = int(pos)
                        logger.debug(f"✓ {motor_name}: pos={int(pos)}")
                    else:
                        with self.read_lock:
                            old_value = self.state_data["servo_pos"][idx]
                        logger.warning(f"⚠️  {motor_name}: read failed, keeping {old_value}")

                    # ⭐ BỎ: speed, load, voltage, current, temp (không cần cho policy)
                    # (Chỉ enable khi debug hoặc monitor robot health)

                except Exception as e:
                    logger.error(f"Exception reading {motor_name}: {e}")

        except Exception as e:
            logger.error(f"Error in update_servo_feedback: {e}")

    def update_imu_data(self) -> None:
        """Update IMU data with calibration and store in state_data."""
        try:
            global filtered_ax, filtered_ay, filtered_az, filtered_gx, filtered_gy, filtered_gz

            if IMU.dataReady():
                IMU.getAgmt()

                # Read raw data
                ax_raw = IMU.axRaw
                ay_raw = IMU.ayRaw
                az_raw = IMU.azRaw

                gx_raw = IMU.gxRaw
                gy_raw = IMU.gyRaw
                gz_raw = IMU.gzRaw

                #  Calibrate and convert accelerometer (LSB → m/s²)
                # đưa giá trị raw vào mảng numpy
                raw_accel = np.array([ax_raw, ay_raw, az_raw])
                calib_accel = accel_SM @ raw_accel - accel_bias  # giá trị calib = raw - offset

                ax = calib_accel[0]
                ay = calib_accel[1]
                az = calib_accel[2]

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
                    acc=np.array([filtered_ax, filtered_ay, filtered_az]),
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

        except Exception as e:
            logger.error(f"Error updating IMU data: {e}")

    def update_distance_sensors(self) -> None:
        """Update distance sensors (placeholder)."""
        pass

    def get_state_data_bytes(self) -> bytes:
        """Get state data as bytes (90 bytes total)"""
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
                imu_bytes
                + dist_bytes
                + pos_bytes
                + speed_bytes
                + load_bytes
                + voltage_bytes
                + current_bytes
                + temp_bytes
            )

            logger.debug(f"State data packed: {len(state_bytes)} bytes")
            return state_bytes

        except Exception as e:
            logger.error(f"Error packing state data: {e}")
            return b""

    def apply_new_positions(self, positions: List[int]) -> bool:
        """Apply new positions to LEFT leg servos"""
        try:
            if not self.robot or not self.robot.is_connected:
                logger.warning("Robot not connected")
                return False

            if len(positions) != 6:
                logger.error(f"Expected 6 positions, got {len(positions)}")
                return False

            success_count = 0
            fail_count = 0

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
                            normalize=False,
                        )
                        success_count += 1

                    except Exception as e:
                        logger.error(f"Failed to set {motor_name} (ID {servo_id}): {e}")
                        fail_count += 1

            self.target_positions = positions.copy()

            result = success_count > 0 and fail_count == 0
            logger.info(
                f"Applied positions (LEFT leg 4-9): {success_count} success, {fail_count} failed"
            )
            return result

        except Exception as e:
            logger.error(f"Error in apply_new_positions: {e}")
            return False

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
                home_pos = [3366, 1958, 1042, 2317, 1584, 1390]
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

    def update_loop(self) -> None:
        """⚠️ KHÔNG DÙNG NỮA"""
        logger.info("⚠️  Main update loop (deprecated)")
        while self.running:
            time.sleep(0.1)
        logger.info("Main update loop stopped")

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
    """Start the LEFT leg MCU server."""
    import argparse

    parser = argparse.ArgumentParser(description="Bimo LEFT Leg MCU Control Server (6 Servos 4-9)")
    parser.add_argument("--port", type=int, default=5556, help="ZeroMQ port")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0", help="Servo serial port")
    parser.add_argument("--speed", type=int, default=3400, help="Default servo speed")
    parser.add_argument("--acceleration", type=int, default=254, help="Default servo acceleration")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")  # ✅ Thêm

    args = parser.parse_args()

    # ✅ Set log level based on --debug flag
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        logging.getLogger().setLevel(logging.INFO)

    server = MCUServerLeft(zmq_port=args.port, serial_port=args.serial_port)
    server.servo_speed = args.speed
    server.servo_accel = args.acceleration

    try:
        if not server.init_zmq():
            return

        if not server.init_robot():
            return

        server.run()

    except Exception as e:
        logger.error(f"Failed to start server: {e}")
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
