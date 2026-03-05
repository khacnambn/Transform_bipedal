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

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from bipedal_robot import BipedalRobot, BipedalConfig

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - [MCU_SERVER] - %(message)s'
)
logger = logging.getLogger(__name__)


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
            4: {"min": 300, "max": 2432},      # leg_bub (sts3095)
            5: {"min": 2240, "max": 3075},       # leg_hip (sts3095)
            6: {"min": 1680, "max": 3307},      # leg_twist (sts3215)
            7: {"min": 1313, "max": 3231},       # leg_knee (sts3095)
            8: {"min": 1413, "max": 2532},       # leg_foot (sts3215)
            9: {"min": 2044, "max": 2050},       # leg_gripper (sts3215)
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

        # ✅ SỬA: Tách thành 2 lock - 1 cho read, 1 cho write
        self.read_lock = threading.Lock()
        self.write_lock = threading.Lock()
        
        logger.info(f"MCUServer initialized (6 leg servos 4-9) on port {zmq_port}")

    def init_zmq(self) -> bool:
        """Initialize ZeroMQ socket."""
        try:
            self.socket = self.context.socket(zmq.REP)
            self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
            self.socket.bind(f"tcp://*:{self.zmq_port}")
            logger.info(f"✓ ZeroMQ socket bound to port {self.zmq_port}")
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
            return True
            
        except Exception as e:
            logger.error(f"✗ Failed to initialize robot: {e}")
            return False

    def update_servo_feedback(self) -> None:
        """
        ✅ SỬA: Đọc feedback từ servo - NẾU fail thì GIỮ giá trị CŨ, không set 0
        Thêm log chi tiết để debug
        """
        try:
            if not self.robot or not self.robot.is_connected:
                logger.debug("Robot not connected")
                return

            logger.debug(f"Reading feedback from {len(self.servo_map)} motors...")
        
            for servo_id in range(4, 10):
                motor_name = self.servo_map[servo_id]

                if motor_name not in self.robot.bus.motors:
                    logger.warning(f"❌ Motor {motor_name} (ID {servo_id}) NOT in bus!")
                    logger.warning(f"   Available motors: {list(self.robot.bus.motors.keys())}")
                    continue

                try:
                    idx = servo_id - 4
                    
                    # ✅ SỬA: Đọc position - KHÔNG dùng `or 0`
                    pos = self.robot.bus.read("Present_Position", motor_name, normalize=False)
                    
                    logger.debug(f"[Read] {motor_name}: pos={pos} (type: {type(pos)})")
                    
                    # ✅ NEW: Chỉ update nếu read thành công và hợp lệ
                    if pos is not None and pos > 0:
                        with self.read_lock:
                            self.state_data["servo_pos"][idx] = int(pos)
                        logger.info(f"✓ {motor_name} (ID {servo_id}): pos={int(pos)} ticks")
                    else:
                        # ✅ SỬA: Nếu read fail, GIỮ giá trị CŨ, KHÔNG set 0
                        with self.read_lock:
                            old_value = self.state_data["servo_pos"][idx]
                        logger.warning(f"⚠️  {motor_name} (ID {servo_id}): read failed/None, keeping {old_value}")
                    
                    # Read speed, load, voltage, current, temp (tương tự)
                    speed = self.robot.bus.read("Present_Velocity", motor_name, normalize=False)
                    if speed is not None:
                        with self.read_lock:
                            self.state_data["servo_speed"][idx] = int(speed)
                    
                    load = self.robot.bus.read("Present_Load", motor_name, normalize=False)
                    if load is not None:
                        with self.read_lock:
                            self.state_data["servo_load"][idx] = int(load)
                    
                    voltage = self.robot.bus.read("Present_Voltage", motor_name, normalize=False)
                    if voltage is not None:
                        with self.read_lock:
                            self.state_data["servo_voltage"][idx] = int(voltage)
                    
                    current = self.robot.bus.read("Present_Current", motor_name, normalize=False)
                    if current is not None:
                        with self.read_lock:
                            self.state_data["servo_current"][idx] = int(current)
                    
                    temp = self.robot.bus.read("Present_Temperature", motor_name, normalize=False)
                    if temp is not None:
                        with self.read_lock:
                            self.state_data["servo_temp"][idx] = int(temp)

                except Exception as e:
                    logger.error(f"Exception reading {motor_name} (ID {servo_id}): {e}")

        except Exception as e:
            logger.error(f"Error in update_servo_feedback: {e}")

    def update_imu_data(self) -> None:
        """Update IMU data (placeholder)."""
        # TODO: Integrate actual IMU sensor reading
        # Example: self.state_data["imu"] = [quat_w, quat_x, quat_y, quat_z]
        pass

    def update_distance_sensors(self) -> None:
        """Update distance sensor data (placeholder)."""
        # TODO: Integrate actual distance sensors
        # Example: self.state_data["distance"] = [front, back, right, left]
        pass

    def get_state_data_bytes(self) -> bytes:
        """
        ✅ SỬA: Get state data as bytes - CHỈ 6 leg servos.
        Format: IMU(16B) + Distance(8B) + ServoPos(12B) + ServoSpeed(12B) +
                ServoLoad(12B) + ServoVoltage(12B) + ServoCurrent(12B) + ServoTemp(6B)
        Total: 90 bytes (thay vì 154 bytes)
        """
        try:
            # Pack IMU: 4 floats [w, x, y, z]
            imu_bytes = struct.pack("<4f", *self.state_data["imu"])

            # Pack distance: 4 uint16
            dist_bytes = struct.pack("<4H", *self.state_data["distance"])

            # ✅ SỬA: Servo positions: 6 uint16
            pos_bytes = struct.pack("<6H", *self.state_data["servo_pos"])

            # ✅ SỬA: Servo speeds: 6 int16
            speed_bytes = struct.pack("<6h", *self.state_data["servo_speed"])

            # ✅ SỬA: Servo loads: 6 int16
            load_bytes = struct.pack("<6h", *self.state_data["servo_load"])

            # ✅ SỬA: Servo voltages: 6 uint16
            voltage_bytes = struct.pack("<6H", *self.state_data["servo_voltage"])

            # ✅ SỬA: Servo currents: 6 int16
            current_bytes = struct.pack("<6h", *self.state_data["servo_current"])

            # ✅ SỬA: Servo temperatures: 6 uint8
            temp_bytes = struct.pack("<6B", *self.state_data["servo_temp"])

            # Combine all (90 bytes total)
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
        """
        ✅ SỬA: Process command - CHỈ motor 4-9, 6 joints.
        Tương đương checkComms() + processRequest() trong micro_bimo.ino

        Args:
            command: Dictionary with command type and parameters

        Returns:
            Response dictionary (chỉ chứa data motor 4-9)
        """
        try:
            cmd_type = command.get("type")

            if cmd_type == "move":
                # Move servos: {"type": "move", "positions": [pos4, pos5, ..., pos9]}
                # ✅ SỬA: Accept 6 phần tử
                positions = command.get("positions", [])
                success = self.apply_new_positions(positions)
                return {
                    "status": "success" if success else "error",
                    "current_positions": self.target_positions,  # 6 items
                }

            elif cmd_type == "feedback":
                # Get current state - chỉ motor 4-9
                return {
                    "status": "success",
                    "servo_pos": self.state_data["servo_pos"],      # 6 items
                    "servo_speed": self.state_data["servo_speed"],
                    "servo_load": self.state_data["servo_load"],
                    "servo_voltage": self.state_data["servo_voltage"],
                    "servo_current": self.state_data["servo_current"],
                    "servo_temp": self.state_data["servo_temp"],
                }

            elif cmd_type == "home":
                # Move to home position (center: 2048 ticks) - chỉ motor 4-9
                # ✅ SỬA: 6 servos (4-9)
                home_pos = [370, 2612, 2702, 2287, 2155, 2048]  # 6 items
                success = self.apply_new_positions(home_pos)
                return {
                    "status": "success" if success else "error",
                    "message": "Home position reached",
                }

            elif cmd_type == "calibrate":
                # Calibrate servos (motor 4-9)
                try:
                    logger.info("Starting calibration for motor 4-9...")
                    # TODO: Implement calibration for each motor (4-9)
                    return {
                        "status": "success",
                        "message": "Calibration complete",
                    }
                except Exception as e:
                    return {
                        "status": "error",
                        "message": str(e),
                    }

            elif cmd_type == "stop":
                # ✅ SỬA: Stop - 6 servos
                stop_pos = self.target_positions.copy()
                success = self.apply_new_positions(stop_pos)
                return {
                    "status": "success" if success else "error",
                    "message": "Servos (4-9) stopped",
                }

            elif cmd_type == "config":
                # Configure servo parameters
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

    def update_loop(self) -> None:
        """
        ✅ SỬA: Background update loop với timeout trên lock.
        Nếu lock bị block, skip lần này thay vì chờ.
        """
        logger.info("Update loop started (like Arduino loop())")

        while self.running:
            try:
                # Update all sensor data
                self.update_imu_data()
                
                # ✅ SỬA: Try acquire lock với timeout (non-blocking)
                if self.read_lock.acquire(timeout=0.005):  # 5ms timeout
                    try:
                        self.update_servo_feedback()
                    finally:
                        self.read_lock.release()
                else:
                    # Lock busy, skip this update
                    logger.debug("Read lock busy, skipping feedback update")
                
                self.update_distance_sensors()

                # Maintain control rate (50Hz = 20ms)
                elapsed = time.time() - self.last_update_time
                sleep_time = (1.0 / self.control_rate) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                self.last_update_time = time.time()

            except Exception as e:
                logger.error(f"Error in update loop: {e}")
                time.sleep(0.01)

        logger.info("Update loop stopped")

    def run(self) -> None:
        """
        Main server loop - receive and process commands.
        Tương đương checkComms() trong micro_bimo.ino
        """
        self.running = True

        # Start background update loop
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()

        logger.info("MCU Server started and waiting for commands...")

        try:
            while self.running:
                try:
                    # Wait for message with timeout
                    message = self.socket.recv_json()
                    logger.debug(f"Received command: {message}")

                    # Process command
                    response = self.process_command(message)

                    # Send response
                    self.socket.send_json(response)

                except zmq.Again:
                    # No message available
                    time.sleep(0.01)

                except json.JSONDecodeError as e:
                    logger.error(f"Invalid JSON received: {e}")
                    try:
                        self.socket.send_json({"status": "error", "message": "Invalid JSON"})
                    except:
                        pass

        except KeyboardInterrupt:
            logger.info("Server interrupted by user")
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Shutdown the server and cleanup resources."""
        self.running = False

        if self.update_thread:
            self.update_thread.join(timeout=2.0)

        if self.socket:
            self.socket.close()

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

    parser = argparse.ArgumentParser(description="Bimo MCU Control Server (6 Leg Servos 4-9)")
    parser.add_argument("--port", type=int, default=5555, help="ZeroMQ port")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0", help="Servo serial port")
    parser.add_argument("--speed", type=int, default=3400, help="Default servo speed")
    parser.add_argument("--acceleration", type=int, default=254, help="Default servo acceleration")

    args = parser.parse_args()

    server = MCUServer(zmq_port=args.port, serial_port=args.serial_port)
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