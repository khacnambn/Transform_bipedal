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

from bipedal_robot.bipedal_left import BipedalRobot, BipedalConfig

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - [MCU_SERVER_LEFT] - %(message)s'
)
logger = logging.getLogger(__name__)


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
            4: "bubleft_joint",               # LEFT leg_bub (sts3095)
            5: "hipleft_joint",               # LEFT leg_hip (sts3095)
            6: "twistleft_joint",             # LEFT leg_twist (sts3215)
            7: "kneeleft_joint",              # LEFT leg_knee (sts3095)
            8: "footleft_joint",              # LEFT leg_foot (sts3215)
            9: "gripperleft_joint",           # LEFT leg_gripper (sts3215)
        }

        # Servo control parameters
        self.servo_speed = 3400      # Goal velocity
        self.servo_accel = 254       # Acceleration

        # ✅ Servo limits for LEFT leg
        self.servo_limits = {
            4: {"min": 1311, "max": 3665},
            5: {"min": 1545, "max": 2365},
            6: {"min": 2560, "max": 2570},
            7: {"min": 1635, "max": 3545},
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

        # Thread locks
        self.read_lock = threading.Lock()
        self.write_lock = threading.Lock()
        
        logger.info(f"MCUServerLeft initialized (6 LEFT leg servos 4-9) on port {zmq_port}")

    def init_zmq(self) -> bool:
        """Initialize ZeroMQ socket."""
        try:
            self.socket = self.context.socket(zmq.REP)
            self.socket.setsockopt(zmq.RCVTIMEO, 100)
            self.socket.bind(f"tcp://*:{self.zmq_port}")
            logger.info(f"✓ ZeroMQ socket bound to port {self.zmq_port}")
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
            return True
            
        except Exception as e:
            logger.error(f"✗ Failed to initialize robot: {e}")
            import traceback
            traceback.print_exc()
            return False

    def update_servo_feedback(self) -> None:
        """Read servo feedback for LEFT leg (motor 4-9)"""
        try:
            if not self.robot or not self.robot.is_connected:
                logger.debug("Robot not connected")
                return

            logger.debug(f"Reading feedback from {len(self.servo_map)} motors...")
        
            for servo_id in range(4, 10):
                motor_name = self.servo_map[servo_id]

                if motor_name not in self.robot.bus.motors:
                    logger.warning(f"❌ Motor {motor_name} (ID {servo_id}) NOT in bus!")
                    continue

                try:
                    idx = servo_id - 4
                    
                    # Read position
                    pos = self.robot.bus.read("Present_Position", motor_name, normalize=False)
                    
                    logger.debug(f"[Read] {motor_name}: pos={pos}")
                    
                    if pos is not None and pos > 0:
                        with self.read_lock:
                            self.state_data["servo_pos"][idx] = int(pos)
                        logger.info(f"✓ {motor_name} (ID {servo_id}): pos={int(pos)} ticks")
                    else:
                        with self.read_lock:
                            old_value = self.state_data["servo_pos"][idx]
                        logger.warning(f"⚠️  {motor_name} (ID {servo_id}): read failed, keeping {old_value}")
                    
                    # Read other sensor data
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
        pass

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
                imu_bytes + dist_bytes + pos_bytes + speed_bytes +
                load_bytes + voltage_bytes + current_bytes + temp_bytes
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
                            normalize=False
                        )
                        success_count += 1

                    except Exception as e:
                        logger.error(f"Failed to set {motor_name} (ID {servo_id}): {e}")
                        fail_count += 1

            self.target_positions = positions.copy()
            
            result = success_count > 0 and fail_count == 0
            logger.info(f"Applied positions (LEFT leg 4-9): {success_count} success, {fail_count} failed")
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
                return {
                    "status": "success",
                    "servo_pos": self.state_data["servo_pos"],
                    "servo_speed": self.state_data["servo_speed"],
                    "servo_load": self.state_data["servo_load"],
                    "servo_voltage": self.state_data["servo_voltage"],
                    "servo_current": self.state_data["servo_current"],
                    "servo_temp": self.state_data["servo_temp"],
                }

            elif cmd_type == "home":
                home_pos = [3536, 1988, 2564, 2535, 1614, 1390]
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

    def update_loop(self) -> None:
        """Background update loop"""
        logger.info("Update loop started (like Arduino loop())")

        while self.running:
            try:
                self.update_imu_data()
                
                if self.read_lock.acquire(timeout=0.005):
                    try:
                        self.update_servo_feedback()
                    finally:
                        self.read_lock.release()
                else:
                    logger.debug("Read lock busy, skipping feedback update")
                
                self.update_distance_sensors()

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
        """Main server loop"""
        self.running = True

        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()

        logger.info("MCU Server (LEFT) started and waiting for commands...")

        try:
            while self.running:
                try:
                    message = self.socket.recv_json()
                    logger.debug(f"Received command: {message}")

                    response = self.process_command(message)

                    self.socket.send_json(response)

                except zmq.Again:
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
        """Shutdown the server"""
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
        logger.info("MCU Server (LEFT) shutdown complete")


def main():
    """Start the LEFT leg MCU server."""
    import argparse

    parser = argparse.ArgumentParser(description="Bimo LEFT Leg MCU Control Server (6 Servos 4-9)")
    parser.add_argument("--port", type=int, default=5556, help="ZeroMQ port")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0", help="Servo serial port")
    parser.add_argument("--speed", type=int, default=3400, help="Default servo speed")
    parser.add_argument("--acceleration", type=int, default=254, help="Default servo acceleration")

    args = parser.parse_args()

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