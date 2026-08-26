import zmq
import json
import time


class SimpleWalkClient:
    """Simple walking gait controller - chỉ 3 joints: hip, knee, foot"""

    def __init__(self, server_ip: str = "localhost", server_port: int = 5555):
        self.server_ip = server_ip
        self.server_port = server_port

        # ZeroMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{server_ip}:{server_port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 5000)

        # ✅ Home position (từ servo calibration)
        # [bub, hip, twist, knee, foot, gripper]
        self.home_pos = [370, 2612, 2702, 2287, 2155, 2048]

        # ✅ Servo config - ticks 0-4096 (NOT 4095!)
        self.servo_config = {
            1: {"name": "leg_hip", "home": 2612, "min": 2240, "max": 3075},
            3: {"name": "leg_knee", "home": 2287, "min": 0, "max": 4096},
            4: {"name": "leg_foot", "home": 2155, "min": 0, "max": 4096},
        }

        # ✅ Safe movement zones (tránh va chạm)
        self.safe_zones = {
            1: {"min_deg": -20, "max_deg": 20},  # leg_hip: -20 to +20°
            3: {"min_deg": -30, "max_deg": 30},  # leg_knee: -30 to +30°
            4: {"min_deg": -15, "max_deg": 15},  # leg_foot: -15 to +15°
        }

        print("✓ Simple Walk Client initialized")
        print("✓ Servo range: 0-4096 ticks = 0-360°")
        print("✓ Controlling: Hip, Knee, Foot (3 joints)")
        print("✓ Fixed: Bub, Twist, Gripper")

    def degree_to_ticks(self, servo_id: int, degrees: float) -> int:
        """
        ✅ Convert degrees to ticks

        Công thức:
          ticks = (degrees / 360) * 4096

        Args:
            servo_id: 1, 3, 4
            degrees: -180 to +180 (absolute degrees, not relative!)

        Returns:
            ticks value (0-4096)
        """
        safe = self.safe_zones[servo_id]

        # Clamp to safe zone
        degrees = max(safe["min_deg"], min(safe["max_deg"], degrees))

        # ✅ ĐÚNG: Convert degrees → ticks
        # ticks = (degrees / 360) * 4096
        ticks = int((degrees / 360.0) * 4096)

        # Clamp to hardware limits
        ticks = max(0, min(4096, ticks))

        return ticks

    def ticks_to_degree(self, servo_id: int, ticks: int) -> float:
        """
        ✅ Convert ticks to degrees

        Công thức:
          degrees = (ticks / 4096) * 360

        Args:
            servo_id: 1, 3, 4
            ticks: 0-4096 (absolute position)

        Returns:
            angle in degrees (absolute)
        """
        # ✅ ĐÚNG: Convert ticks → degrees
        # degrees = (ticks / 4096) * 360
        degrees = (ticks / 4096.0) * 360.0

        return round(degrees, 2)

    def print_servo_angles(self, positions: list):
        """In ra góc hiện tại của 3 servos"""
        print("\n📐 Current Angles (Absolute):")
        for servo_id in [1, 3, 4]:
            ticks = positions[servo_id]
            degrees = self.ticks_to_degree(servo_id, ticks)
            name = self.servo_config[servo_id]["name"]
            safe = self.safe_zones[servo_id]

            if safe["min_deg"] <= degrees <= safe["max_deg"]:
                status = "✓"
            else:
                status = "⚠️"

            print(f"  {status} {name:12} {degrees:7.2f}° ({ticks:4d} ticks)")

    def send_command(self, positions: list) -> bool:
        """Gửi lệnh move tới server"""
        try:
            self.socket.send_json({"type": "move", "positions": positions})
            response = self.socket.recv_json()
            return response.get("status") == "success"
        except zmq.error.Again:
            print("✗ Timeout: Server không response")
            return False
        except Exception as e:
            print(f"✗ Error: {e}")
            return False

    def get_feedback(self) -> list:
        """Đọc vị trí hiện tại từ servo"""
        try:
            self.socket.send_json({"type": "feedback"})
            response = self.socket.recv_json()
            if response.get("status") == "success":
                return response.get("servo_pos", self.home_pos)
            return self.home_pos
        except Exception as e:
            print(f"✗ Error reading feedback: {e}")
            return self.home_pos

    def interpolate(self, start: list, end: list, steps: int) -> list:
        """Nội suy tuyến tính từ start → end"""
        trajectory = []
        for step in range(steps + 1):
            t = step / steps
            pos = [int(start[i] + (end[i] - start[i]) * t) for i in range(6)]
            trajectory.append(pos)
        return trajectory

    def walk_forward(
        self,
        num_steps: int = 3,
        hip_deg: float = 100,
        knee_deg: float = 150,
        foot_deg: float = 90,
    ):
        """
        ✅ Simple walking gait - điều khiển 3 joints (hip, knee, foot)

        Args:
            num_steps: số bước
            hip_deg: vị trí hip (degrees absolute 0-360)
            knee_deg: vị trí knee (degrees absolute 0-360)
            foot_deg: vị trí foot (degrees absolute 0-360)
        """
        print("\n" + "=" * 60)
        print(f"🚶 Walking Forward - {num_steps} steps")
        print(f"   Hip:  {hip_deg:6.1f}°")
        print(f"   Knee: {knee_deg:6.1f}°")
        print(f"   Foot: {foot_deg:6.1f}°")
        print("=" * 60)

        for step_num in range(num_steps):
            print(f"\n📍 Step {step_num + 1}/{num_steps}")

            # ========== PHASE 1: LIFT ==========
            print("  [1/3] Lifting...")
            lift_pos = self.home_pos.copy()
            lift_pos[1] = self.degree_to_ticks(1, hip_deg)  # Hip
            lift_pos[3] = self.degree_to_ticks(3, knee_deg - 30)  # Knee nâng lên
            lift_pos[4] = self.degree_to_ticks(4, foot_deg)  # Foot

            trajectory = self.interpolate(self.home_pos, lift_pos, steps=8)
            for pos in trajectory:
                if not self.send_command(pos):
                    print("✗ Command failed")
                    return
                time.sleep(0.05)

            self.print_servo_angles(lift_pos)

            # ========== PHASE 2: FORWARD ==========
            print("  [2/3] Moving forward...")
            forward_pos = self.home_pos.copy()
            forward_pos[1] = self.degree_to_ticks(1, hip_deg + 30)  # Hip move forward
            forward_pos[3] = self.degree_to_ticks(3, knee_deg)  # Knee duỗi ra
            forward_pos[4] = self.degree_to_ticks(4, foot_deg - 20)  # Foot hướng xuống

            trajectory = self.interpolate(lift_pos, forward_pos, steps=12)
            for pos in trajectory:
                if not self.send_command(pos):
                    print("✗ Command failed")
                    return
                time.sleep(0.05)

            self.print_servo_angles(forward_pos)

            # ========== PHASE 3: DOWN ==========
            print("  [3/3] Stepping down...")
            down_pos = self.home_pos.copy()
            down_pos[1] = self.degree_to_ticks(1, 90)  # Hip neutral (90°)
            down_pos[3] = self.degree_to_ticks(3, 120)  # Knee at 120°
            down_pos[4] = self.degree_to_ticks(4, 90)  # Foot neutral (90°)

            trajectory = self.interpolate(forward_pos, down_pos, steps=12)
            for pos in trajectory:
                if not self.send_command(pos):
                    print("✗ Command failed")
                    return
                time.sleep(0.05)

            print(f"  ✓ Step {step_num + 1} complete")

        print("\n" + "=" * 60)
        print("✅ Walk complete!")
        print("=" * 60)

    def test_angles(self):
        """Test mode: Kiểm tra từng góc một"""
        print("\n" + "=" * 60)
        print("🧪 Angle Test Mode - Hip (1), Knee (3), Foot (4)")
        print("Conversion: degrees = (ticks / 4096) * 360")
        print("=" * 60)

        while True:
            try:
                user_input = (
                    input("\n> Format: joint_id angle (e.g., 1 100) or 'back': ")
                    .strip()
                    .lower()
                )

                if user_input == "back":
                    break

                parts = user_input.split()
                if len(parts) != 2:
                    print("❌ Invalid format. Example: 1 100")
                    continue

                servo_id = int(parts[0])
                degrees = float(parts[1])

                if servo_id not in [1, 3, 4]:
                    print("❌ Joint ID must be 1 (hip), 3 (knee), or 4 (foot)")
                    continue

                config = self.servo_config[servo_id]
                safe = self.safe_zones[servo_id]

                print(f"\n  Joint: {config['name']} (ID {servo_id})")
                print(
                    f"  Home: {config['home']} ticks = {self.ticks_to_degree(servo_id, config['home']):.2f}°"
                )
                print(f"  Safe zone: {safe['min_deg']}° to {safe['max_deg']}°")
                print(f"  Request: {degrees}°")

                # Convert and clamp
                clamped = max(safe["min_deg"], min(safe["max_deg"], degrees))
                if clamped != degrees:
                    print(f"  ⚠️  Clamped to {clamped}° (unsafe range)")
                else:
                    print(f"  ✓ Safe angle")

                ticks = self.degree_to_ticks(servo_id, clamped)
                converted_deg = self.ticks_to_degree(servo_id, ticks)
                print(f"  Converted: {ticks} ticks → {converted_deg}° (verify)")

                # Send command
                pos = self.home_pos.copy()
                pos[servo_id] = ticks

                if self.send_command(pos):
                    print(f"  ✓ Command sent!")
                    self.print_servo_angles(pos)
                else:
                    print(f"  ✗ Command failed")

            except ValueError:
                print("❌ Invalid input")
            except KeyboardInterrupt:
                break

    def go_home(self):
        """Return to home position"""
        print("\n🏠 Returning to home position...")
        current_pos = self.get_feedback()
        trajectory = self.interpolate(current_pos, self.home_pos, steps=15)
        for pos in trajectory:
            self.send_command(pos)
            time.sleep(0.05)
        print("✓ At home position")
        self.print_servo_angles(self.home_pos)


def main():
    """Simple walking gait controller"""
    client = SimpleWalkClient(server_ip="localhost", server_port=5555)

    print("\n" + "=" * 60)
    print("Simple Walking Gait - 3 Joints (Hip, Knee, Foot)")
    print("Conversion: degrees = (ticks / 4096) * 360")
    print("=" * 60)
    print("\nCommands:")
    print("  walk [steps] [hip_deg] [knee_deg] [foot_deg]")
    print("    Angles in ABSOLUTE degrees (0-360)")
    print("\nExamples:")
    print("  walk                    → 3 steps, 100° hip, 150° knee, 90° foot")
    print("  walk 5 120 160 100      → 5 steps, custom angles")
    print("  walk 1 80 140 80        → 1 step, smaller motion")
    print("  test                    → Test individual joints")
    print("  home                    → Return to home")
    print("  exit                    → Quit")
    print("=" * 60)

    while True:
        try:
            user_input = input("\n> ").strip().lower()

            if user_input == "exit" or user_input == "quit":
                client.go_home()
                print("👋 Goodbye!")
                break

            elif user_input == "home":
                client.go_home()

            elif user_input == "test":
                client.test_angles()

            elif user_input.startswith("walk"):
                parts = user_input.split()
                num_steps = 3
                hip_deg = 100
                knee_deg = 150
                foot_deg = 90

                if len(parts) > 1:
                    try:
                        num_steps = int(parts[1])
                        if len(parts) > 2:
                            hip_deg = float(parts[2])
                        if len(parts) > 3:
                            knee_deg = float(parts[3])
                        if len(parts) > 4:
                            foot_deg = float(parts[4])

                        # Validate ranges
                        num_steps = max(1, min(10, num_steps))
                        hip_deg = max(0, min(360, hip_deg))
                        knee_deg = max(0, min(360, knee_deg))
                        foot_deg = max(0, min(360, foot_deg))

                        client.walk_forward(num_steps, hip_deg, knee_deg, foot_deg)
                    except ValueError:
                        print(
                            "❌ Invalid format. Use: walk [steps] [hip] [knee] [foot]"
                        )
                else:
                    client.walk_forward()

            else:
                print("❌ Unknown command")

        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
            client.go_home()
            break
        except Exception as e:
            print(f"❌ Error: {e}")


if __name__ == "__main__":
    main()
