import zmq
import json
import time
import math
from threading import Thread, Event

class DualLegGait:
    """Unified controller cho cả 2 chân (phải & trái) của robot bipedal"""
    
    def __init__(self, right_host: str = "mobile1.local", right_port: int = 5555,
                 left_host: str = "mobile2.local", left_port: int = 5556):
        # Right leg (chan phai)
        self.right = self._create_leg_controller(right_host, right_port, "RIGHT")
        # Left leg (chan trai)
        self.left = self._create_leg_controller(left_host, left_port, "LEFT")
        
        print("\n" + "=" * 70)
        print("✓ Dual Leg Gait Controller initialized")
        print(f"  Right leg: {right_host}:{right_port}")
        print(f"  Left leg:  {left_host}:{left_port}")
        print("=" * 70)
    
    def _create_leg_controller(self, host: str, port: int, side: str) -> 'SingleLegController':
        """Create controller for one leg"""
        return SingleLegController(host, port, side)
    
    def go_home_both(self):
        """Về home position cho cả 2 chân"""
        print("\n" + "=" * 70)
        print("🏠 STEP 1: Going HOME (both legs)")
        print("=" * 70)
        
        # Move both legs in parallel
        right_success = self.right.go_home()
        left_success = self.left.go_home()
        
        return right_success and left_success
    
    def go_to_initial_pose_both(self):
        """Move to initial pose cho cả 2 chân"""
        print("\n" + "=" * 70)
        print("🎯 STEP 2: Going to INITIAL POSE (both legs)")
        print("=" * 70)
        
        right_success, right_angles = self.right.go_to_initial_pose()
        left_success, left_angles = self.left.go_to_initial_pose()
        
        return (right_success and left_success), right_angles, left_angles
    
    def synchronized_swing_gait(self, right_angles: dict, left_angles: dict, 
                                num_cycles: int = 5, hip_swing_range: float = 15,
                                knee_swing_range: float = 20):
        """
        ✅ Synchronized swing gait cho cả 2 chân
        2 chân đi ngược pha (khi chân phải swing forward, chân trái swing backward)
        """
        print("\n" + "=" * 70)
        print("🚶 STEP 3: SYNCHRONIZED SWING GAIT")
        print(f"   Cycles: {num_cycles}")
        print(f"   Hip swing: ±{hip_swing_range}°")
        print(f"   Knee swing: ±{knee_swing_range}°")
        print("   Phase: RIGHT forward ↔ LEFT backward (alternating)")
        print("=" * 70)
        
        initial_hip_r = right_angles["hip"]
        initial_knee_r = right_angles["knee"]
        initial_hip_l = left_angles["hip"]
        initial_knee_l = left_angles["knee"]
        
        steps_per_cycle = 30
        
        for cycle in range(num_cycles):
            print(f"\n📍 Cycle {cycle + 1}/{num_cycles}")
            print("  [Synchronized swinging]...")
            
            for step in range(steps_per_cycle + 1):
                t = step / steps_per_cycle  # 0 → 1
                
                # RIGHT leg: normal phase
                swing_phase_r = math.sin(2 * math.pi * t)
                hip_deg_r = initial_hip_r + swing_phase_r * hip_swing_range
                knee_deg_r = initial_knee_r + swing_phase_r * knee_swing_range
                foot_deg_r = -(hip_deg_r + knee_deg_r)
                
                # LEFT leg: opposite phase (180° offset)
                swing_phase_l = math.sin(2 * math.pi * (t + 0.5))
                hip_deg_l = initial_hip_l + swing_phase_l * hip_swing_range
                knee_deg_l = initial_knee_l + swing_phase_l * knee_swing_range
                foot_deg_l = -(hip_deg_l + knee_deg_l)
                
                # Build position arrays
                pos_r = self.right.home_pos.copy()
                pos_r[1] = self.right.degree_to_ticks(5, hip_deg_r)
                pos_r[3] = self.right.degree_to_ticks(7, knee_deg_r)
                pos_r[4] = self.right.degree_to_ticks(8, foot_deg_r)
                
                pos_l = self.left.home_pos.copy()
                pos_l[1] = self.left.degree_to_ticks(5, hip_deg_l)
                pos_l[3] = self.left.degree_to_ticks(7, knee_deg_l)
                pos_l[4] = self.left.degree_to_ticks(8, foot_deg_l)
                
                # Send commands to both legs
                success_r = self.right.send_command(pos_r)
                success_l = self.left.send_command(pos_l)
                
                if not (success_r and success_l):
                    print("❌ Failed to send command to one or both legs")
                    return False
                
                time.sleep(0.03)
            
            # Print status
            print(f"  ✓ Cycle {cycle + 1} complete")
        
        print("\n" + "=" * 70)
        print("✅ Synchronized swing gait complete!")
        print("=" * 70)
        return True
    
    def run_full_walking_sequence(self, num_cycles: int = 5):
        """Run full walking sequence cho cả 2 chân"""
        print("\n" + "=" * 70)
        print("🤖 FULL BIPEDAL WALKING SEQUENCE")
        print("=" * 70)
        
        # Step 1: Home
        if not self.go_home_both():
            print("❌ Failed to go home")
            return False
        
        time.sleep(1)
        
        # Step 2: Initial pose
        success, right_angles, left_angles = self.go_to_initial_pose_both()
        if not success:
            print("❌ Failed to go to initial pose")
            return False
        
        time.sleep(1)
        
        # Step 3: Synchronized swing
        if not self.synchronized_swing_gait(right_angles, left_angles, num_cycles=num_cycles):
            print("❌ Failed during synchronized swing gait")
            return False
        
        # Return home
        print("\n" + "=" * 70)
        print("🏠 Returning to HOME")
        print("=" * 70)
        if not self.go_home_both():
            print("❌ Failed to return home")
            return False
        
        print("\n" + "=" * 70)
        print("🎉 WALKING SEQUENCE COMPLETE!")
        print("=" * 70)
        return True
    
    def read_positions_both(self):
        """Read current position của cả 2 chân"""
        print("\n" + "=" * 70)
        print("📍 Reading Positions (Both Legs)")
        print("=" * 70)
        
        print("\n[RIGHT LEG]")
        self.right.read_current_position()
        
        print("\n[LEFT LEG]")
        self.left.read_current_position()


class SingleLegController:
    """Controller cho 1 chân (phải hoặc trái)"""
    
    def __init__(self, server_ip: str, server_port: int, side: str = "RIGHT"):
        self.server_ip = server_ip
        self.server_port = server_port
        self.side = side.upper()
        
        # ZeroMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{server_ip}:{server_port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 5000)
        
        # Config based on side
        if self.side == "RIGHT":
            self.home_pos = [370, 2612, 2702, 2287, 2155, 2048]
            self.servo_config = {
                5: {"name": "hip",  "home_ticks": 2612, "min_ticks": 2240, "max_ticks": 3075},
                7: {"name": "knee", "home_ticks": 2287, "min_ticks": 1313, "max_ticks": 3231},
                8: {"name": "foot", "home_ticks": 2155, "min_ticks": 1413, "max_ticks": 2532},
            }
        else:  # LEFT
            self.home_pos = [3536, 1988, 2564, 2535, 1614, 1390]
            self.servo_config = {
                5: {"name": "hip",  "home_ticks": 1988, "min_ticks": 1545, "max_ticks": 2365},
                7: {"name": "knee", "home_ticks": 2535, "min_ticks": 1635, "max_ticks": 3545},
                8: {"name": "foot", "home_ticks": 1614, "min_ticks": 1111, "max_ticks": 2363},
            }
        
        print(f"✓ {self.side} leg controller initialized on {server_ip}:{server_port}")
    
    def degree_to_ticks(self, servo_id: int, degrees_relative: float) -> int:
        """Convert degrees to ticks"""
        config = self.servo_config[servo_id]
        home_ticks = config["home_ticks"]
        min_ticks = config["min_ticks"]
        max_ticks = config["max_ticks"]
        
        if self.side == "RIGHT":
            # RIGHT leg: normal
            min_deg = (min_ticks - home_ticks) / 4096.0 * 360.0
            max_deg = (max_ticks - home_ticks) / 4096.0 * 360.0
            degrees_relative = max(min_deg, min(max_deg, degrees_relative))
            ticks = home_ticks + (degrees_relative / 360.0) * 4096.0
        else:
            # LEFT leg: inverted (ticks decrease = angle increases)
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
    
    def ticks_to_degree(self, servo_id: int, ticks: int) -> float:
        """Convert ticks to degrees"""
        config = self.servo_config[servo_id]
        home_ticks = config["home_ticks"]
        
        if self.side == "RIGHT":
            degrees_relative = (ticks - home_ticks) / 4096.0 * 360.0
        else:
            degrees_relative = (home_ticks - ticks) / 4096.0 * 360.0
        
        return round(degrees_relative, 2)
    
    def print_angles(self, positions: list, label: str = ""):
        """Print current angles"""
        if label:
            print(f"\n📐 {label}")
        else:
            print(f"\n📐 {self.side} Leg Angles (relative to home = 0°):")
        
        servo_indices = {
            1: (5, "hip"),
            3: (7, "knee"),
            4: (8, "foot"),
        }
        
        for idx, (servo_id, name) in servo_indices.items():
            if idx < len(positions):
                ticks = positions[idx]
                degrees = self.ticks_to_degree(servo_id, ticks)
                status = "→" if degrees >= 0 else "←"
                print(f"  {status} {name:8} {degrees:7.2f}° ({ticks:4d} ticks)")
    
    def send_command(self, positions: list) -> bool:
        """Send command to leg"""
        try:
            self.socket.send_json({"type": "move", "positions": positions})
            response = self.socket.recv_json()
            return response.get("status") == "success"
        except zmq.error.Again:
            print(f"✗ {self.side}: Timeout")
            return False
        except Exception as e:
            print(f"✗ {self.side}: Error {e}")
            return False
    
    def get_feedback(self, timeout: float = 5.0) -> list:
        """Get position feedback"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                self.socket.send_json({"type": "feedback"})
                response = self.socket.recv_json()
                
                if response.get("status") == "success":
                    pos = response.get("servo_pos", self.home_pos)
                    
                    if len(pos) == 6 and not all(p == 0 for p in pos):
                        print(f"  ✓ {self.side} feedback valid")
                        return pos
                
                time.sleep(0.1)
            except:
                time.sleep(0.1)
        
        print(f"❌ {self.side}: Timeout waiting for feedback")
        return self.home_pos
    
    def go_home(self):
        """Return to home"""
        print(f"\n🏠 {self.side} leg → HOME")
        current_pos = self.get_feedback(timeout=5.0)
        self.print_angles(current_pos, f"{self.side} Current:")
        
        if all(p == 0 for p in current_pos):
            if not self.send_command(self.home_pos):
                return False
            time.sleep(0.5)
        else:
            trajectory = self.interpolate(current_pos, self.home_pos, steps=20)
            for pos in trajectory:
                if not self.send_command(pos):
                    return False
                time.sleep(0.04)
        
        self.print_angles(self.home_pos, f"✓ {self.side} at HOME")
        return True
    
    def go_to_initial_pose(self):
        """Move to initial pose"""
        print(f"\n🎯 {self.side} leg → INITIAL POSE")
        
        initial_hip_deg = 15
        initial_knee_deg = -30
        initial_foot_deg = 15
        
        initial_pos = self.home_pos.copy()
        initial_pos[1] = self.degree_to_ticks(5, initial_hip_deg)
        initial_pos[3] = self.degree_to_ticks(7, initial_knee_deg)
        initial_pos[4] = self.degree_to_ticks(8, initial_foot_deg)
        
        current_pos = self.get_feedback(timeout=2.0)
        
        if not all(p == 0 for p in current_pos):
            trajectory = self.interpolate(current_pos, initial_pos, steps=20)
        else:
            trajectory = [initial_pos]
        
        for pos in trajectory:
            if not self.send_command(pos):
                return False, None
            time.sleep(0.04)
        
        self.print_angles(initial_pos, f"✓ {self.side} at INITIAL POSE")
        
        return True, {
            "hip": initial_hip_deg,
            "knee": initial_knee_deg,
            "foot": initial_foot_deg,
        }
    
    def read_current_position(self):
        """Read current position"""
        print(f"\n📍 {self.side} Leg Position:")
        current_pos = self.get_feedback(timeout=2.0)
        self.print_angles(current_pos)
        return True
    
    def interpolate(self, start: list, end: list, steps: int) -> list:
        """Linear interpolation"""
        trajectory = []
        for step in range(steps + 1):
            t = step / steps
            pos = [int(start[i] + (end[i] - start[i]) * t) for i in range(6)]
            trajectory.append(pos)
        return trajectory


def main():
    """Interactive dual leg controller"""
    dual = DualLegGait(
        right_host="mobile1.local",
        right_port=5555,
        left_host="mobile2.local",
        left_port=5556
    )
    
    print("\n" + "=" * 70)
    print("DUAL LEG WALKING CONTROLLER")
    print("=" * 70)
    print("\nCommands:")
    print("  run [cycles]       → Run full walking sequence")
    print("  home               → Return both legs to home")
    print("  initial            → Move both legs to initial pose")
    print("  read               → Read current position of both legs")
    print("  exit               → Quit")
    print("=" * 70)
    
    while True:
        try:
            user_input = input("\n> ").strip().lower()
            
            if user_input in ["exit", "quit"]:
                dual.go_home_both()
                print("👋 Goodbye!")
                break
            
            elif user_input == "read":
                dual.read_positions_both()
            
            elif user_input == "home":
                dual.go_home_both()
            
            elif user_input == "initial":
                success, r_ang, l_ang = dual.go_to_initial_pose_both()
                if success:
                    print("✓ Both legs ready for swing!")
            
            elif user_input.startswith("run"):
                parts = user_input.split()
                num_cycles = int(parts[1]) if len(parts) > 1 else 5
                num_cycles = max(1, min(50, num_cycles))
                dual.run_full_walking_sequence(num_cycles=num_cycles)
            
            else:
                print("❌ Unknown command")
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
            dual.go_home_both()
            break
        except Exception as e:
            print(f"❌ Error: {e}")


if __name__ == "__main__":
    main()