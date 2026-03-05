import zmq
import json
import time
import math

class WalkingGait:
    """Walking gait controller - 3 joints: hip (5), knee (7), foot (8)"""
    
    def __init__(self, server_ip: str = "localhost", server_port: int = 5556):
        self.server_ip = server_ip
        self.server_port = server_port
        
        # ZeroMQ setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{server_ip}:{server_port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 5000)
        
        # ✅ Home position [bub(4), hip(5), twist(6), knee(7), foot(8), gripper(9)]
        self.home_pos = [3536, 1988, 2564, 2535, 1614, 1390]
        
        # ✅ Index mapping: servo_id 4-9 → array index 0-5
        self.servo_id_to_index = {4: 0, 5: 1, 6: 2, 7: 3, 8: 4, 9: 5}
        self.index_to_servo_id = {0: 4, 1: 5, 2: 6, 3: 7, 4: 8, 5: 9}
        
        # ✅ Servo config - HOME = 0° (center)
        # min/max là giới hạn relative từ home
        # ✅ SỬA: Correct min/max for LEFT leg
        # For LEFT leg: positive angle = ticks DECREASE (inverted from RIGHT)
        # But min_ticks should ALWAYS be < max_ticks!
        self.servo_config = {
            # HIP (servo 5): home=1988
            # Forward (+angle) → ticks decrease → toward 1545
            # Backward (-angle) → ticks increase → toward 2365
            5: {
                "name": "hip",
                "home_ticks": 1988,
                "min_ticks": 1545,    # ← Forward limit
                "max_ticks": 2365,    # ← Backward limit
            },
            
            # KNEE (servo 7): home=2535
            # Forward (+angle) → ticks decrease → toward 1635
            # Backward (-angle) → ticks increase → toward 3545
            7: {
                "name": "knee",
                "home_ticks": 2535,
                "min_ticks": 1635,    # ← Forward limit
                "max_ticks": 3545,    # ← Backward limit
            },
            
            # FOOT (servo 8): home=1614
            # Forward (+angle) → ticks decrease → toward 1111
            # Backward (-angle) → ticks increase → toward 2363
            8: {
                "name": "foot",
                "home_ticks": 1614,
                "min_ticks": 1111,    # ← Forward limit
                "max_ticks": 2363,    # ← Backward limit
            },
        }
        
        print("✓ Walking Gait Client (LEFT leg) initialized")
        print("✓ Controlling: Hip (5), Knee (7), Foot (8)")
        print("✓ Home position = 0° (center)")
        print("✓ Array indices: 0=bub(4), 1=hip(5), 2=twist(6), 3=knee(7), 4=foot(8), 5=gripper(9)")
        print("✓ Motor limits SWAPPED for LEFT leg (mirror anatomy)")
    
    def degree_to_ticks(self, servo_id: int, degrees_relative: float) -> int:
        """
        ✅ FIXED: Convert relative degrees to ticks (INVERTED for LEFT leg)
        
        For LEFT leg: positive angle = ticks decrease
    
        Args:
            servo_id: 5, 7, 8
            degrees_relative: angle relative to home (-90 to +90 typically)
    
        Returns:
            ticks value (clamped to min/max)
        """
        config = self.servo_config[servo_id]
        home_ticks = config["home_ticks"]
        min_ticks = config["min_ticks"]      # Forward limit
        max_ticks = config["max_ticks"]      # Backward limit
    
        # ✅ FIX: Calculate max range from home
        # Forward range: home_ticks → min_ticks (positive angle)
        # Backward range: home_ticks → max_ticks (negative angle)
    
        ticks_range_forward = home_ticks - min_ticks    # Range when going forward
        ticks_range_backward = max_ticks - home_ticks   # Range when going backward
    
        # Calculate ticks based on angle direction
        if degrees_relative >= 0:
            # Positive angle = forward = decrease ticks
            # Map: 0° → home, max_deg → min_ticks
            max_angle_forward = (ticks_range_forward / 4096.0) * 360.0
            clipped_deg = min(degrees_relative, max_angle_forward)
            ticks = home_ticks - (clipped_deg / 360.0) * 4096.0
        else:
            # Negative angle = backward = increase ticks
            # Map: 0° → home, min_deg → max_ticks
            max_angle_backward = (ticks_range_backward / 4096.0) * 360.0
            clipped_deg = max(degrees_relative, -max_angle_backward)
            ticks = home_ticks - (clipped_deg / 360.0) * 4096.0
    
        # Final clamp
        ticks = int(max(min_ticks, min(max_ticks, ticks)))
    
        return ticks
    
    def ticks_to_degree(self, servo_id: int, ticks: int) -> float:
        """
        ✅ FIXED: Convert ticks to relative degrees (INVERTED for LEFT leg)
    
        Home (ticks) = 0°
        For LEFT leg: ticks decrease = angle increases (mirrored)
    
        Args:
            servo_id: 5, 7, 8
            ticks: absolute position
    
        Returns:
            angle in degrees relative to home
        """
        config = self.servo_config[servo_id]
        home_ticks = config["home_ticks"]
        
        # ✅ LEFT leg inverted formula
        # Decrease from home = positive angle
        # Increase from home = negative angle
        
        degrees_relative = (home_ticks - ticks) / 4096.0 * 360.0
        
        return round(degrees_relative, 2)
    
    def print_angles(self, positions: list, label: str = ""):
        """
        ✅ In ra góc hiện tại (relative từ home)
        
        positions: array 6 phần tử [index0-5] tương ứng servo [4-9]
        """
        if label:
            print(f"\n📐 {label}")
        else:
            print("\n📐 Current Angles (relative to home = 0°):")
        
        # ✅ FIX: Dùng index (0-5), không dùng servo_id (4-9)
        servo_indices = {
            1: (5, "hip"),      # index 1 = servo 5 (hip)
            3: (7, "knee"),     # index 3 = servo 7 (knee)
            4: (8, "foot"),     # index 4 = servo 8 (foot)
        }
        
        for idx, (servo_id, name) in servo_indices.items():
            if idx < len(positions):
                ticks = positions[idx]
                degrees = self.ticks_to_degree(servo_id, ticks)
                
                # Status indicator
                if degrees >= 0:
                    status = "→"
                else:
                    status = "←"
                
                print(f"  {status} {name:8} {degrees:7.2f}° ({ticks:4d} ticks)")
    
    def send_command(self, positions: list) -> bool:
        """Gửi lệnh move tới server"""
        try:
            self.socket.send_json({
                "type": "move",
                "positions": positions
            })
            response = self.socket.recv_json()
            return response.get("status") == "success"
        except zmq.error.Again:
            print("✗ Timeout: Server không response")
            return False
        except Exception as e:
            print(f"✗ Error: {e}")
            return False
    
    def get_feedback(self, timeout: float = 5.0) -> list:
        """
        ✅ SỬA: Đợi feedback hợp lệ (không phải 0)
        
        Args:
            timeout: Thời gian chờ tối đa (giây)
        
        Returns:
            Position array nếu hợp lệ, home_pos nếu timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                self.socket.send_json({"type": "feedback"})
                response = self.socket.recv_json()
                
                if response.get("status") == "success":
                    pos = response.get("servo_pos", self.home_pos)
                    
                    if len(pos) != 6:
                        print(f"⚠️  Invalid length: {len(pos)}, retrying...")
                        time.sleep(0.1)
                        continue
                    
                    # ✅ NEW: Kiểm tra position có hợp lệ không (không phải 0)
                    if all(p == 0 for p in pos):
                        print("⚠️  Position all zeros, waiting for feedback update...")
                        time.sleep(0.1)
                        continue
                    
                    # ✅ SUCCESS: Feedback hợp lệ
                    print(f"  ✓ Feedback valid: hip={pos[1]} knee={pos[3]} foot={pos[4]}")
                    return pos
                
            except zmq.error.Again:
                print("⚠️  Feedback timeout, retrying...")
                time.sleep(0.1)
                continue
            except Exception as e:
                print(f"✗ Error reading feedback: {e}")
                time.sleep(0.1)
                continue
        
        # Timeout - return home as fallback
        print(f"❌ Timeout waiting for valid feedback (>{timeout}s), using home position")
        return self.home_pos
    
    def go_home(self):
        """Return to home position (all joints = 0°)"""
        print("\n" + "=" * 70)
        print("🏠 STEP 1: Going to HOME position (all joints = 0°)")
        print("=" * 70)
        
        # ✅ NEW: Đợi feedback hợp lệ
        print("\nWaiting for valid feedback...")
        current_pos = self.get_feedback(timeout=5.0)
        
        if current_pos is None or len(current_pos) != 6:
            print("❌ Error: Invalid feedback from server")
            return False
        
        self.print_angles(current_pos, "Current position:")
        
        # ✅ NEW: Check if feedback is valid before interpolating
        if all(p == 0 for p in current_pos):
            print("\n⚠️  WARNING: Invalid position (all 0)")
            print("   Moving directly to HOME with minimal motion...")
            # Send home directly without interpolation
            if not self.send_command(self.home_pos):
                return False
            time.sleep(0.5)
            self.print_angles(self.home_pos, "✓ At home position:")
            return True
        
        # ✅ NORMAL: Smooth interpolation từ current → home
        print("\nSmooth move to home...")
        trajectory = self.interpolate(current_pos, self.home_pos, steps=20)
        for pos in trajectory:
            if not self.send_command(pos):
                return False
            time.sleep(0.04)
        
        self.print_angles(self.home_pos, "✓ At home position (0°, 0°, 0°):")
        return True
    
    def go_to_initial_pose(self):
        """Go to initial pose"""
        print("\n" + "=" * 70)
        print("🎯 STEP 2: Going to INITIAL POSE")
        print("   Hip:  +15° (relative to home)")
        print("   Knee: -30° (relative to home)")
        print("   Foot: +15° (relative to home)")
        print("=" * 70)
        
        initial_hip_deg = 15
        initial_knee_deg = -30
        initial_foot_deg = 15
        
        initial_pos = self.home_pos.copy()
        initial_pos[1] = self.degree_to_ticks(5, initial_hip_deg)
        initial_pos[3] = self.degree_to_ticks(7, initial_knee_deg)
        initial_pos[4] = self.degree_to_ticks(8, initial_foot_deg)
        
        print("\nMoving to initial pose...")
        
        # ✅ NEW: Get current position first
        current_pos = self.get_feedback(timeout=2.0)
        
        # Use interpolation if current position is valid
        if not all(p == 0 for p in current_pos):
            trajectory = self.interpolate(current_pos, initial_pos, steps=20)
        else:
            # Current position invalid, move directly
            trajectory = [initial_pos]
        
        for pos in trajectory:
            if not self.send_command(pos):
                return False, None
            time.sleep(0.04)
        
        self.print_angles(initial_pos, "✓ At initial pose:")
        
        return True, {
            "hip": initial_hip_deg,
            "knee": initial_knee_deg,
            "foot": initial_foot_deg,
        }
    
    def swing_gait(self, initial_angles: dict, num_cycles: int = 5, 
                   hip_swing_range: float = 15, knee_swing_range: float = 20):
        """
        ✅ Swing gait loop (RELATIVE từ home = 0°):
        - Hip: initial_hip ± hip_swing_range (±15°)
        - Knee: initial_knee ± knee_swing_range (±20°)
        - Foot: -(hip + knee)
        """
        print("\n" + "=" * 70)
        print("🚶 STEP 3: SWING GAIT LOOP")
        print(f"   Cycles: {num_cycles}")
        print(f"   Hip swing: ±{hip_swing_range}°")
        print(f"   Knee swing: ±{knee_swing_range}°")
        print(f"   Foot: -(hip + knee)")
        print("=" * 70)
        
        initial_hip = initial_angles["hip"]
        initial_knee = initial_angles["knee"]
        
        steps_per_cycle = 30
        
        for cycle in range(num_cycles):
            print(f"\n📍 Cycle {cycle + 1}/{num_cycles}")
            print("  [Swinging]...")
            
            for step in range(steps_per_cycle + 1):
                t = step / steps_per_cycle  # 0 → 1
                
                # Sinusoidal motion: -1 to +1
                swing_phase = math.sin(2 * math.pi * t)
                
                # Hip swing: initial_hip ± hip_swing_range
                hip_deg = initial_hip + swing_phase * hip_swing_range
                
                # Knee swing: initial_knee ± knee_swing_range
                knee_deg = initial_knee + swing_phase * knee_swing_range
                
                # Foot: -(hip + knee) để balance
                foot_deg = -(hip_deg + knee_deg)
                
                # Create position array
                pos = self.home_pos.copy()
                pos[1] = self.degree_to_ticks(5, hip_deg)   # Index 1 = servo 5
                pos[3] = self.degree_to_ticks(7, knee_deg)  # Index 3 = servo 7
                pos[4] = self.degree_to_ticks(8, foot_deg)  # Index 4 = servo 8
                
                if not self.send_command(pos):
                    return False
                
                time.sleep(0.03)
            
            # Print final swing position
            pos = self.home_pos.copy()
            pos[1] = self.degree_to_ticks(5, hip_deg)
            pos[3] = self.degree_to_ticks(7, knee_deg)
            pos[4] = self.degree_to_ticks(8, foot_deg)
            self.print_angles(pos, f"  Cycle {cycle + 1} complete")
        
        print("\n" + "=" * 70)
        print("✅ Swing gait complete!")
        print("=" * 70)
        return True
    
    def run_full_sequence(self, num_cycles: int = 5):
        """Run full sequence"""
        print("\n" + "=" * 70)
        print("🤖 FULL WALKING SEQUENCE")
        print("=" * 70)
        
        # Step 1: Home
        if not self.go_home():
            print("❌ Failed to go home")
            return False
        
        time.sleep(1)
        
        # Step 2: Initial pose
        success, initial_angles = self.go_to_initial_pose()
        if not success:
            print("❌ Failed to go to initial pose")
            return False
        
        time.sleep(1)
        
        # Step 3: Swing gait
        if not self.swing_gait(initial_angles, num_cycles=num_cycles):
            print("❌ Failed during swing gait")
            return False
        
        # Return home
        print("\n" + "=" * 70)
        print("🏠 Returning to HOME")
        print("=" * 70)
        if not self.go_home():
            print("❌ Failed to return home")
            return False
        
        print("\n" + "=" * 70)
        print("🎉 SEQUENCE COMPLETE!")
        print("=" * 70)
        return True
    
    def interpolate(self, start: list, end: list, steps: int) -> list:
        """Nội suy tuyến tính từ start → end"""
        trajectory = []
        for step in range(steps + 1):
            t = step / steps
            pos = [
                int(start[i] + (end[i] - start[i]) * t)
                for i in range(6)
            ]
            trajectory.append(pos)
        return trajectory
    
    def read_current_position(self):
        """
        ✅ NEW: Đọc và hiển thị vị trí hiện tại của tất cả servo
        Không gửi lệnh di chuyển, chỉ đọc feedback
        """
        print("\n" + "=" * 70)
        print("📍 Reading Current Position")
        print("=" * 70)
        
        # Get feedback
        current_pos = self.get_feedback(timeout=2.0)
        
        if current_pos is None or len(current_pos) != 6:
            print("❌ Failed to read position")
            return False
        
        # Print all servos
        print("\n📐 All Servo Positions:")
        print("-" * 70)
        
        servo_info = [
            (4, "bub",     0),
            (5, "hip",     1),
            (6, "twist",   2),
            (7, "knee",    3),
            (8, "foot",    4),
            (9, "gripper", 5),
        ]
        
        for servo_id, name, idx in servo_info:
            ticks = current_pos[idx]
            
            # Calculate degrees (relative to home)
            if servo_id in self.servo_config:
                degrees = self.ticks_to_degree(servo_id, ticks)
                home_ticks = self.servo_config[servo_id]["home_ticks"]
                print(f"  [{idx}] {name:8} (ID {servo_id}): {ticks:4d} ticks → {degrees:7.2f}° (home: {home_ticks})")
            else:
                # Fixed servo (không điều khiển)
                print(f"  [{idx}] {name:8} (ID {servo_id}): {ticks:4d} ticks (FIXED)")
        
        print("-" * 70)
        
        # Summary for controlled servos
        print("\n📊 Controlled Servos (relative to home = 0°):")
        self.print_angles(current_pos, "")
        
        return True

def main():
    """Interactive walking gait controller"""
    LEG2_HOST = "mobile2.local"
    gait = WalkingGait(server_ip=LEG2_HOST, server_port=5556)

    print("\n" + "=" * 70)
    print("Walking Gait - Home (0°) → Initial Pose → Swing")
    print("All angles are RELATIVE to home position")
    print("=" * 70)
    print("\nCommands:")
    print("  run [cycles]       → Run full sequence (default: 5 cycles)")
    print("  home               → Go to home (all joints = 0°)")
    print("  initial            → Go to initial pose only")
    print("  swing [cycles]     → Swing gait only")
    print("  read               → Read current position ← NEW!")
    print("  exit               → Quit")
    print("=" * 70)
    
    while True:
        try:
            user_input = input("\n> ").strip().lower()
            
            if user_input == "exit" or user_input == "quit":
                gait.go_home()
                print("👋 Goodbye!")
                break
            
            elif user_input == "read":
                gait.read_current_position()
            
            elif user_input == "home":
                gait.go_home()
            
            elif user_input == "initial":
                success, angles = gait.go_to_initial_pose()
                if success:
                    print("✓ Now at initial pose, ready for swing!")
            
            elif user_input.startswith("run"):
                parts = user_input.split()
                num_cycles = int(parts[1]) if len(parts) > 1 else 5
                num_cycles = max(1, min(50, num_cycles))
                
                gait.run_full_sequence(num_cycles=num_cycles)
            
            elif user_input.startswith("swing"):
                parts = user_input.split()
                num_cycles = int(parts[1]) if len(parts) > 1 else 5
                num_cycles = max(1, min(50, num_cycles))
                
                # Create initial angles at current position
                current_pos = gait.get_feedback()
                if current_pos and len(current_pos) == 6:
                    initial_angles = {
                        "hip": gait.ticks_to_degree(5, current_pos[1]),    # Index 1 = servo 5
                        "knee": gait.ticks_to_degree(7, current_pos[3]),   # Index 3 = servo 7
                        "foot": gait.ticks_to_degree(8, current_pos[4]),   # Index 4 = servo 8
                    }
                    gait.swing_gait(initial_angles, num_cycles=num_cycles)
                else:
                    print("❌ Failed to read current position")
            
            else:
                print("❌ Unknown command. Type 'read', 'home', 'initial', 'swing', 'run', or 'exit'")
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
            gait.go_home()
            break
        except ValueError as e:
            print(f"❌ Invalid input: {e}")
        except Exception as e:
            print(f"❌ Error: {e}")


if __name__ == "__main__":
    main()