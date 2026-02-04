#!/usr/bin/env python
"""
Simple walking gait with position + speed + acceleration control.
Giống Bimo: check position + gửi walking gait.
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bipedal_robot import BipedalConfig, BipedalRobot
from lerobot.motors import MotorCalibration


def load_normalized_calib(calib_file: str):
    """Load normalized calibration"""
    with open(calib_file, "r") as f:
        return json.load(f)


def setup_calibration_in_bus(robot, calib):
    """Setup calibration in FeetechMotorsBus."""
    motor_limits = calib["motor_limits"]
    motor_info = calib.get("motor_info", {})
    
    calibration = {}
    
    for motor_name, motor in robot.bus.motors.items():
        motor_id = motor.id
        motor_id_str = str(motor_id)
        
        if motor_id_str in motor_info:
            info = motor_info[motor_id_str]
            home_steps = info["home_steps"]
            min_steps = info["min_steps"]
            max_steps = info["max_steps"]
        else:
            home_steps = 2048
            min_steps = 0
            max_steps = 4095
        
        calibration[motor_name] = MotorCalibration(
            id=motor_id,
            drive_mode=0,
            homing_offset=home_steps,
            range_min=min_steps,
            range_max=max_steps,
        )
    
    robot.bus.calibration = calibration
    print(f"✓ Setup calibration for {len(calibration)} motors\n")


def deg_to_ticks(degrees: float, motor_id: int, motor_info: dict) -> int:
    """Convert degrees to raw ticks using calibration, handle wrap-around"""
    motor_id_str = str(motor_id)
    if motor_id_str not in motor_info:
        raise ValueError(f"Motor {motor_id} not in calibration!")
    
    info = motor_info[motor_id_str]
    home_steps = info["home_steps"]
    min_steps = info["min_steps"]
    max_steps = info["max_steps"]
    
    # Convert degrees to ticks offset
    # degrees / 360 * 4096 = ticks offset from HOME
    ticks_offset = (degrees / 360.0) * 4096
    target_ticks = int(home_steps + ticks_offset)
    
    # ✅ CLAMP to valid motor range FIRST (before wrap-around)
    if min_steps <= max_steps:
        # Normal case: motor 4, 5, 6
        target_ticks = max(min_steps, min(max_steps, target_ticks))
    else:
        # Wrap-around case: motor 7, 8
        # min_steps > max_steps means valid range is [min_steps, 4095] U [0, max_steps]
        
        # First clamp to 12-bit [0, 4095]
        target_ticks = target_ticks % 4096
        
        # Check if in valid range
        if target_ticks < min_steps and target_ticks > max_steps:
            # In invalid gap - snap to nearest boundary
            # Use shortest path distance considering wrap-around
            dist_to_min = min_steps - target_ticks
            dist_to_max = target_ticks - max_steps
            if dist_to_min < dist_to_max:
                target_ticks = min_steps
            else:
                target_ticks = max_steps
    
    return target_ticks


def ticks_to_deg(ticks: int, motor_id: int, motor_info: dict) -> float:
    """Convert raw ticks back to degrees (for verification)"""
    motor_id_str = str(motor_id)
    if motor_id_str not in motor_info:
        return 0.0
    
    info = motor_info[motor_id_str]
    home_steps = info["home_steps"]
    min_steps = info["min_steps"]
    max_steps = info["max_steps"]
    
    # ✅ FIX: Xử lý wrap-around đúng cho motor 8
    # Motor 8: min_steps=4, max_steps=3993 (wrap-around)
    # Valid range: [4, 3993] hoặc wrap qua 0 → [4, 65535]
    
    if min_steps > max_steps:
        # Wrap-around motor (motor 7, 8)
        # Home ở giữa, có thể quay 2 hướng qua wrap
        
        # Tính offset từ home
        # Nếu ticks gần home, dùng trực tiếp
        # Nếu ticks xa home qua wrap, phải handle
        
        if ticks >= min_steps:
            # Ở phía min_steps → max_steps
            ticks_offset = ticks - home_steps
        elif ticks <= max_steps:
            # Ở phía max_steps → 0
            ticks_offset = ticks - home_steps
        else:
            # Ở giữa (invalid gap), snap to nearest
            dist_to_min = min_steps - ticks
            dist_to_max = ticks - max_steps
            if dist_to_min < dist_to_max:
                ticks_offset = min_steps - home_steps
            else:
                ticks_offset = max_steps - home_steps
    else:
        # Normal motor (4, 5, 6)
        ticks_offset = ticks - home_steps
    
    # Convert offset back to degrees
    degrees = (ticks_offset / 4096.0) * 360.0
    
    # Normalize to [-180, 180]
    while degrees > 180:
        degrees -= 360
    while degrees < -180:
        degrees += 360
    
    return degrees


def clamp_to_limits(value: float, motor_id: int, motor_info: dict) -> float:
    """Clamp degree value to motor limits"""
    motor_id_str = str(motor_id)
    if motor_id_str not in motor_info:
        return value
    
    info = motor_info[motor_id_str]
    min_limit = info["min_norm"]
    max_limit = info["max_norm"]
    
    clamped = max(min_limit, min(max_limit, value))
    
    if clamped != value:
        print(f"    ⚠ Motor {motor_id}: {value}° clamped to [{min_limit:.2f}°, {max_limit:.2f}°] → {clamped:.2f}°")
    
    return clamped


def move_to_position_blocking(robot, motor_name: str, target_ticks: int, speed: int = 150, 
                               accel: int = 10, timeout: float = 5.0, tolerance: int = 100) -> bool:
    """
    Move motor to target position with ADAPTIVE SPEED and wait until it stops.
    Reduces speed as it gets closer to target to avoid overshoot.
    
    Returns:
        True if motor reached target (within tolerance)
    """
    robot.write_pos_ex(
        motor_name,
        position=target_ticks,
        speed=speed,
        acceleration=accel,
        normalize=False
    )
    
    start_time = time.time()
    prev_pos = None
    no_movement_count = 0
    last_speed_adjustment = 0
    
    while time.time() - start_time < timeout:
        try:
            current_pos = robot.bus.read("Present_Position", motor_name, normalize=False)
            distance_to_target = abs(current_pos - target_ticks)
            current_time = time.time() - start_time
            
            # Adaptive speed control - reduce as we get closer
            if distance_to_target < 50 and current_time - last_speed_adjustment > 0.2:
                try:
                    robot.bus.write("Goal_Velocity", motor_name, 20, num_retry=1)
                    last_speed_adjustment = current_time
                except:
                    pass
            elif distance_to_target < 100 and current_time - last_speed_adjustment > 0.2:
                try:
                    robot.bus.write("Goal_Velocity", motor_name, 30, num_retry=1)
                    last_speed_adjustment = current_time
                except:
                    pass
            
            # Check if motor has stopped
            if prev_pos == current_pos:
                no_movement_count += 1
            else:
                no_movement_count = 0
            
            if no_movement_count >= 3:
                error = abs(current_pos - target_ticks)
                return error <= tolerance
            
            prev_pos = current_pos
            time.sleep(0.1)
            
        except Exception as e:
            return False
    
    return False


def test_simple_gait():
    """Test simple walking gait with speed + acceleration"""
    
    try:
        calib = load_normalized_calib("config/calibration/rightcalib_normalized.json")
        motor_info = calib["motor_info"]
    except FileNotFoundError:
        print("✗ Normalized calibration not found!")
        print("  Run: python examples/normalize_calibration.py")
        return
    
    print("="*100)
    print("🚶 BIPEDAL WALKING GAIT TEST (Degree-based tuning)")
    print("="*100 + "\n")
    
    # Print motor limits in degrees
    print("Motor Limits (in degrees):")
    print("-"*100)
    for motor_id_str in sorted(motor_info.keys(), key=int):
        motor_id = int(motor_id_str)
        info = motor_info[motor_id_str]
        print(f"  Motor {motor_id}: [{info['min_norm']:7.2f}°, {info['max_norm']:7.2f}°] (range: {info['range_norm']:.2f}°)")
    print("-"*100 + "\n")
    
    config = BipedalConfig(port="/dev/ttyACM0")
    robot = BipedalRobot(config)
    
    try:
        # Connect
        print("[1/7] Connecting...")
        robot.bus.connect(handshake=False)
        robot.configure()
        print("✓ Connected\n")
        
        # Setup calibration
        print("[2/7] Setting up calibration...")
        setup_calibration_in_bus(robot, calib)
        
        # Check current positions TRƯỚC khi move
        print("[3/7] Reading current motor positions...")
        positions = robot.read_leg_positions(normalize=False)
        for motor_name, pos in positions.items():
            if pos is not None:
                print(f"  {motor_name}: {pos} (raw ticks)")
        print()
        
        # Move to HOME
        print("[4/7] Moving to HOME...")
        # Only use leg motors (4, 5, 6, 7, 8) - skip other motors (1, 2, 3, 9)
        leg_motor_names = ["leg_bub_right", "leg_hip_right", "leg_twist_right", "leg_knee_right", "leg_foot_right"]
        motor_ids = {name: robot.bus.motors[name].id for name in leg_motor_names if name in robot.bus.motors}
        
        for motor_name in leg_motor_names:
            if motor_name not in motor_ids:
                continue
            motor_id = motor_ids[motor_name]
            # HOME position is 0 degrees, convert to raw ticks
            home_ticks = deg_to_ticks(0.0, motor_id, motor_info)
            robot.write_pos_ex(
                motor_name,
                position=home_ticks,
                speed=500,
                acceleration=20,
                normalize=False  # Raw ticks, not normalized
            )
        
        time.sleep(2)
        
        # Check position SAU khi HOME
        print("\n[4.1] Position after HOME:")
        positions = robot.read_leg_positions(normalize=False)
        for motor_name, pos in positions.items():
            if pos is not None:
                print(f"  {motor_name}: {pos}")
        print()
        
        # Gait sequence (all values in DEGREES for easy tuning)
        print("[5/7] Testing walking gait...\n")
        
        gait_sequence = [
            {
                "name": "Lift knee",
                "motors": {
                    "leg_bub_right": {"degrees": 20, "speed": 800, "accel": 50},
                    "leg_hip_right": {"degrees": -15, "speed": 800, "accel": 50},
                    "leg_twist_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_knee_right": {"degrees": 15, "speed": 800, "accel": 50},
                    "leg_foot_right": {"degrees": -15, "speed": 800, "accel": 50},  # ✅ Very low speed/accel for foot motor
                },
                "duration": 1.0,
            },
            {
                "name": "Step forward",
                "motors": {
                    "leg_bub_right": {"degrees": 5, "speed": 800, "accel": 50},
                    "leg_hip_right": {"degrees": 5, "speed": 800, "accel": 50},
                    "leg_twist_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_knee_right": {"degrees": -5, "speed": 800, "accel": 50},
                    "leg_foot_right": {"degrees": 5, "speed": 800, "accel": 50},  # ✅ Very low speed/accel for foot motor
                },
                "duration": 1.0,
            },
            {
                "name": "Put foot down",
                "motors": {
                    "leg_bub_right": {"degrees": 20, "speed": 800, "accel": 50},
                    "leg_hip_right": {"degrees": -15, "speed": 800, "accel": 50},
                    "leg_twist_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_knee_right": {"degrees": 15, "speed": 800, "accel": 50},
                    "leg_foot_right": {"degrees": -15, "speed": 800, "accel": 50},  # ✅ Very low speed/accel for foot motor
                },
                "duration": 1.0,
            },
            {
                "name": "Push back",
                "motors": {
                    "leg_bub_right": {"degrees": 5, "speed": 800, "accel": 50},
                    "leg_hip_right": {"degrees": 5, "speed": 800, "accel": 50},
                    "leg_twist_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_knee_right": {"degrees": -5, "speed": 800, "accel": 50},
                    "leg_foot_right": {"degrees": 5, "speed": 800, "accel": 50},  # ✅ Very low speed/accel for foot motor
                },
                "duration": 1.0,
            },
            {
                "name": "Return to HOME",
                "motors": {
                    "leg_bub_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_hip_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_twist_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_knee_right": {"degrees": 0, "speed": 800, "accel": 50},
                    "leg_foot_right": {"degrees": 0, "speed": 800, "accel": 50},  # ✅ Very low speed/accel for foot motor
                },
                "duration": 1.0,
            },
        ]
        
        # Execute gait 2 times
        for iteration in range(2):
            print(f"🔄 Gait cycle {iteration + 1}/2:")
            
            for phase in gait_sequence:
                print(f"  → {phase['name']}")
                
                # STEP 1: Send commands to ALL motors at the same time
                print("    Sending commands to all motors...")
                for motor_name, motor_config in phase["motors"].items():
                    motor_id = motor_ids[motor_name]
                    target_degrees = motor_config["degrees"]
                    speed = motor_config["speed"]
                    accel = motor_config["accel"]
                    
                    # ✅ Clamp to motor limits (in degrees)
                    clamped_degrees = clamp_to_limits(target_degrees, motor_id, motor_info)
                    
                    # ✅ Convert degrees → ticks
                    target_ticks = deg_to_ticks(clamped_degrees, motor_id, motor_info)
                    
                    # ✅ Verify conversion (ticks → degrees back)
                    verify_degrees = ticks_to_deg(target_ticks, motor_id, motor_info)
                    
                    # Debug output
                    print(f"      {motor_name}: {target_degrees:6.2f}° → ticks={target_ticks:5d} (speed={speed}, acc={accel})")
                    
                    # ✅ Send to motor (raw ticks, not normalized)
                    robot.write_pos_ex(
                        motor_name,
                        position=target_ticks,
                        speed=speed,
                        acceleration=accel,
                        normalize=False
                    )
                
                # STEP 2: Wait for all motors to complete movement
                print(f"    Waiting {phase['duration']:.1f}s for motors to reach target...")
                time.sleep(phase["duration"])
                
                # STEP 3 (Optional): Check final positions
                # positions = robot.read_leg_positions(normalize=False)
                # print(f"    ✓ Phase complete\n")
            
            print()
        
        print("[6/7] Gait completed!\n")
        
        # Return to HOME cuối cùng
        print("[7/7] Returning to HOME...")
        for motor_name in leg_motor_names:
            if motor_name not in motor_ids:
                continue
            motor_id = motor_ids[motor_name]
            # HOME position is 0 degrees, convert to raw ticks
            home_ticks = deg_to_ticks(0.0, motor_id, motor_info)
            robot.write_pos_ex(
                motor_name,
                position=home_ticks,
                speed=500,
                acceleration=20,
                normalize=False  # Raw ticks, not normalized
            )
        
        time.sleep(2)
        
        # Final position check
        print("\n[7.1] Final positions:")
        positions = robot.read_leg_positions(normalize=False)
        for motor_name, pos in positions.items():
            if pos is not None:
                print(f"  {motor_name}: {pos}")
        
        print("\n" + "="*100)
        print("✓ TEST COMPLETED")
        print("="*100)
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            robot.disconnect()
            print("✓ Disconnected")
        except:
            pass


if __name__ == "__main__":
    test_simple_gait()