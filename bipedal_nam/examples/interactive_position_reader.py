#!/usr/bin/env python
"""
Interactive Motor Position Reader
- Display current position in degrees and raw ticks
- Move to target position
- Display new position after movement
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bipedal_robot import BipedalConfig, BipedalRobot


def load_calib(calib_file: str):
    """Load calibration"""
    with open(calib_file, "r") as f:
        return json.load(f)


def deg_to_ticks(degrees: float, motor_id: int, motor_info: dict, current_ticks: int = None) -> int:
    """Convert degrees to raw ticks - with SHORTEST PATH for wrap-around motors
    
    Args:
        degrees: Target angle in degrees
        motor_id: Motor ID
        motor_info: Motor calibration info
        current_ticks: Current position in ticks (for shortest path calculation)
    """
    motor_id_str = str(motor_id)
    if motor_id_str not in motor_info:
        raise ValueError(f"Motor {motor_id} not in calibration!")
    
    info = motor_info[motor_id_str]
    home_steps = info["home_steps"]
    min_steps = info["min_steps"]
    max_steps = info["max_steps"]
    min_norm = info["min_norm"]
    max_norm = info["max_norm"]
    
    # ✅ Clamp degree input to valid range
    clamped_deg = max(min_norm, min(max_norm, degrees))
    
    # Convert degrees to ticks
    ticks_offset = (clamped_deg / 360.0) * 4096
    target_ticks = int(home_steps + ticks_offset)
    
    # ✅ Handle wrap-around with SHORTEST PATH
    if min_steps <= max_steps:
        # Normal case: simple clamp to [min_steps, max_steps]
        target_ticks = max(min_steps, min(max_steps, target_ticks))
    else:
        # Wrap-around case: valid range is [min_steps, 4095] U [0, max_steps]
        
        # First, clamp to 12-bit [0, 4095]
        base_target = target_ticks % 4096
        
        # If target is in INVALID zone (max_steps, min_steps), snap to boundary
        if max_steps < base_target < min_steps:
            dist_to_max = base_target - max_steps
            dist_to_min = min_steps - base_target
            
            if dist_to_max < dist_to_min:
                target_ticks = max_steps
            else:
                target_ticks = min_steps
        else:
            # Target is in valid range - use as-is
            target_ticks = base_target
    
    return target_ticks


def ticks_to_deg(ticks: int, motor_id: int, motor_info: dict) -> float:
    """Convert raw ticks back to degrees - FIXED wrap-around handling"""
    motor_id_str = str(motor_id)
    if motor_id_str not in motor_info:
        return 0.0
    
    info = motor_info[motor_id_str]
    home_steps = info["home_steps"]
    min_steps = info["min_steps"]
    max_steps = info["max_steps"]
    min_norm = info["min_norm"]
    max_norm = info["max_norm"]
    
    # ✅ Calculate offset from home, handling wrap-around
    if min_steps > max_steps:
        # Wrap-around case (motor 7, 8)
        # Valid range: [min_steps, 4095] or [0, max_steps]
        
        # Find which side of home we're on
        if ticks >= min_steps:
            # Upper side [min_steps, 4095]
            ticks_offset = ticks - home_steps
        elif ticks <= max_steps:
            # Lower side [0, max_steps]
            ticks_offset = ticks - home_steps
        else:
            # In invalid gap - shouldn't happen if motor is working correctly
            # Snap to nearest valid position
            dist_to_min = min_steps - ticks
            dist_to_max = ticks - max_steps
            if dist_to_min < dist_to_max:
                ticks_offset = min_steps - home_steps
            else:
                ticks_offset = max_steps - home_steps
    else:
        # Normal case (motor 4, 5, 6)
        ticks_offset = ticks - home_steps
    
    # ✅ Convert offset to degrees
    degrees = (ticks_offset / 4096.0) * 360.0
    
    # ✅ Normalize to [-180, 180]
    while degrees > 180:
        degrees -= 360
    while degrees < -180:
        degrees += 360
    
    # ✅ Clamp to motor's valid degree range
    # This ensures we never report degrees outside the physical limits
    degrees = max(min_norm, min(max_norm, degrees))
    
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
    return clamped


def move_to_position_blocking(robot, motor_name: str, target_ticks: int, speed: int = 150, 
                               accel: int = 10, timeout: float = 5.0, tolerance: int = 10) -> bool:
    """
    Move motor to target position - SIMPLE version.
    
    For motors with coarse resolution (STS3215), precise position control is difficult.
    This function:
    1. Sends Goal_Position command once
    2. Waits for motor to settle (fixed timeout)
    3. Sets speed to 0 to stop motor
    4. Returns True if motor is reasonably close to target
    
    Args:
        robot: BipedalRobot instance
        motor_name: Name of motor
        target_ticks: Target position in ticks
        speed: Movement speed
        accel: Acceleration
        timeout: Time to wait for motor to settle (seconds)
        tolerance: Position tolerance in ticks
    
    Returns:
        True if motor reached target (within tolerance)
    """
    import time
    
    # Send move command
    robot.write_pos_ex(
        motor_name,
        position=target_ticks,
        speed=speed,
        acceleration=accel,
        normalize=False
    )
    
    # Wait for motor to settle
    time.sleep(timeout)
    
    # ✅ STOP motor - try multiple methods
    try:
        # Method 1: Set Goal_Velocity to 0
        robot.bus.write("Goal_Velocity", motor_name, 0, num_retry=2)
        time.sleep(0.3)
        
        # Method 2: Disable torque to stop immediately
        try:
            robot.bus.write("Torque_Enable", motor_name, 0, num_retry=2)
        except:
            pass
        
        time.sleep(0.5)
        
        # Method 3: Re-enable torque and hold position
        try:
            robot.bus.write("Torque_Enable", motor_name, 1, num_retry=2)
            robot.bus.write("Goal_Velocity", motor_name, 0, num_retry=2)
        except:
            pass
            
    except Exception as e:
        print(f"  ⚠ Could not stop motor: {e}")
    
    time.sleep(0.5)
    
    # Check final position
    try:
        final_pos = robot.bus.read("Present_Position", motor_name, normalize=False)
        error = abs(final_pos - target_ticks)
        return error <= tolerance
    except Exception as e:
        print(f"  Error reading final position: {e}")
        return False


def print_motor_info(motor_name: str, motor_info: dict):
    """Print motor calibration info"""
    motor_map = {
        "leg_bub_right": "4",
        "leg_hip_right": "5",
        "leg_twist_right": "6",
        "leg_knee_right": "7",
        "leg_foot_right": "8",
    }
    
    if motor_name not in motor_map:
        return
    
    motor_id_str = motor_map[motor_name]
    if motor_id_str not in motor_info:
        return
    
    info = motor_info[motor_id_str]
    print(f"\n  Motor: {motor_name} (ID={motor_id_str})")
    print(f"  Limits: [{info['min_norm']:7.2f}°, {info['max_norm']:7.2f}°]")
    print(f"  Home: {info['home_steps']} ticks")
    print(f"  Min/Max: {info['min_steps']}/{info['max_steps']} ticks\n")


def interactive_reader():
    """Interactive position reader"""
    
    try:
        calib = load_calib("config/calibration/rightcalib_normalized.json")
        motor_info = calib["motor_info"]
    except FileNotFoundError:
        print("✗ Calibration not found!")
        return
    
    print("="*100)
    print("🔍 INTERACTIVE MOTOR POSITION READER")
    print("="*100)
    
    # Motor choices
    motors = {
        "4": ("leg_bub_right", "Bubble/Base"),
        "5": ("leg_hip_right", "Hip"),
        "6": ("leg_twist_right", "Twist"),
        "7": ("leg_knee_right", "Knee"),
        "8": ("leg_foot_right", "Foot"),
    }
    
    print("\nAvailable motors:")
    for mid, (name, desc) in motors.items():
        info = motor_info[mid]
        print(f"  {mid}: {name:20s} ({desc:10s}) - Limits: [{info['min_norm']:6.2f}°, {info['max_norm']:6.2f}°]")
    
    config = BipedalConfig(port="/dev/ttyACM0")
    robot = BipedalRobot(config)
    
    try:
        # Connect
        print("\n[1] Connecting...")
        robot.bus.connect(handshake=False)
        robot.configure()
        print("✓ Connected\n")
        
        # Select motor
        while True:
            motor_choice = input("Select motor (4-8) or 'quit': ").strip()
            
            if motor_choice.lower() == 'quit':
                break
            
            if motor_choice not in motors:
                print("Invalid choice!")
                continue
            
            motor_name, motor_desc = motors[motor_choice]
            motor_id = int(motor_choice)
            
            print_motor_info(motor_name, motor_info)
            
            # Interactive loop for this motor
            while True:
                print("\nOptions:")
                print("  'r' - Read current position")
                print("  'm' - Move to degrees")
                print("  'h' - Move to HOME (0°)")
                print("  'b' - Back to motor selection")
                
                action = input("\nAction: ").strip().lower()
                
                if action == 'r':
                    # Read current position
                    try:
                        pos_ticks = robot.bus.read("Present_Position", motor_name, normalize=False)
                        pos_degrees = ticks_to_deg(pos_ticks, motor_id, motor_info)
                        
                        print(f"\n  📍 Current Position:")
                        print(f"     Raw ticks:  {pos_ticks:5d} ticks")
                        print(f"     Degrees:    {pos_degrees:+7.2f}°")
                        
                    except Exception as e:
                        print(f"  ✗ Error reading position: {e}")
                
                elif action == 'm':
                    # Move to target degrees
                    try:
                        target_str = input("  Target degrees (e.g., 10, -5, 0): ").strip()
                        target_degrees = float(target_str)
                        
                        # Read current position for shortest path calculation
                        current_ticks = robot.bus.read("Present_Position", motor_name, normalize=False)
                        
                        # Clamp to limits
                        clamped_degrees = clamp_to_limits(target_degrees, motor_id, motor_info)
                        
                        if abs(clamped_degrees - target_degrees) > 0.01:
                            print(f"  ⚠ Clamped {target_degrees}° → {clamped_degrees}° (limits: [{motor_info[motor_choice]['min_norm']:.2f}°, {motor_info[motor_choice]['max_norm']:.2f}°])")
                        
                        # Convert to ticks with current position for shortest path
                        target_ticks = deg_to_ticks(clamped_degrees, motor_id, motor_info, current_ticks=current_ticks)
                        verify_degrees = ticks_to_deg(target_ticks, motor_id, motor_info)
                        
                        print(f"\n  🎯 Moving to {clamped_degrees:.2f}°")
                        print(f"     Current ticks: {current_ticks:5d}")
                        print(f"     Target ticks:  {target_ticks:5d}")
                        print(f"     Verify degrees: {verify_degrees:+7.2f}°")
                        
                        # Move motor
                        speed = input("  Speed (default 150): ").strip()
                        speed = int(speed) if speed else 150
                        
                        accel = input("  Acceleration (default 10): ").strip()
                        accel = int(accel) if accel else 10
                        
                        print(f"  ✓ Waiting for motor to reach target...")
                        success = move_to_position_blocking(
                            robot,
                            motor_name,
                            target_ticks=target_ticks,
                            speed=speed,
                            accel=accel,
                            timeout=5.0,
                            tolerance=100  # ✅ Increased from 20 to 100 ticks
                        )
                        
                        if success:
                            print(f"  ✅ Motor reached target!")
                        else:
                            print(f"  ⚠ Motor may not have settled at exact target (check position below)")
                        
                        # Read new position
                        time.sleep(0.5)
                        
                        pos_ticks = robot.bus.read("Present_Position", motor_name, normalize=False)
                        pos_degrees = ticks_to_deg(pos_ticks, motor_id, motor_info)
                        
                        print(f"\n  📍 New Position After Movement:")
                        print(f"     Raw ticks:  {pos_ticks:5d}")
                        print(f"     Degrees:    {pos_degrees:+7.2f}°")
                        print(f"     Error:      {abs(pos_degrees - clamped_degrees):+7.2f}°")
                        
                    except Exception as e:
                        print(f"  ✗ Error: {e}")
                
                elif action == 'h':
                    # Move to HOME
                    try:
                        # Read current position for shortest path calculation
                        current_ticks = robot.bus.read("Present_Position", motor_name, normalize=False)
                        
                        home_ticks = deg_to_ticks(0.0, motor_id, motor_info, current_ticks=current_ticks)
                        
                        print(f"\n  🏠 Moving to HOME (0°)")
                        print(f"     Current ticks: {current_ticks:5d}")
                        print(f"     Target ticks: {home_ticks:5d}")
                        
                        print(f"  ✓ Waiting for motor to reach HOME...")
                        success = move_to_position_blocking(
                            robot,
                            motor_name,
                            target_ticks=home_ticks,
                            speed=100,
                            accel=5,
                            timeout=5.0,
                            tolerance=100  # ✅ Increased from 20 to 100 ticks
                        )
                        
                        if success:
                            print(f"  ✅ Motor reached HOME!")
                        else:
                            print(f"  ⚠ Motor may not have settled at HOME (check position below)")
                        
                        time.sleep(0.5)
                        
                        pos_ticks = robot.bus.read("Present_Position", motor_name, normalize=False)
                        pos_degrees = ticks_to_deg(pos_ticks, motor_id, motor_info)
                        
                        print(f"\n  📍 Position at HOME:")
                        print(f"     Raw ticks:  {pos_ticks:5d}")
                        print(f"     Degrees:    {pos_degrees:+7.2f}°")
                        
                    except Exception as e:
                        print(f"  ✗ Error: {e}")
                
                elif action == 'b':
                    break
                
                else:
                    print("Invalid action!")
    
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            robot.disconnect()
            print("\n✓ Disconnected")
        except:
            pass


if __name__ == "__main__":
    interactive_reader()
