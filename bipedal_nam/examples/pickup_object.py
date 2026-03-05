#!/usr/bin/env python
"""
Pickup object with feedback - wait until motor reached goal position.

Workflow:
  1. Move motor 4 to 2300 (arm up) + WAIT UNTIL REACHED
  2. Move motor 9 to 2854 (gripper open) + WAIT UNTIL REACHED
  3. Move motor 4 to 2813 (arm down slowly) + WAIT UNTIL REACHED
  4. Move motor 9 to 2300 (gripper close) + WAIT UNTIL REACHED
  5. Move motor 4 back to 2300 (arm up slowly) + WAIT UNTIL REACHED
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bipedal_robot import BipedalConfig, BipedalRobot
from lerobot.motors import MotorCalibration

import logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def load_normalized_calib(calib_file: str):
    """Load normalized calibration"""
    with open(calib_file, "r") as f:
        return json.load(f)


def setup_calibration_in_bus(robot, calib):
    """Setup calibration in FeetechMotorsBus."""
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


def move_motor(robot, motor_name: str, target_pos: int, speed: int = 150, 
               accel: int = 10, timeout: float = 5.0, tolerance: int = 50):
    """
    Move motor to target position and WAIT until reached (with feedback).
    
    Args:
        robot: BipedalRobot instance
        motor_name: Motor name (e.g., "leg_bub_right")
        target_pos: Target position (raw ticks)
        speed: Movement speed (0-2047)
        accel: Acceleration (0-254)
        timeout: Max time to wait (seconds)
        tolerance: Position tolerance in ticks (±tolerance)
    
    Returns:
        bool: True if reached, False if timeout
    """
    try:
        # Send command
        robot.write_pos_ex(
            motor_name,
            position=target_pos,
            speed=speed,
            acceleration=accel,
            normalize=False
        )
        
        start_time = time.time()
        reached = False
        
        # ✅ LOOP: Kiểm tra position liên tục
        while time.time() - start_time < timeout:
            current = robot.bus.read("Present_Position", motor_name, normalize=False)
            error = abs(current - target_pos)
            
            # Kiểm tra đã đến goal?
            if error <= tolerance:
                elapsed = time.time() - start_time
                logger.info(f"  ✓ {motor_name}: {current} ticks (target: {target_pos}, error: ±{error}) [{elapsed:.2f}s]")
                reached = True
                break
            
            time.sleep(0.05)  # Check mỗi 50ms
        
        if not reached:
            # Timeout - motor chưa đến
            current = robot.bus.read("Present_Position", motor_name, normalize=False)
            error = abs(current - target_pos)
            logger.warning(f"  ⚠️ {motor_name}: TIMEOUT! Pos={current}, Target={target_pos}, Error=±{error}")
            return False
        
        return True
        
    except Exception as e:
        logger.error(f"  ✗ Error moving {motor_name}: {e}")
        return False


def pickup_object():
    """Pickup object workflow with feedback control"""
    
    try:
        calib = load_normalized_calib("config/calibration/rightcalib_normalized.json")
        motor_info = calib["motor_info"]
    except FileNotFoundError:
        logger.error("✗ Calibration not found!")
        logger.error("  Run: python examples/normalize_calibration.py")
        return
    
    print("="*100)
    print("🤖 PICKUP OBJECT - MOTOR 4 (Arm) + MOTOR 9 (Gripper) WITH FEEDBACK")
    print("="*100 + "\n")
    
    config = BipedalConfig(port="/dev/ttyACM0")
    robot = BipedalRobot(config)
    
    try:
        # Connect
        logger.info("[1/6] Connecting to motors...")
        robot.bus.connect(handshake=False)
        robot.configure()
        logger.info("✓ Connected\n")
        
        # Setup calibration
        logger.info("[2/6] Setting up calibration...")
        setup_calibration_in_bus(robot, calib)
        
        # Get motor references
        motor_4 = "leg_bub_right"  # Arm motor
        motor_9 = "leg_gripper_right"  # Gripper motor
        
        # Check if motors exist
        if motor_4 not in robot.bus.motors:
            logger.error(f"✗ Motor 4 ({motor_4}) not found!")
            return
        if motor_9 not in robot.bus.motors:
            logger.error(f"✗ Motor 9 ({motor_9}) not found!")
            return
        
        logger.info("[3/6] Reading current positions...")
        pos_4 = robot.bus.read("Present_Position", motor_4, normalize=False)
        pos_9 = robot.bus.read("Present_Position", motor_9, normalize=False)
        logger.info(f"  Motor 4 (arm): {pos_4}")
        logger.info(f"  Motor 9 (gripper): {pos_9}\n")
        
        success_count = 0
        
        move_speed = 700
        move_accel = 70

        # Step 1: Move motor 4 to 2400 (arm up)
        logger.info("[4/6] Step 1: Arm up - Moving motor 4 to 2600...")
        if move_motor(robot, motor_4, target_pos=2600, speed=move_speed, accel=move_accel, timeout=5.0, tolerance=50):
            success_count += 1
        else:
            logger.error("  ✗ FAILED - continuing anyway...\n")
        logger.info("")
        
        # Step 2: Move motor 9 to 2854 (gripper open)
        logger.info("[5/6] Step 2: Gripper open - Moving motor 9 to 2854...")
        if move_motor(robot, motor_9, target_pos=2854, speed=move_speed, accel=move_accel, timeout=5.0, tolerance=50):
            success_count += 1
        else:
            logger.error("  ✗ FAILED - continuing anyway...\n")
        logger.info("")
        
        # Step 3: Move motor 4 SLOWLY to 1745 (arm down to object)
        logger.info("[6/6] Step 3: Arm down (SLOW) - Moving motor 4 to 1745...")
        if move_motor(robot, motor_4, target_pos=2071, speed=move_speed, accel=move_accel, timeout=10.0, tolerance=50):
            success_count += 1
        else:
            logger.error("  ✗ FAILED - continuing anyway...\n")
        logger.info("")
        
        # Step 4: Move motor 9 NORMAL speed to 2200 (gripper close)
        logger.info("[7/6] Step 4: Gripper close - Moving motor 9 to 2200...")
        if move_motor(robot, motor_9, target_pos=2280, speed=move_speed, accel=move_accel, timeout=5.0, tolerance=50):
            success_count += 1
        else:
            logger.error("  ✗ FAILED - continuing anyway...\n")
        logger.info("")
        
        # Step 5: Move motor 4 SLOWLY back to 2500 (arm up with object)
        logger.info("[8/6] Step 5: Arm up (SLOW) - Moving motor 4 back to 2500...")
        if move_motor(robot, motor_4, target_pos=2600, speed=move_speed, accel=move_accel, timeout=10.0, tolerance=50):
            success_count += 1
        else:
            logger.error("  ✗ FAILED - continuing anyway...\n")
        logger.info("")
        
        # Verify final positions
        logger.info("Verifying final positions:")
        logger.info("-"*100)
        pos_4_final = robot.bus.read("Present_Position", motor_4, normalize=False)
        pos_9_final = robot.bus.read("Present_Position", motor_9, normalize=False)

        error_4 = abs(pos_4_final - 2600)
        error_9 = abs(pos_9_final - 2280)

        logger.info(f"  Motor 4 (arm): {pos_4_final} (target: 2600, error: ±{error_4})")
        logger.info(f"  Motor 9 (gripper): {pos_9_final} (target: 2280, error: ±{error_9})")
        logger.info("-"*100 + "\n")
        
        logger.info("="*100)
        logger.info(f"✓ PICKUP COMPLETED! ({success_count}/5 steps successful)")
        logger.info("="*100)
        
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            robot.disconnect()
            logger.info("✓ Disconnected\n")
        except:
            pass


if __name__ == "__main__":
    pickup_object()