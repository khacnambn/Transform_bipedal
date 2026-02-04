#!/usr/bin/env python

# Copyright 2024 Nam. All rights reserved.

"""
Read current motor position and save as HOME calibration.
Đọc vị trí hiện tại của motor và lưu làm vị trí HOME.
"""

import json
import logging
import time
from pathlib import Path

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def save_motor_home(motor_ids=None, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000):
    """Read current position and save as HOME.
    
    Args:
        motor_ids: List of motor IDs to save, or None to save all active motors
        port: Serial port
        baudrate: Baud rate
    """
    
    # Motor configuration
    motor_names = {
        1: "base_right_wheel",
        2: "base_docking",
        3: "base_left_wheel",
        4: "leg_bub_right",
        5: "leg_hip_right",
        6: "leg_twist_right",
        7: "leg_knee_right",
        8: "leg_foot_right",
        9: "leg_gripper_right",
    }
    
    motor_models = {
        1: "sts3215", 2: "sts3215", 3: "sts3215",
        4: "sts3095", 5: "sts3095", 6: "sts3215",
        7: "sts3095", 8: "sts3215", 9: "sts3215",
    }
    
    # Default to all active motors if none specified
    if motor_ids is None:
        motor_ids = [1, 4, 5, 6, 7, 8]  # Currently active motors
    elif isinstance(motor_ids, int):
        motor_ids = [motor_ids]
    
    logger.info("="*80)
    logger.info(f"📝 SAVE MOTOR(S) {motor_ids} CURRENT POSITION AS HOME")
    logger.info("="*80 + "\n")
    
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors.motors_bus import Motor, MotorNormMode
    except ImportError as e:
        logger.error(f"✗ LeRobot not installed: {e}")
        return False
    
    logger.info(f"[1/4] Creating motor objects for {len(motor_ids)} motors...\n")
    
    # Create motors dict
    motors = {}
    for motor_id in motor_ids:
        motor_name = motor_names.get(motor_id, f"motor_{motor_id}")
        motor_model = motor_models.get(motor_id, "unknown")
        motors[motor_name] = Motor(
            id=motor_id,
            model=motor_model,
            norm_mode=MotorNormMode.RANGE_0_100
        )
        logger.info(f"  • Motor {motor_id} ({motor_name}, {motor_model})")
    logger.info("")
    logger.info(f"[2/4] Connecting to {port} @ {baudrate} baud...\n")
    
    try:
        bus = FeetechMotorsBus(
            port=port,
            motors=motors,
            protocol_version=0,
        )
        bus.connect(handshake=False)
        logger.info("✓ Connected\n")
    except Exception as e:
        logger.error(f"✗ Failed to connect: {e}")
        return False
    
    try:
        logger.info(f"[3/4] Reading current positions...\n")
        
        positions_read = {}
        
        for motor_id in motor_ids:
            motor_name = motor_names.get(motor_id, f"motor_{motor_id}")
            
            try:
                current_pos = bus.read("Present_Position", motor_name, normalize=False)
                positions_read[motor_name] = {
                    "id": motor_id,
                    "position": int(current_pos)
                }
                
                # Also read health info
                try:
                    voltage = bus.read("Present_Voltage", motor_name, normalize=False)
                    temp = bus.read("Present_Temperature", motor_name, normalize=False)
                    positions_read[motor_name]["voltage"] = voltage
                    positions_read[motor_name]["temperature"] = temp
                    logger.info(f"  ✓ Motor {motor_id} ({motor_name}): Pos={current_pos:.0f}, V={voltage:.1f}V, T={temp:.0f}°C")
                except:
                    logger.info(f"  ✓ Motor {motor_id} ({motor_name}): Pos={current_pos:.0f}")
                    
            except Exception as e:
                logger.error(f"  ✗ Motor {motor_id} ({motor_name}): Error - {e}")
                return False
        
        logger.info("")
        logger.info(f"[4/4] Saving to calibration file...\n")
        
        # Create calibration directory
        calib_dir = Path("config/calibration")
        calib_dir.mkdir(parents=True, exist_ok=True)
        
        # Load existing calibration or create new
        calib_file = calib_dir / "calibration.json"
        
        if calib_file.exists():
            with open(calib_file, "r") as f:
                calib_data = json.load(f)
            logger.info(f"  📂 Loading existing calibration from {calib_file}\n")
        else:
            calib_data = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "description": "Motor HOME positions calibration",
                "baudrate": baudrate,
                "port": port,
                "home_positions": {},
                "motor_ids": {},
                "motor_models": {},
            }
            logger.info(f"  📄 Creating new calibration file\n")
        
        # Update with current motor data
        calib_data["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
        
        for motor_name, data in positions_read.items():
            motor_id = data["id"]
            position = data["position"]
            calib_data["home_positions"][motor_name] = position
            calib_data["motor_ids"][motor_name] = motor_id
            calib_data["motor_models"][motor_name] = motor_models.get(motor_id, "unknown")
        
        # Save to JSON
        with open(calib_file, "w") as f:
            json.dump(calib_data, f, indent=2)
        
        logger.info(f"✓ Saved to {calib_file}\n")
        
        # Display calibration data
        logger.info("Current calibration content:")
        logger.info("-" * 80)
        with open(calib_file, "r") as f:
            content = f.read()
            print(content)
        logger.info("-" * 80 + "\n")
        
        logger.info("="*80)
        logger.info(f"✓ SUCCESS! {len(motor_ids)} motor(s) HOME saved!")
        logger.info("="*80 + "\n")
        
        logger.info("Summary:")
        for motor_name, data in positions_read.items():
            logger.info(f"  • {motor_name}: HOME Position = {data['position']}")
        logger.info("")
        
        logger.info("Next steps:")
        logger.info("  • Move all motors to HOME: python move_motors_home.py")
        logger.info("  • Read motor status: python read_motor_info.py\n")
        
        return True
        
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        try:
            bus.disconnect()
            logger.info("✓ Disconnected\n")
        except Exception as e:
            logger.debug(f"Disconnect note: {e}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Save motor current position as HOME calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Save all active motors at once (1, 4, 5, 6, 7, 8)
  python save_motor_home.py
  
  # Save specific motor
  python save_motor_home.py --motor 1
  
  # Save multiple specific motors
  python save_motor_home.py --motor 1 4 5 7 8
  
  # Save with custom port
  python save_motor_home.py --port /dev/ttyUSB0
        """
    )
    parser.add_argument("--motor", type=int, nargs="*", default=None,
                       help="Motor IDs to save (if empty, saves all active: 1,4,5,6,7,8)")
    parser.add_argument("--port", default="/dev/ttyACM0",
                       help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000,
                       help="Baud rate (default: 1000000)")
    args = parser.parse_args()
    
    # If no motor specified, use all active motors
    motor_ids = args.motor if args.motor else None
    
    # Validate motor IDs if specified
    if motor_ids:
        for motor_id in motor_ids:
            if not (1 <= motor_id <= 9):
                parser.error(f"Motor ID {motor_id} must be between 1 and 9")
    
    result = save_motor_home(motor_ids, args.port, args.baudrate)
    exit(0 if result else 1)


if __name__ == "__main__":
    main()
