#!/usr/bin/env python

# Copyright 2024 Nam. All rights reserved.

"""
Motor control using LeRobot FeetechMotorsBus API.
Sử dụng LeRobot Motor Bus để điều khiển motor một cách đơn giản và nhất quan.
"""

import argparse
import json
import logging
from pathlib import Path

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def load_motor_config(config_file: str = "config/default_config.py"):
    """Load motor configuration."""
    
    config_module = {}
    with open(config_file) as f:
        exec(f.read(), config_module)
    
    return config_module


def load_calibration(calib_file: str = "config/calibration/calibration.json"):
    """Load calibration data."""
    
    try:
        with open(calib_file, "r") as f:
            data = json.load(f)
        return data.get("home_positions", {}), data.get("motor_ids", {})
    except Exception as e:
        logger.error(f"✗ Failed to load calibration: {e}")
        return None, None


def main(args):
    """Main function using LeRobot FeetechMotorsBus."""
    
    logger.info("="*80)
    logger.info("🤖 LEROBOT FEETECH MOTOR BUS CONTROL")
    logger.info("="*80 + "\n")
    
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
    except ImportError:
        logger.error("✗ LeRobot not installed or FeetechMotorsBus not available!")
        return False
    
    # Load calibration
    logger.info("[1/5] Loading calibration...")
    home_positions, motor_ids = load_calibration()
    if home_positions is None:
        return False
    logger.info(f"  ✓ Loaded {len(home_positions)} motor positions\n")
    
    # Create motor definitions
    logger.info("[2/5] Creating motor definitions...")
    
    motors = {
        "left_hip": {"id": 1, "model": "STS3215"},
        "left_knee": {"id": 2, "model": "STS3215"},
        "left_ankle": {"id": 3, "model": "STS3215"},
        "trunk_1": {"id": 4, "model": "STS3095"},
        "trunk_2": {"id": 5, "model": "STS3095"},
        "right_hip": {"id": 6, "model": "STS3215"},
        "trunk_3": {"id": 7, "model": "STS3095"},
        "right_knee": {"id": 8, "model": "STS3215"},
        "right_ankle": {"id": 9, "model": "STS3215"},
    }
    
    logger.info(f"  ✓ Created {len(motors)} motor definitions\n")
    
    # Initialize motor bus
    logger.info("[3/5] Initializing FeetechMotorsBus...")
    logger.info(f"  Port: {args.port}")
    logger.info(f"  Baudrate: {args.baudrate}\n")
    
    try:
        bus = FeetechMotorsBus(
            port=args.port,
            motors=motors,
            protocol_version=0,
        )
    except Exception as e:
        logger.error(f"✗ Failed to initialize motor bus: {e}")
        return False
    
    # Connect
    logger.info("[4/5] Connecting to motors...")
    try:
        bus.connect()
        logger.info("  ✓ Connected\n")
    except Exception as e:
        logger.error(f"✗ Failed to connect: {e}")
        return False
    
    # Read and display current positions
    logger.info("[5/5] Reading current positions...\n")
    logger.info("-" * 80)
    logger.info(f"{'Motor ID':<10} {'Name':<15} {'Current':<12} {'HOME':<12} {'Difference':<12}")
    logger.info("-" * 80)
    
    try:
        current_positions = {}
        
        for motor_name in motors.keys():
            try:
                # Read current position (normalized)
                current = bus.read("Present_Position", motor_name, normalize=False)
                current_positions[motor_name] = current
                
                motor_id = motors[motor_name]["id"]
                home = home_positions.get(motor_name, 0)
                diff = abs(current - home)
                
                print(f"{motor_id:<10} {motor_name:<15} {current:<12.0f} {home:<12} {diff:<12.0f}")
                
            except Exception as e:
                logger.warning(f"  ✗ {motor_name}: {e}")
        
        logger.info("-" * 80 + "\n")
        
        # Ask for user confirmation
        logger.info("❓ Check positions above. Are they correct?")
        while True:
            user_input = input("👉 Continue to HOME? [Y/N]: ").strip().lower()
            
            if user_input in ['y', 'yes', 'có', 'c']:
                logger.info("\n✓ Moving to HOME...\n")
                break
            elif user_input in ['n', 'no', 'không', 'k']:
                logger.info("\n✗ Cancelled.\n")
                bus.disconnect()
                return False
            else:
                print("❌ Please enter Y or N")
        
        # Move all motors to HOME
        logger.info("-" * 80)
        logger.info("Moving motors to HOME:")
        logger.info("-" * 80 + "\n")
        
        for motor_name, home_pos in home_positions.items():
            try:
                # Write to Goal_Position (normalized)
                bus.write("Goal_Position", motor_name, home_pos, normalize=False)
                motor_id = motors[motor_name]["id"]
                print(f"  ✓ Motor {motor_id} ({motor_name}): Moving to {home_pos}")
                
            except Exception as e:
                logger.warning(f"  ✗ {motor_name}: {e}")
        
        logger.info("\n" + "-" * 80)
        logger.info("Verifying final positions (waiting 3 seconds)...\n")
        
        import time
        time.sleep(3)
        
        logger.info("-" * 80)
        logger.info(f"{'Motor ID':<10} {'Name':<15} {'Current':<12} {'HOME':<12} {'Status':<12}")
        logger.info("-" * 80)
        
        all_ok = True
        for motor_name in motors.keys():
            try:
                current = bus.read("Present_Position", motor_name, normalize=False)
                motor_id = motors[motor_name]["id"]
                home = home_positions.get(motor_name, 0)
                diff = abs(current - home)
                
                if diff < 50:
                    status = "✓ OK"
                else:
                    status = "⚠️  WARNING"
                    all_ok = False
                
                print(f"{motor_id:<10} {motor_name:<15} {current:<12.0f} {home:<12} {status:<12}")
                
            except Exception as e:
                logger.warning(f"  ✗ {motor_name}: {e}")
                all_ok = False
        
        logger.info("-" * 80 + "\n")
        
        if all_ok:
            logger.info("="*80)
            logger.info("✓ SUCCESS! All motors at HOME")
            logger.info("="*80 + "\n")
        else:
            logger.warning("⚠️  Some motors may not be at HOME yet.\n")
        
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
        except:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Motor control using LeRobot FeetechMotorsBus",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Move all motors to HOME
  python lerobot_motor_control.py
  
  # Use custom port
  python lerobot_motor_control.py --port /dev/ttyUSB0
        """
    )
    parser.add_argument("--port", default="/dev/ttyACM0", 
                       help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000, 
                       help="Baud rate (default: 1000000)")
    args = parser.parse_args()
    
    success = main(args)
    exit(0 if success else 1)
