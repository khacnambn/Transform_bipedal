#!/usr/bin/env python

# Copyright 2024 Nam. All rights reserved.

"""
Detect and find all motors using LeRobot FeetechMotorsBus.
Phát hiện và tìm tất cả các động cơ sử dụng LeRobot.
"""

import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def find_motors_lerobot(port: str = "/dev/ttyACM0", baudrate: int = 1_000_000):
    """Find all motors using LeRobot FeetechMotorsBus."""
    
    logger.info("="*80)
    logger.info("🔍 FINDING MOTORS USING LEROBOT FEETECHMOTORSBUS")
    logger.info("="*80 + "\n")
    
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
    except ImportError as e:
        logger.error(f"✗ LeRobot not installed: {e}")
        return False
    
    logger.info(f"[1/3] Scanning for motors on {port} @ {baudrate} baud...\n")
    
    try:
        # First, try to create a bus with a dummy motor to establish connection
        # This will fail, but we'll catch it and try again with scan_for_motors
        
        logger.info("Step 1: Initializing FeetechMotorsBus...")
        logger.info(f"  Port: {port}")
        logger.info(f"  Baudrate: {baudrate}\n")
        
        bus = FeetechMotorsBus(
            port=port,
            motors={},  # Empty, just to initialize
            protocol_version=0,
        )
        
        logger.info("Step 2: Scanning for motors (this may take a while)...\n")
        
        # Try to find motors using the internal scan method
        try:
            # Access the internal port handler to scan
            bus.set_baudrate(baudrate)
            bus.port_handler.openPort()
            
            logger.info("Scanning IDs 1-31...\n")
            logger.info("-" * 80)
            
            found_motors = {}
            packet_handler = bus.packet_handler
            
            for motor_id in range(1, 32):
                try:
                    # Try to ping each ID
                    model_number, comm, error = packet_handler.ping(bus.port_handler, motor_id)
                    
                    if comm == 0:  # COMM_SUCCESS
                        found_motors[motor_id] = model_number
                        logger.info(f"  ✓ Motor ID {motor_id:2d}: Model {model_number}")
                    
                except Exception as e:
                    # Motor not responding
                    pass
            
            logger.info("-" * 80 + "\n")
            
            if found_motors:
                logger.info(f"[2/3] Found {len(found_motors)} motors:\n")
                
                # Map model numbers to names
                model_map = {
                    777: "STS3215",
                    2569: "STS3095",
                }
                
                logger.info("-" * 80)
                logger.info(f"{'Motor ID':<12} {'Model Number':<15} {'Model Name':<15}")
                logger.info("-" * 80)
                
                motor_list = []
                for motor_id, model_num in sorted(found_motors.items()):
                    model_name = model_map.get(model_num, f"Unknown ({model_num})")
                    print(f"{motor_id:<12} {model_num:<15} {model_name:<15}")
                    motor_list.append((motor_id, model_name))
                
                logger.info("-" * 80 + "\n")
                
                logger.info("[3/3] Motor Configuration:\n")
                logger.info("Motor mapping (suggested):")
                logger.info("-" * 80)
                
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
                
                motor_config = {}
                for motor_id, model_name in motor_list:
                    name = motor_names.get(motor_id, f"motor_{motor_id}")
                    motor_config[name] = {"id": motor_id, "model": model_name}
                    print(f"  {name:<15} = Motor ID {motor_id} ({model_name})")
                
                logger.info("-" * 80 + "\n")
                
                logger.info("="*80)
                logger.info(f"✓ SUCCESS! Found {len(found_motors)} motors")
                logger.info("="*80 + "\n")
                
                return motor_config
            else:
                logger.warning("⚠️  No motors found!\n")
                return False
            
        finally:
            try:
                bus.port_handler.closePort()
            except:
                pass
        
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Find motors using LeRobot FeetechMotorsBus",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Scan with default port
  python find_motors_lerobot.py
  
  # Scan with custom port
  python find_motors_lerobot.py --port /dev/ttyUSB0
  
  # Scan with custom baudrate
  python find_motors_lerobot.py --baudrate 115200
        """
    )
    parser.add_argument("--port", default="/dev/ttyACM0",
                       help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000,
                       help="Baud rate (default: 1000000)")
    args = parser.parse_args()
    
    result = find_motors_lerobot(args.port, args.baudrate)
    
    if result:
        # Save to config file if requested
        logger.info("\nWould you like to save this configuration? (not yet implemented)")
    
    exit(0 if result else 1)


if __name__ == "__main__":
    main()
