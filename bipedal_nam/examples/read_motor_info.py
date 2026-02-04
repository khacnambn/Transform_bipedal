#!/usr/bin/env python

# Copyright 2024 Nam. All rights reserved.

"""
Read motor status using LeRobot FeetechMotorsBus API.
Đọc trạng thái chi tiết về các motor: Position, Voltage, Temperature, v.v.
"""

import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def read_motor_info(port: str = "/dev/ttyACM0", baudrate: int = 1_000_000):
    """Read detailed motor information using LeRobot API."""
    
    logger.info("="*80)
    logger.info("📖 READING MOTOR INFORMATION USING LEROBOT API")
    logger.info("="*80 + "\n")
    
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors.motors_bus import Motor, MotorNormMode
    except ImportError as e:
        logger.error(f"✗ LeRobot not installed: {e}")
        return False
    
    # Define all motors using Motor objects
    motors = {
        "left_hip": Motor(id=1, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
        "left_knee": Motor(id=2, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
        "left_ankle": Motor(id=3, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
        "trunk_1": Motor(id=4, model="sts3095", norm_mode=MotorNormMode.RANGE_0_100),
        "trunk_2": Motor(id=5, model="sts3095", norm_mode=MotorNormMode.RANGE_0_100),
        "right_hip": Motor(id=6, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
        "trunk_3": Motor(id=7, model="sts3095", norm_mode=MotorNormMode.RANGE_0_100),
        "right_knee": Motor(id=8, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
        "right_ankle": Motor(id=9, model="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
    }
    
    logger.info(f"[1/3] Initializing FeetechMotorsBus...")
    logger.info(f"  Port: {port}")
    logger.info(f"  Baudrate: {baudrate}")
    logger.info(f"  Motors: {len(motors)}\n")
    
    try:
        bus = FeetechMotorsBus(
            port=port,
            motors=motors,
            protocol_version=0,
        )
    except Exception as e:
        logger.error(f"✗ Failed to initialize motor bus: {e}")
        return False
    
    logger.info("[2/3] Connecting to motors...")
    try:
        bus.connect(handshake=False)  # Skip firmware version check
        logger.info("  ✓ Connected\n")
    except Exception as e:
        logger.error(f"✗ Failed to connect: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    logger.info("[3/3] Reading motor information...\n")
    
    try:
        # Prepare data to read
        data_names = [
            "Present_Position",
            "Present_Velocity",
            "Present_Load",
            "Present_Voltage",
            "Present_Temperature",
            "Max_Torque",
            "Torque_Enable",
        ]
        
        logger.info("-" * 120)
        logger.info(f"{'Motor':<15} {'ID':<4} {'Pos':<6} {'Vel':<6} {'Load':<6} {'Volt':<6} {'Temp':<5} {'MaxTrq':<6} {'Torque':<6}")
        logger.info("-" * 120)
        
        for motor_name in sorted(motors.keys()):
            try:
                row_data = [motor_name, motors[motor_name].id]
                
                for data_name in data_names:
                    try:
                        # Read value without normalization (raw value)
                        value = bus.read(data_name, motor_name, normalize=False)
                        row_data.append(f"{value:.0f}")
                    except Exception as e:
                        logger.debug(f"  Could not read {data_name} from {motor_name}: {e}")
                        row_data.append("--")
                
                # Format output
                print(f"{row_data[0]:<15} {row_data[1]:<4} {row_data[2]:<6} {row_data[3]:<6} {row_data[4]:<6} {row_data[5]:<6} {row_data[6]:<5} {row_data[7]:<6} {row_data[8]:<6}")
                
            except Exception as e:
                logger.warning(f"  ✗ Error reading {motor_name}: {e}")
        
        logger.info("-" * 120 + "\n")
        
        # Read detailed info for each motor
        logger.info("\n" + "="*80)
        logger.info("DETAILED MOTOR INFORMATION")
        logger.info("="*80 + "\n")
        
        for motor_name in sorted(motors.keys()):
            try:
                motor_id = motors[motor_name].id
                model = motors[motor_name].model
                
                logger.info(f"Motor {motor_id}: {motor_name} ({model})")
                logger.info("-" * 80)
                
                info = {
                    "Present_Position": "Current position (0-4095)",
                    "Present_Velocity": "Current velocity",
                    "Present_Load": "Current load percentage",
                    "Present_Voltage": "Current voltage (V)",
                    "Present_Temperature": "Current temperature (°C)",
                    "Max_Torque": "Maximum torque limit",
                    "Torque_Enable": "Torque enabled (1=on, 0=off)",
                }
                
                for data_name, description in info.items():
                    try:
                        value = bus.read(data_name, motor_name, normalize=False)
                        print(f"  {data_name:<20} = {value:>8.1f}  ({description})")
                    except Exception as e:
                        print(f"  {data_name:<20} = ERROR    ({description})")
                
                print()
                
            except Exception as e:
                logger.warning(f"Error reading details for {motor_name}: {e}")
        
        logger.info("="*80 + "\n")
        
        logger.info("✓ SUCCESS! Motor information read complete\n")
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


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Read motor information using LeRobot FeetechMotorsBus",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Read with default port
  python read_motor_info.py
  
  # Read with custom port
  python read_motor_info.py --port /dev/ttyUSB0
  
  # Read with custom baudrate
  python read_motor_info.py --baudrate 115200
        """
    )
    parser.add_argument("--port", default="/dev/ttyACM0",
                       help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000,
                       help="Baud rate (default: 1000000)")
    args = parser.parse_args()
    
    result = read_motor_info(args.port, args.baudrate)
    exit(0 if result else 1)


if __name__ == "__main__":
    main()
