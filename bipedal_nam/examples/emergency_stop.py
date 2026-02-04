#!/usr/bin/env python
"""
Emergency STOP - Turn off motor 8 immediately
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bipedal_robot import BipedalConfig, BipedalRobot

config = BipedalConfig(port="/dev/ttyACM0")
robot = BipedalRobot(config)

try:
    print("Connecting...")
    robot.bus.connect(handshake=False)
    
    print("🛑 EMERGENCY STOP - Disabling motor 8...")
    
    # Try to write Torque_Enable = 0 directly without num_retry
    try:
        import struct
        motor = robot.bus.motors["leg_foot_right"]
        motor_id = motor.id
        
        # Write torque disable directly
        robot.bus.packet_handler.write1ByteTxRx(
            robot.bus.port_handler,
            motor_id,
            robot.bus.PACKET_MAPPING["Torque_Enable"][0],  # Address
            0  # Disable torque
        )
        print("✓ Motor 8 disabled!")
        
    except Exception as e:
        print(f"⚠ Error: {e}")
    
    # Check if it worked
    import time
    time.sleep(0.5)
    
    pos = robot.bus.read("Present_Position", "leg_foot_right", normalize=False)
    current = robot.bus.read("Present_Current", "leg_foot_right", normalize=False)
    
    print(f"\nMotor 8 Status:")
    print(f"  Position: {pos} ticks")
    print(f"  Current: {current} mA")
    
finally:
    robot.disconnect()
    print("\n✓ Disconnected")
