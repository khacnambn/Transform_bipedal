#!/usr/bin/env python
"""
Debug script: Check degree to ticks conversion for all motors
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def deg_to_ticks(degrees: float, motor_id: int, motor_info: dict) -> int:
    """Convert degrees to raw ticks using calibration with wrap-around handling"""
    motor_id_str = str(motor_id)
    if motor_id_str not in motor_info:
        raise ValueError(f"Motor {motor_id} not in calibration!")
    
    info = motor_info[motor_id_str]
    home_steps = info["home_steps"]
    
    # Convert degrees to ticks offset
    ticks_offset = (degrees / 360.0) * 4096
    target_ticks = home_steps + ticks_offset
    
    # Handle wrap-around: normalize to [-2048, 2048] range first
    if target_ticks > 2048:
        target_ticks -= 4096
    if target_ticks < -2048:
        target_ticks += 4096
    
    # Convert negative to 0-4095 range
    if target_ticks < 0:
        target_ticks += 4096
    
    return int(target_ticks)


def test_conversions():
    """Test degree to ticks conversion"""
    
    with open("config/calibration/rightcalib_normalized.json", "r") as f:
        calib = json.load(f)
    
    motor_info = calib["motor_info"]
    
    print("="*100)
    print("DEGREE → TICKS CONVERSION TEST")
    print("="*100 + "\n")
    
    test_degrees = [-45, -20, 0, 20, 45]
    
    for motor_id_str in sorted(motor_info.keys(), key=int):
        motor_id = int(motor_id_str)
        info = motor_info[motor_id_str]
        
        print(f"Motor {motor_id} (home_steps={info['home_steps']}):")
        print(f"  Limits: [{info['min_norm']:.2f}°, {info['max_norm']:.2f}°]")
        print(f"  Min/Max ticks: [{info['min_steps']}, {info['max_steps']}]")
        print("  Conversions:")
        
        for deg in test_degrees:
            try:
                ticks = deg_to_ticks(deg, motor_id, motor_info)
                print(f"    {deg:6.1f}° → {ticks:5d} ticks")
            except Exception as e:
                print(f"    {deg:6.1f}° → ERROR: {e}")
        
        print()
    
    print("="*100)
    print("✓ Test completed - check for any 0→4096 jumps or values outside [0, 4095]")
    print("="*100)


if __name__ == "__main__":
    test_conversions()
