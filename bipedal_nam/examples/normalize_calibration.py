#!/usr/bin/env python
"""
Normalize calibration: Convert motor limits from steps (0-4096) to degrees [-180, 180]
This makes it easier to command motors with intuitive angle values.
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from lerobot.motors import MotorCalibration, MotorNormMode


def normalize_calibration(input_file: str, output_file: str = None):
    """
    Normalize calibration from steps to degrees.
    
    HOME position becomes 0 degrees in normalized space.
    Min/Max positions become negative/positive values in degrees [-180, 180].
    """
    
    with open(input_file, "r") as f:
        calib = json.load(f)
    
    home_positions = calib["home_positions"]
    motor_limits = calib["motor_limits"]
    
    normalized = {
        "home_positions": {},  # Will be 0 for all motors
        "motor_limits": {},
        "motor_info": {}
    }
    
    print("="*100)
    print("NORMALIZING CALIBRATION: Steps → Degrees")
    print("="*100 + "\n")
    
    print(f"{'Motor':<8} {'HOME':<10} {'Min (steps)':<15} {'Min (norm)':<15} {'Max (steps)':<15} {'Max (norm)':<15}")
    print("-"*100)
    
    for motor_id_str in sorted(motor_limits.keys(), key=int):
        motor_id = int(motor_id_str)
        home_steps = int(home_positions[motor_id_str])
        min_steps = motor_limits[motor_id_str]["min"]
        max_steps = motor_limits[motor_id_str]["max"]
        
        # HOME = 0 (normalized)
        normalized["home_positions"][motor_id_str] = 0
        
        # Calculate normalized limits
        # Normalize means: map [min_steps, max_steps] → [min_norm, max_norm]
        # where the range is symmetric around HOME = 0
        
        # Calculate distances from HOME
        min_dist = min_steps - home_steps  # Can be negative (wrap-around)
        max_dist = max_steps - home_steps
        
        # Handle wrap-around: normalize to [-2048, 2048]
        if min_dist > 2048:
            min_dist -= 4096
        if min_dist < -2048:
            min_dist += 4096
        
        if max_dist > 2048:
            max_dist -= 4096
        if max_dist < -2048:
            max_dist += 4096
        
        # Convert steps to degrees
        # 4096 steps = 360 degrees
        min_norm = (min_dist / 4096.0) * 360
        max_norm = (max_dist / 4096.0) * 360
        
        # Clamp to [-180, 180]
        min_norm = max(-180, min(180, min_norm))
        max_norm = max(-180, min(180, max_norm))
        
        # Ensure min < max for storage
        # if min_norm > max_norm:
        #     min_norm, max_norm = max_norm, min_norm
        
        normalized["motor_limits"][motor_id_str] = {
            "min": round(min_norm, 2),
            "max": round(max_norm, 2)
        }
        
        normalized["motor_info"][motor_id_str] = {
            "home_steps": home_steps,
            "min_steps": min_steps,
            "max_steps": max_steps,
            "min_norm": round(min_norm, 2),
            "max_norm": round(max_norm, 2),
            "range_norm": round(max_norm - min_norm, 2)
        }
        
        print(f"{motor_id:<8} {home_steps:<10} {min_steps:<15} {min_norm:<15.2f} {max_steps:<15} {max_norm:<15.2f}")
    
    print("-"*100 + "\n")
    
    # Save normalized calibration
    if output_file is None:
        output_file = input_file.replace(".json", "_normalized.json")
    
    with open(output_file, "w") as f:
        json.dump(normalized, f, indent=2)
    
    print(f"✓ Saved normalized calibration to: {output_file}\n")
    
    # Print summary
    print("="*100)
    print("NORMALIZED LIMITS IN DEGREES (HOME = 0°)")
    print("="*100)
    print(f"{'Motor':<8} {'Min (°)':<15} {'Max (°)':<15} {'Range (°)':<15} {'Usage':<40}")
    print("-"*100)
    
    for motor_id_str in sorted(motor_limits.keys(), key=int):
        info = normalized["motor_info"][motor_id_str]
        motor_id = int(motor_id_str)
        min_n = info["min_norm"]
        max_n = info["max_norm"]
        rng = info["range_norm"]
        
        usage = f"[{min_n:6.2f}, {max_n:6.2f}]"
        
        print(f"{motor_id:<8} {min_n:<15.2f} {max_n:<15.2f} {rng:<15.2f} {usage:<40}")
    
    print("-"*100)
    
    return normalized


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Normalize motor calibration")
    parser.add_argument("--input", default="config/calibration/rightcalib.json",
                       help="Input calibration file")
    parser.add_argument("--output", default=None,
                       help="Output calibration file (default: input_normalized.json)")
    args = parser.parse_args()
    
    normalize_calibration(args.input, args.output)