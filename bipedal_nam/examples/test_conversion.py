#!/usr/bin/env python
"""Verify new deg_to_ticks conversion"""

import json

with open("config/calibration/rightcalib_normalized.json") as f:
    calib = json.load(f)

motor_info = calib["motor_info"]

def deg_to_ticks(degrees: float, motor_id: int, motor_info: dict) -> int:
    """Convert degrees to raw ticks using calibration"""
    motor_id_str = str(motor_id)
    if motor_id_str not in motor_info:
        raise ValueError(f"Motor {motor_id} not in calibration!")
    
    info = motor_info[motor_id_str]
    home_steps = info["home_steps"]
    min_steps = info["min_steps"]
    max_steps = info["max_steps"]
    min_norm = info["min_norm"]
    max_norm = info["max_norm"]
    
    # ✅ CORRECT conversion: map degree range to tick range
    if min_steps <= max_steps:
        # Normal case: linear mapping
        degree_range = max_norm - min_norm
        tick_range = max_steps - min_steps
        
        if degree_range == 0:
            return int(home_steps)
        
        target_ticks = int(min_steps + (degrees - min_norm) / degree_range * tick_range)
    else:
        # Wrap-around case
        if degrees < 0:
            target_ticks = int(home_steps + (degrees / abs(min_norm)) * (home_steps - min_steps))
        else:
            target_ticks = int(home_steps + (degrees / max_norm) * (max_steps - home_steps))
        
        target_ticks = target_ticks % 4096
        if target_ticks < min_steps and target_ticks > max_steps:
            dist_to_min = min_steps - target_ticks
            dist_to_max = target_ticks - max_steps
            if dist_to_min < dist_to_max:
                target_ticks = min_steps
            else:
                target_ticks = max_steps
    
    target_ticks = max(0, min(4095, target_ticks))
    return target_ticks

print("="*100)
print("NEW CONVERSION TEST")
print("="*100 + "\n")

m4 = motor_info["4"]
print(f"Motor 4: {m4['min_norm']:.2f}° → {m4['max_norm']:.2f}° = {m4['min_steps']} → {m4['max_steps']} ticks\n")

tests = [
    ("0° (HOME)", 0.0),
    ("-20° (gait)", -20.0),
    ("-90° (mid)", -90.0),
    ("-179.74° (min limit)", -179.74),
    ("20° (should clamp)", 20.0),
]

for label, degrees in tests:
    ticks = deg_to_ticks(degrees, 4, motor_info)
    print(f"{label:25} → {ticks:4d} ticks", end="")
    
    # Verify: should be in [381, 2432]
    if m4['min_steps'] <= ticks <= m4['max_steps']:
        print(" ✓ VALID")
    else:
        print(" ❌ OUT OF RANGE")
