#!/usr/bin/env python

# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Configuration module for bipedal robot project.
"""

# Default configuration
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 115200

# Motor parameters
MOTOR_MODEL = "sts3215"
MOTOR_IDS = {
    "left_hip": 1,
    "left_knee": 2,
    "left_ankle": 3,
    "right_hip": 4,
    "right_knee": 5,
    "right_ankle": 6,
}

# Gait parameters
DEFAULT_STEP_LENGTH = 0.2  # meters
DEFAULT_GAIT_FREQUENCY = 2.0  # Hz
DEFAULT_WALK_HEIGHT = 0.1  # meters

# Safety parameters
JOINT_LIMITS = {
    "hip": (-1.57, 1.57),      # ±90 degrees
    "knee": (0.0, 1.57),       # 0-90 degrees
    "ankle": (-0.785, 0.785),  # ±45 degrees
}

# Leg dimensions
THIGH_LENGTH = 0.2  # meters
CALF_LENGTH = 0.2   # meters
