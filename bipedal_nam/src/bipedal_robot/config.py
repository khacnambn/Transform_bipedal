# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from dataclasses import dataclass, field
from lerobot.robots.config import RobotConfig
from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig


def default_cameras() -> dict[str, CameraConfig]:
    """Default camera configuration for bipedal robot."""
    return {
        "front": OpenCVCameraConfig(
            index_or_path="/dev/video0",
            fps=30,
            width=640,
            height=480,
        ),
    }


@dataclass(kw_only=True)
class BipedalConfig(RobotConfig):
    """Configuration for bipedal walking robot."""
    
    # Hardware Configuration
    port: str = "/dev/ttyACM0"
    """Serial port for motor communication"""
    
    baudrate: int = 1000000
    """Serial port baud rate (default: 1000000 for Feetech STS3215)"""
    
    disable_torque_on_disconnect: bool = True
    """Disable motor torque when disconnecting"""
    
    # Robot Structure
    num_legs: int = 2
    """Number of legs (typically 2 for bipedal)"""
    
    joints_per_leg: int = 3
    """Number of joints per leg (Hip, Knee, Ankle)"""
    
    # IMU Sensor Configuration
    has_imu: bool = True
    """Whether robot has IMU sensor"""
    
    imu_port: str = "/dev/ttyUSB0"
    """Serial port for IMU communication"""
    
    # Motor Safety
    max_relative_target: float | dict[str, float] | None = None
    """Maximum relative target position for safety"""
    
    use_degrees: bool = False
    """Use degrees instead of radians"""
    
    # Camera Configuration
    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras)
    """Camera configurations"""
    
    # Gait Parameters
    walk_height: float = 0.1
    """Height of leg lift during walking (meters)"""
    
    step_length: float = 0.2
    """Length of each step (meters)"""
    
    gait_frequency: float = 2.0
    """Walking frequency (Hz)"""
    
    # Stability Parameters
    balance_enabled: bool = True
    """Enable balance control"""
    
    com_offset_x: float = 0.0
    """Center of mass offset X"""
    
    com_offset_y: float = 0.0
    """Center of mass offset Y"""
