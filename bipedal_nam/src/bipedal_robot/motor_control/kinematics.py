# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import logging
import numpy as np
from typing import Dict, Tuple

logger = logging.getLogger(__name__)


class BipedalKinematics:
    """
    Kinematics solver for bipedal robot with 3 DOF per leg.
    
    Leg structure:
    - Thigh: hip to knee
    - Calf: knee to ankle
    - Total leg length: adjustable
    """
    
    def __init__(self, config):
        """
        Initialize kinematics solver.
        
        Args:
            config: BipedalConfig object
        """
        self.config = config
        
        # Leg dimensions (in meters)
        self.thigh_length = 0.2  # Hip to knee
        self.calf_length = 0.2   # Knee to ankle
        self.total_leg_length = self.thigh_length + self.calf_length
        
        logger.info(
            f"Kinematics initialized: "
            f"thigh={self.thigh_length}m, calf={self.calf_length}m"
        )
    
    def forward(self, angles: Dict[str, float]) -> Dict[str, Tuple[float, float, float]]:
        """
        Compute end-effector positions from joint angles (forward kinematics).
        
        Args:
            angles: Dict of joint angles {leg_joint: angle_rad}
            
        Returns:
            Dict of end-effector positions {leg: (x, y, z)}
        """
        positions = {}
        
        for leg in ["left", "right"]:
            hip_angle = angles.get(f"{leg}_hip", 0.0)
            knee_angle = angles.get(f"{leg}_knee", 0.0)
            ankle_angle = angles.get(f"{leg}_ankle", 0.0)
            
            # Forward kinematics calculation
            # Simplified 2D kinematics (x, y plane)
            knee_x = self.thigh_length * np.cos(hip_angle)
            knee_y = self.thigh_length * np.sin(hip_angle)
            
            ankle_x = knee_x + self.calf_length * np.cos(hip_angle + knee_angle)
            ankle_y = knee_y + self.calf_length * np.sin(hip_angle + knee_angle)
            
            # Add height variation from ankle joint
            ankle_z = -0.1 + 0.05 * np.sin(ankle_angle)
            
            positions[leg] = (ankle_x, ankle_y, ankle_z)
        
        return positions
    
    def inverse(self, foot_positions: Dict[str, Tuple[float, float, float]]) -> Dict[str, float]:
        """
        Compute joint angles from desired end-effector positions (inverse kinematics).
        
        Args:
            foot_positions: Dict of desired positions {leg: (x, y, z)}
            
        Returns:
            Dict of joint angles {leg_joint: angle_rad}
        """
        angles = {}
        
        for leg in ["left", "right"]:
            if leg not in foot_positions:
                # Use default position if not specified
                foot_positions[leg] = (0.2, 0.0, -0.2)
            
            x, y, z = foot_positions[leg]
            
            # 2D inverse kinematics for leg plane
            # Distance from hip to foot
            d = np.sqrt(x**2 + y**2)
            
            # Check if position is reachable
            if d > self.total_leg_length or d < abs(self.thigh_length - self.calf_length):
                logger.warning(f"Position out of reach for {leg} leg: d={d:.3f}m")
                # Clamp to reachable distance
                d = np.clip(d, abs(self.thigh_length - self.calf_length), self.total_leg_length)
            
            # Hip angle (direction to foot)
            hip_angle = np.arctan2(y, x)
            
            # Knee angle (using law of cosines)
            cos_knee = (d**2 - self.thigh_length**2 - self.calf_length**2) / (
                2 * self.thigh_length * self.calf_length
            )
            cos_knee = np.clip(cos_knee, -1.0, 1.0)
            knee_angle = np.arccos(cos_knee)
            
            # Ankle angle (compensate for height)
            ankle_angle = np.arcsin(np.clip(z * 10, -1.0, 1.0))
            
            angles[f"{leg}_hip"] = hip_angle
            angles[f"{leg}_knee"] = knee_angle
            angles[f"{leg}_ankle"] = ankle_angle
        
        return angles
    
    def compute_leg_jacobian(self, angles: Dict[str, float], leg: str = "left") -> np.ndarray:
        """
        Compute Jacobian matrix for leg (for velocity control).
        
        Args:
            angles: Joint angles
            leg: "left" or "right"
            
        Returns:
            3x3 Jacobian matrix
        """
        hip = angles.get(f"{leg}_hip", 0.0)
        knee = angles.get(f"{leg}_knee", 0.0)
        
        # Simplified 2D Jacobian
        j11 = -self.thigh_length * np.sin(hip) - self.calf_length * np.sin(hip + knee)
        j12 = -self.calf_length * np.sin(hip + knee)
        j13 = 0.0
        
        j21 = self.thigh_length * np.cos(hip) + self.calf_length * np.cos(hip + knee)
        j22 = self.calf_length * np.cos(hip + knee)
        j23 = 0.0
        
        j31 = 0.0
        j32 = 0.0
        j33 = 1.0
        
        return np.array([
            [j11, j12, j13],
            [j21, j22, j23],
            [j31, j32, j33],
        ])
