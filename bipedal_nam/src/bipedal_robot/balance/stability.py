# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import logging
from typing import Dict

logger = logging.getLogger(__name__)


class BalanceController:
    """
    Maintains balance and stability during walking.
    
    Implements:
    - Center of mass tracking
    - Foot placement adjustment
    - Compliance control
    """
    
    def __init__(self, config):
        """
        Initialize balance controller.
        
        Args:
            config: BipedalConfig object
        """
        self.config = config
        self.com_x = 0.0  # Center of mass X position
        self.com_y = 0.0  # Center of mass Y position
        
        logger.info("Balance controller initialized")
    
    def apply_stability_constraints(
        self, 
        target_positions: Dict[str, float]
    ) -> Dict[str, float]:
        """
        Apply stability constraints to target positions.
        
        Ensures:
        - Joint limits are respected
        - Movements are smooth
        - Robot stays within support polygon
        
        Args:
            target_positions: Desired joint positions
            
        Returns:
            Constrained joint positions
        """
        constrained = {}
        
        # Joint limits (in radians)
        joint_limits = {
            "hip": (-1.57, 1.57),      # ±90 degrees
            "knee": (0.0, 1.57),       # 0-90 degrees (no hyperextension)
            "ankle": (-0.785, 0.785),  # ±45 degrees
        }
        
        for name, value in target_positions.items():
            # Parse joint type
            joint_type = name.split("_")[1]  # "hip", "knee", or "ankle"
            
            if joint_type in joint_limits:
                min_limit, max_limit = joint_limits[joint_type]
                # Clamp to joint limits
                constrained[name] = max(min_limit, min(max_limit, value))
            else:
                constrained[name] = value
        
        return constrained
    
    def compute_center_of_mass(
        self, 
        joint_positions: Dict[str, float]
    ) -> tuple:
        """
        Estimate center of mass from joint positions.
        
        Args:
            joint_positions: Current joint angles
            
        Returns:
            (com_x, com_y) center of mass position
        """
        # Simplified COM estimation
        # Assumes equal mass distribution
        
        left_hip = joint_positions.get("left_hip", 0.0)
        right_hip = joint_positions.get("right_hip", 0.0)
        
        # COM at midpoint between hips
        com_x = (left_hip + right_hip) / 2.0
        com_y = 0.0
        
        return (com_x, com_y)
    
    def get_stability_margin(
        self, 
        joint_positions: Dict[str, float]
    ) -> float:
        """
        Compute stability margin (distance to tipping).
        
        Args:
            joint_positions: Current joint positions
            
        Returns:
            Stability margin in meters (higher = more stable)
        """
        # Simplified stability calculation
        com_x, com_y = self.compute_center_of_mass(joint_positions)
        
        # Assume support polygon is 0.2m x 0.3m
        support_width = 0.2
        support_length = 0.3
        
        # Distance to edge
        margin_x = support_width / 2.0 - abs(com_x)
        margin_y = support_length / 2.0 - abs(com_y)
        
        stability_margin = min(margin_x, margin_y)
        
        return max(0.0, stability_margin)
