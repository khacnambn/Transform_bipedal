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


class GaitController:
    """
    Generates walking gaits for bipedal robot.
    
    Implements alternating leg walking pattern:
    - Stance phase: leg on ground
    - Swing phase: leg in air moving forward
    """
    
    def __init__(self, config):
        """
        Initialize gait controller.
        
        Args:
            config: BipedalConfig object
        """
        self.config = config
        self.phase = 0.0  # Current phase in gait cycle [0, 1]
        
        logger.info(
            f"Gait controller initialized: "
            f"step_length={config.step_length}m, "
            f"frequency={config.gait_frequency}Hz"
        )
    
    def compute_step(
        self, 
        step_num: int, 
        speed: float = 1.0
    ) -> Dict[str, Tuple[float, float, float]]:
        """
        Compute foot positions for a single step.
        
        Args:
            step_num: Step number in the walking sequence
            speed: Speed multiplier
            
        Returns:
            Dict of foot target positions {leg: (x, y, z)}
        """
        # Compute phase for this step
        phase = (step_num % 2) * 0.5  # 0.0 or 0.5 for alternating legs
        
        positions = {}
        
        # Left leg
        left_phase = phase
        positions["left"] = self._compute_leg_trajectory(left_phase, speed, "left")
        
        # Right leg (opposite phase)
        right_phase = (phase + 0.5) % 1.0
        positions["right"] = self._compute_leg_trajectory(right_phase, speed, "right")
        
        return positions
    
    def _compute_leg_trajectory(
        self, 
        phase: float, 
        speed: float = 1.0,
        leg: str = "left"
    ) -> Tuple[float, float, float]:
        """
        Compute trajectory for single leg during one gait cycle.
        
        Args:
            phase: Phase in gait cycle [0, 1]
            speed: Speed multiplier
            leg: "left" or "right"
            
        Returns:
            Target position (x, y, z)
        """
        step_length = self.config.step_length * speed
        walk_height = self.config.walk_height
        
        # Stance phase: 0.0 to 0.5 (leg on ground)
        # Swing phase: 0.5 to 1.0 (leg in air)
        
        if phase < 0.5:  # Stance phase
            # Leg gradually moves back (negative x)
            x = step_length * 0.5 * (1.0 - 2 * phase)
            y = 0.0
            z = -0.2  # On ground
        else:  # Swing phase
            # Leg lifted and moved forward
            swing_phase = (phase - 0.5) * 2  # Normalize to [0, 1]
            x = -step_length * 0.5 + step_length * swing_phase
            y = 0.0
            # Parabolic lift trajectory
            z = -0.2 + walk_height * (4 * swing_phase * (1 - swing_phase))
        
        # Side-to-side offset for left/right legs
        if leg == "left":
            y = -0.05
        else:
            y = 0.05
        
        return (x, y, z)
    
    def compute_trajectory(
        self, 
        duration: float, 
        dt: float = 0.01,
        speed: float = 1.0
    ) -> Dict[str, list]:
        """
        Generate full trajectory for specified duration.
        
        Args:
            duration: Total duration (seconds)
            dt: Time step (seconds)
            speed: Speed multiplier
            
        Returns:
            Dict of trajectories {leg: [(x, y, z), ...]}
        """
        num_steps = int(duration / dt)
        gait_period = 1.0 / self.config.gait_frequency
        
        trajectories = {"left": [], "right": []}
        
        for step in range(num_steps):
            time = step * dt
            phase = (time / gait_period) % 1.0
            
            # Left leg
            left_phase = phase
            left_pos = self._compute_leg_trajectory(left_phase, speed, "left")
            trajectories["left"].append(left_pos)
            
            # Right leg
            right_phase = (phase + 0.5) % 1.0
            right_pos = self._compute_leg_trajectory(right_phase, speed, "right")
            trajectories["right"].append(right_pos)
        
        logger.info(
            f"Generated trajectory for {duration}s at {speed}x speed "
            f"({num_steps} steps)"
        )
        
        return trajectories
    
    def get_phase_info(self) -> Dict[str, float]:
        """Get current phase information."""
        return {
            "gait_phase": self.phase,
            "frequency": self.config.gait_frequency,
            "step_length": self.config.step_length,
        }
