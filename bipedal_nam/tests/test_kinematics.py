# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import pytest
import numpy as np

from bipedal_robot import BipedalConfig
from bipedal_robot.motor_control.kinematics import BipedalKinematics


@pytest.fixture
def config():
    """Create test configuration."""
    return BipedalConfig()


@pytest.fixture
def kinematics(config):
    """Create kinematics solver."""
    return BipedalKinematics(config)


def test_forward_kinematics_neutral(kinematics):
    """Test forward kinematics at neutral position."""
    angles = {
        "left_hip": 0.0,
        "left_knee": 0.0,
        "left_ankle": 0.0,
        "right_hip": 0.0,
        "right_knee": 0.0,
        "right_ankle": 0.0,
    }
    
    positions = kinematics.forward(angles)
    
    assert "left" in positions
    assert "right" in positions
    assert len(positions["left"]) == 3  # (x, y, z)


def test_inverse_kinematics_reachability(kinematics):
    """Test inverse kinematics."""
    positions = {
        "left": (0.2, -0.05, -0.2),
        "right": (0.2, 0.05, -0.2),
    }
    
    angles = kinematics.inverse(positions)
    
    assert len(angles) == 6
    assert "left_hip" in angles
    assert "left_knee" in angles
    assert "right_hip" in angles


def test_jacobian_computation(kinematics):
    """Test Jacobian matrix computation."""
    angles = {
        "left_hip": 0.1,
        "left_knee": 0.2,
    }
    
    jacobian = kinematics.compute_leg_jacobian(angles, "left")
    
    assert jacobian.shape == (3, 3)
    assert np.all(np.isfinite(jacobian))


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
