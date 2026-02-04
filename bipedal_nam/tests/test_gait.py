# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import pytest

from bipedal_robot import BipedalConfig
from bipedal_robot.motor_control.gait_controller import GaitController


@pytest.fixture
def config():
    """Create test configuration."""
    return BipedalConfig()


@pytest.fixture
def gait(config):
    """Create gait controller."""
    return GaitController(config)


def test_gait_step_generation(gait):
    """Test gait step generation."""
    step = gait.compute_step(0, speed=1.0)
    
    assert "left" in step
    assert "right" in step
    assert len(step["left"]) == 3  # (x, y, z)


def test_alternating_legs(gait):
    """Test that legs alternate during walking."""
    step0 = gait.compute_step(0, speed=1.0)
    step1 = gait.compute_step(1, speed=1.0)
    
    # Step 0: left leg should swing, right leg should be in stance
    # Step 1: opposite
    assert step0["left"][2] != step1["left"][2] or step0["right"][2] != step1["right"][2]


def test_trajectory_generation(gait):
    """Test full trajectory generation."""
    trajectory = gait.compute_trajectory(duration=2.0, dt=0.01, speed=1.0)
    
    assert "left" in trajectory
    assert "right" in trajectory
    assert len(trajectory["left"]) > 0
    assert len(trajectory["right"]) > 0
    assert len(trajectory["left"]) == len(trajectory["right"])


def test_speed_variation(gait):
    """Test gait at different speeds."""
    traj_slow = gait.compute_trajectory(duration=1.0, speed=0.5)
    traj_fast = gait.compute_trajectory(duration=1.0, speed=2.0)
    
    # Both should have trajectories
    assert len(traj_slow["left"]) > 0
    assert len(traj_fast["left"]) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
