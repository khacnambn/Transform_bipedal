#!/usr/bin/env python

# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Test configuration for bipedal robot (mock/simulation).

Use this when testing without actual hardware.
"""

import logging

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Test configuration
TEST_PORT = "/dev/null"  # Mock port
TEST_STEP_LENGTH = 0.15
TEST_GAIT_FREQUENCY = 1.5
TEST_WALK_HEIGHT = 0.08
