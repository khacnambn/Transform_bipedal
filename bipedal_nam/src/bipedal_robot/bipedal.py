#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from typing import Any, Dict
import numpy as np

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


logger = logging.getLogger(__name__)


class BipedalConfig:
    """Configuration for Bipedal Robot"""
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000,
                 use_degrees: bool = False, disable_torque_on_disconnect: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.use_degrees = use_degrees
        self.disable_torque_on_disconnect = disable_torque_on_disconnect


class BipedalRobot:
    """
    Bipedal robot with 4 motors controlled via FeetechMotorsBus.
    Motor config: 4(sts3095), 5(sts3095), 7(sts3095), 8(sts3215)
    """

    def __init__(self, config: BipedalConfig = None):
        if config is None:
            config = BipedalConfig()
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                # base
                "base_right_wheel": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_docking": Motor(2, "sts3215", norm_mode_body),
                "base_left_wheel": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                # leg
                "leg_bub_right": Motor(4, "sts3095", norm_mode_body),
                "leg_hip_right": Motor(5, "sts3095", norm_mode_body),
                "leg_twist_right": Motor(6, "sts3215", norm_mode_body),
                "leg_knee_right": Motor(7, "sts3095", norm_mode_body),
                "leg_foot_right": Motor(8, "sts3215", norm_mode_body),
                "leg_gripper_right": Motor(9, "sts3215", norm_mode_body),
            },
        )
        self.base_motors = [motor for motor in self.bus.motors if motor.startswith("base")]
        self.leg_motors = [motor for motor in self.bus.motors if motor.startswith("leg")]

    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "bub_right.pos",
                "hip_right.pos",
                "twist_right.pos",
                "knee_right.pos",
                "foot_right.pos",
                "gripper_right.pos",
                "x.vel",
                "y.vel",
                "theta.vel",
            ),
            float,
        )

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected 

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            self.bus.connect()
        except Exception as e:
            # Bỏ qua lỗi firmware version mismatch
            if "firmware" in str(e).lower():
                logger.warning(f"⚠️  Firmware version mismatch (ignored): {e}")
                # Vẫn tiếp tục kết nối
                pass
            else:
                raise

    def configure(self):
        # Set-up arm actuators (position mode)
        # We assume that at connection time, arm is in a rest position,
        # and torque can be safely disabled to run calibration.
        self.bus.disable_torque()
        for name in self.leg_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus.write("P_Coefficient", name, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 32)

        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)

        self.bus.enable_torque()


    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        steps_per_deg = 4096.0 / 360.0
        speed_in_steps = degps * steps_per_deg
        speed_int = int(round(speed_in_steps))
        # Cap the value to fit within signed 16-bit range (-32768 to 32767)
        if speed_int > 0x7FFF:
            speed_int = 0x7FFF  # 32767 -> maximum positive value
        elif speed_int < -0x8000:
            speed_int = -0x8000  # -32768 -> minimum negative value
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        steps_per_deg = 4096.0 / 360.0
        magnitude = raw_speed
        degps = magnitude / steps_per_deg
        return degps

    def _body_to_wheel_raw(
        self,
        x: float,
        y: float,
        theta: float,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
        max_raw: int = 3000,
    ) -> dict:
        """
        Convert desired body-frame velocities into wheel raw commands.

        Parameters:
          x_cmd      : Linear velocity in x (m/s).
          y_cmd      : Linear velocity in y (m/s).
          theta_cmd  : Rotational velocity (deg/s).
          wheel_radius: Radius of each wheel (meters).
          base_radius : Distance from the center of rotation to each wheel (meters).
          max_raw    : Maximum allowed raw command (ticks) per wheel.

        Returns:
          A dictionary with wheel raw commands:
             {"base_left_wheel": value, "base_back_wheel": value, "base_right_wheel": value}.

        Notes:
          - Internally, the method converts theta_cmd to rad/s for the kinematics.
          - The raw command is computed from the wheels angular speed in deg/s
            using _degps_to_raw(). If any command exceeds max_raw, all commands
            are scaled down proportionally.
        """
        # Convert rotational velocity from deg/s to rad/s.
        theta_rad = theta * (np.pi / 180.0)
        # Create the body velocity vector [x, y, theta_rad].
        velocity_vector = np.array([x, y, theta_rad])

        # Define the wheel mounting angles with a -90° offset.
        angles = np.radians(np.array([240, 0, 120]) - 90)
        # Build the kinematic matrix: each row maps body velocities to a wheel’s linear speed.
        # The third column (base_radius) accounts for the effect of rotation.
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        # Compute each wheel’s linear speed (m/s) and then its angular speed (rad/s).
        wheel_linear_speeds = m.dot(velocity_vector)
        wheel_angular_speeds = wheel_linear_speeds / wheel_radius

        # Convert wheel angular speeds from rad/s to deg/s.
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

        # Scaling
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps = wheel_degps * scale

        # Convert each wheel’s angular speed (deg/s) to a raw integer.
        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]

        return {
            "base_left_wheel": wheel_raw[0],
            "base_back_wheel": wheel_raw[1],
            "base_right_wheel": wheel_raw[2],
        }

    def _wheel_raw_to_body(
        self,
        left_wheel_speed,
        back_wheel_speed,
        right_wheel_speed,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
    ) -> dict[str, Any]:
        """
        Convert wheel raw command feedback back into body-frame velocities.

        Parameters:
          wheel_raw   : Vector with raw wheel commands ("base_left_wheel", "base_back_wheel", "base_right_wheel").
          wheel_radius: Radius of each wheel (meters).
          base_radius : Distance from the robot center to each wheel (meters).

        Returns:
          A dict (x.vel, y.vel, theta.vel) all in m/s
        """

        # Convert each raw command back to an angular speed in deg/s.
        wheel_degps = np.array(
            [
                self._raw_to_degps(left_wheel_speed),
                self._raw_to_degps(back_wheel_speed),
                self._raw_to_degps(right_wheel_speed),
            ]
        )

        # Convert from deg/s to rad/s.
        wheel_radps = wheel_degps * (np.pi / 180.0)
        # Compute each wheel’s linear speed (m/s) from its angular speed.
        wheel_linear_speeds = wheel_radps * wheel_radius

        # Define the wheel mounting angles with a -90° offset.
        angles = np.radians(np.array([240, 0, 120]) - 90)
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        # Solve the inverse kinematics: body_velocity = M⁻¹ · wheel_linear_speeds.
        m_inv = np.linalg.inv(m)
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x, y, theta_rad = velocity_vector
        theta = theta_rad * (180.0 / np.pi)
        return {
            "x.vel": x,
            "y.vel": y,
            "theta.vel": theta,
        }  # m/s and deg/s

    def stop_base(self):
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)
        logger.info("Base motors stopped")

    def write_pos_ex(
        self,
        motor_name: str,
        position: int,
        speed: int = 1000,
        acceleration: int = 50,
        normalize: bool = False,
    ) -> None:
        """
        Giống SCServo WritePosEx: set position + speed + acceleration.
        
        Args:
            motor_name: Motor name (e.g., "leg_knee_right")
            position: Target position (raw ticks hoặc normalized value)
            speed: Goal velocity (raw value)
            acceleration: Acceleration (raw value)
            normalize: If True, convert position from normalized [-100, 100] to raw ticks
        """
        # 1) Set acceleration
        try:
            self.bus.write("Acceleration", motor_name, acceleration, normalize=False)
        except Exception as e:
            logger.warning(f"Failed to set acceleration: {e}")

        # 2) Set speed
        try:
            self.bus.write("Goal_Velocity", motor_name, speed, normalize=False)
        except Exception as e:
            logger.warning(f"Failed to set speed: {e}")

        # 3) Set position
        self.bus.write("Goal_Position", motor_name, position, normalize=normalize)

    def read_motor_position(self, motor_name: str, normalize: bool = False) -> int:
        """Read current position of motor"""
        try:
            return self.bus.read("Present_Position", motor_name, normalize=normalize)
        except Exception as e:
            logger.error(f"Failed to read position: {e}")
            return None

    def read_leg_positions(self, normalize: bool = False) -> Dict[str, int]:
        """Read all leg motor positions"""
        leg_motors = [
            "leg_bub_right", "leg_hip_right", "leg_twist_right",
            "leg_knee_right", "leg_foot_right"
        ]
        positions = {}
        for motor_name in leg_motors:
            if motor_name in self.bus.motors:
                positions[motor_name] = self.read_motor_position(motor_name, normalize)
        return positions

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_base()
        self.bus.disconnect(self.config.disable_torque_on_disconnect)

        logger.info(f"{self} disconnected.")
