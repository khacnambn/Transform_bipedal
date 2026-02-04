# Bipedal Robot Control System

A modular bipedal robot control system built on top of the LeRobot framework by HuggingFace.

## Features

- **6 DOF Control**: 3 degrees of freedom per leg (Hip, Knee, Ankle)
- **Gait Generation**: Alternating leg walking pattern
- **Kinematics Solver**: Forward and inverse kinematics
- **Balance Control**: Stability constraints and COM tracking
- **Camera Integration**: Support for multiple cameras
- **Motor Calibration**: Automatic motor calibration and configuration
- **Degree-Based Tuning**: Write gait sequences in degrees, automatic conversion to motor ticks

## Project Structure

```
bipedal_nam/
├── src/bipedal_robot/
│   ├── config.py              # Configuration classes
│   ├── bipedal.py             # Main robot class
│   ├── motor_control/
│   │   ├── kinematics.py      # Forward/Inverse kinematics
│   │   └── gait_controller.py # Gait generation
│   ├── balance/
│   │   └── stability.py       # Balance control
│   ├── sensors/               # Sensor interfaces
│   └── utils/                 # Utility functions
├── tests/                     # Unit tests
├── examples/                  # Example scripts
├── config/                    # Configuration files
└── pyproject.toml            # Project configuration
```

## Installation

1. Install the package with dependencies:

```bash
cd /home/nam/Lekiwi_new/bipedal_nam
pip install -e .
```

2. Install development dependencies (optional):

```bash
pip install -e ".[dev]"
```

## Quick Start

### Basic Robot Control

```python
from bipedal_robot import Bipedal, BipedalConfig

# Create configuration
config = BipedalConfig(
    port="/dev/ttyACM0",
    has_imu=True,
)

# Initialize robot
robot = Bipedal(config)

# Connect and calibrate
robot.connect(calibrate=True)

# Walk
robot.walk(steps=5, speed=1.0)

# Disconnect
robot.disconnect()
```

### Custom Configuration

```python
from bipedal_robot import BipedalConfig

config = BipedalConfig(
    port="/dev/ttyACM0",
    step_length=0.25,
    gait_frequency=2.5,
    walk_height=0.12,
    balance_enabled=True,
)
```

## Motor Configuration

Default motor setup (can be customized in `bipedal.py`):

- **Left Leg**:
  - Motor 1: Hip
  - Motor 2: Knee
  - Motor 3: Ankle

- **Right Leg**:
  - Motor 4: Hip
  - Motor 5: Knee
  - Motor 6: Ankle

Motor Type: FeetechSTS3215 (via FeetechMotorsBus)

## Control Modes

### Standing Position
```python
robot.stand()
```

### Walking
```python
# Walk 10 steps at 1.5x speed
robot.walk(steps=10, speed=1.5)
```

### Direct Joint Control
```python
# Send direct joint angles
action = {
    "target_positions": {
        "left_hip": 0.5,
        "left_knee": 0.3,
        "left_ankle": 0.1,
        "right_hip": -0.5,
        "right_knee": 0.3,
        "right_ankle": 0.1,
    }
}
robot.send_action(action)
```

### Get Current State
```python
# Read observations from sensors
obs = robot.get_observation()
print(obs["joint_positions"])
print(obs["joint_velocities"])
```

## Configuration Parameters

Key configuration parameters in `BipedalConfig`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyACM0` | Serial port for motor communication |
| `has_imu` | `True` | Enable IMU sensor |
| `step_length` | `0.2` | Length of each step (meters) |
| `gait_frequency` | `2.0` | Walking frequency (Hz) |
| `walk_height` | `0.1` | Height of leg lift (meters) |
| `balance_enabled` | `True` | Enable balance control |

## Testing

Run unit tests:

```bash
pytest tests/
```

Run with coverage:

```bash
pytest --cov=src/bipedal_robot tests/
```

## Documentation

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [Motor Documentation](../lerobot/docs/source/feetech.mdx)

## Hardware Requirements

- 6x FeetechSTS3215 servo motors (or compatible)
- Serial communication interface (USB/UART)
- Optional: IMU sensor for balance feedback
- Optional: Camera(s) for vision

## Troubleshooting

### Motor Connection Issues
- Check serial port: `ls /dev/tty*`
- Verify motor IDs are unique
- Check motor power supply

### Calibration Errors
- Ensure all motors are in neutral position before calibration
- Check motor torque limits
- Verify calibration file is writable

## Contributing

Contributions welcome! Please follow the project coding standards.

## License

Apache License 2.0 - See LICENSE file

## Author

Nam

## References

- LeRobot Framework: https://github.com/huggingface/lerobot
- FeetechSTS3215 Documentation
