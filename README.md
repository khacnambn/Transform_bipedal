# Transform_bipedal

URDF and USD files → Namtransformer

This project combines two mobile manipulators into a single bipedal robot system with distributed control architecture using ZMQ communication.

## System Architecture

The system consists of:
- **2 Mobile Manipulators** (Left & Right legs) - Each with dedicated edge computing
- **Edge Server** (on each mobile manipulator) - Reads IMU and servo motor data, publishes via ZMQ
- **Main Control Laptop** - Receives sensor data, processes policy, sends control commands via ZMQ client

![Control_diagram](Communicatio_Diagram.png) 

## Directory Structure

- **`bipedal_nam/`** - Main module for bipedal robot control and policies
- **`lerobot/`** - LeRobot framework for robot learning and inference
- **`examples_client/`** - Client examples for main laptop control

## Installation Guide

### 1. Clone repository
```bash
git clone https://github.com/khacnambn/Transform_bipedal.git
cd Transform_bipedal
```

### 2. Install dependencies

**For Ubuntu (Main Laptop):**
```bash
pip install -r lerobot/requirements-ubuntu.txt
pip install -e bipedal_nam/
pip install -e lerobot/
pip install zmq  # For ZMQ communication
```
**For Edge Servers (on each mobile manipulator - Raspberry Pi5/4):**
```bash
pip install zmq
pip install -e bipedal_nam/
```

## How to Run

### Step 1: Start Edge Servers (on each mobile manipulator)

**Left Leg Server:**
```bash
cd bipedal_nam/src/leg_server
python leg_server_left.py
```

**Right Leg Server:**
```bash
cd bipedal_nam/src/leg_server
python leg_server_right.py
```

### Step 2: Run Main Control Client (on laptop)

**Simple Walk:**
```bash
cd examples_client
python simple_walk_client.py
```

**Gait Controller:**
```bash
cd examples_client
python walking_gait.py
```

**Left Gait:**
```bash
cd examples_client
python walking_gait_left.py
```

**Dual Gait (Both legs):**
```bash
cd examples_client
python walking_gait_dual.py
```

**LeRobot Motor Control:**
```bash
cd bipedal_nam/examples
python lerobot_motor_control.py
```

## Running Unit Tests

**Test Gait:**
```bash
cd bipedal_nam
python -m pytest tests/test_gait.py -v
```

**Test Kinematics:**
```bash
cd bipedal_nam
python -m pytest tests/test_kinematics.py -v
```

**Test LeRobot:**
```bash
cd lerobot
python -m pytest tests/ -v
```

## Configuration

Edit configuration files:
```bash
# Default configuration
bipedal_nam/config/default_config.py

# Motor calibration
bipedal_nam/config/calibration/calibration.json
```

## Important Files

- `bipedal_nam/src/bipedal_robot/bipedal.py` - Main robot control class
- `bipedal_nam/src/bipedal_robot/motor_control/gait_controller.py` - Gait controller
- `bipedal_nam/src/bipedal_robot/motor_control/kinematics.py` - Kinematics solver
- `bipedal_nam/src/leg_server/leg_server_left.py` - Left leg ZMQ server
- `bipedal_nam/src/leg_server/leg_server_right.py` - Right leg ZMQ server
- `lerobot/src/lerobot/` - LeRobot framework

## Troubleshooting

If you encounter import errors:
```bash
pip install -e . --no-deps
python -m pip install --upgrade pip
```

If ZMQ connection fails:
```bash
# Check if servers are running
lsof -i :5555  # Left leg server
lsof -i :5556  # Right leg server
```

## License

MIT License
