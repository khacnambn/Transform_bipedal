import json
import logging
import time
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bipedal_robot import BipedalConfig, BipedalRobot
from lerobot.motors import MotorCalibration, MotorNormMode


def load_calibration(calib_file: str = "config/calibration/rightcalib.json"):
    """Load calibration from normalized JSON file (degree-based, linear)."""
    try:
        with open(calib_file, "r") as f:
            data = json.load(f)
        
        motor_info = data.get("motor_info", {})
        
        # Convert to home_positions and motor_limits format
        home_positions = {}
        motor_limits = {}
        
        for motor_id_str, info in motor_info.items():
            motor_id = int(motor_id_str)
            home_positions[motor_id] = info.get("home_steps", 2048)
            
            # Store min/max in ticks (linear 0-4095)
            motor_limits[motor_id_str] = {
                "min": info.get("min_steps", 0),
                "max": info.get("max_steps", 4095),
                "home": info.get("home_steps", 2048)
            }
        
        return home_positions, motor_limits
    except Exception as e:
        logger.error(f"✗ Failed to load calibration: {e}")
        return {}, {}


def setup_calibration_in_bus(robot, home_positions, motor_limits):
    """
    Setup calibration in FeetechMotorsBus to handle normalization.
    This allows LeRobot to clamp positions correctly.
    """
    calibration = {}
    
    for motor_name, motor in robot.bus.motors.items():
        motor_id = motor.id
        motor_id_str = str(motor_id)
        
        if motor_id_str in motor_limits:
            limits = motor_limits[motor_id_str]
            range_min = limits.get("min", 0)
            range_max = limits.get("max", 4095)
        else:
            range_min = 0
            range_max = 4095
        
        # Create MotorCalibration object
        calibration[motor_name] = MotorCalibration(
            id=motor_id,
            drive_mode=0,
            homing_offset=0,
            range_min=range_min,
            range_max=range_max,
        )
    
    # Set calibration in bus
    robot.bus.calibration = calibration
    logger.info(f"✓ Setup calibration for {len(calibration)} motors")


def find_shortest_path(current_pos: int, target_pos: int, 
                       range_min: int, range_max: int) -> int:
    """
    Find shortest path to target considering wrap-around.
    
    For wrap-around motors (range_min > range_max):
    - Valid ranges: [range_min, 4096] OR [0, range_max]
    - Choose path with shortest distance
    
    Args:
        current_pos: Current position (0-4096)
        target_pos: Target position (0-4096)
        range_min: Min position limit
        range_max: Max position limit
    
    Returns:
        Target position (clamped to valid range)
    """
    
    # Normal case: range_min <= range_max
    if range_min <= range_max:
        # Clamp to [range_min, range_max]
        return max(range_min, min(range_max, target_pos))
    
    # Wrap-around case: range_min > range_max
    # Valid ranges: [range_min, 4096] or [0, range_max]


def move_motors_home(port: str = "/dev/ttyACM0", baudrate: int = 1_000_000, 
                     calib_file: str = "config/calibration/rightcalib.json",
                     p_coef: int = 16, d_coef: int = 16, confirm: bool = True):
    """
    Move all motors to HOME position using BipedalRobot with LeRobot calibration.
    
    Uses LeRobot's normalization to ensure motors move in correct direction
    and stay within limits.
    """
    
    logger.info("="*80)
    logger.info("🏠 MOVE MOTORS TO HOME POSITION (BIPEDAL ROBOT)")
    logger.info("="*80 + "\n")
    
    # Load calibration
    logger.info("[1/5] Loading calibration...")
    home_positions, motor_limits = load_calibration(calib_file)
    if not home_positions:
        return False
    logger.info(f"  ✓ Loaded {len(home_positions)} HOME positions")
    logger.info(f"  ✓ Loaded {len(motor_limits)} motor limits\n")
    
    # Create robot
    logger.info("[2/5] Creating BipedalRobot...")
    config = BipedalConfig(port=port, baudrate=baudrate)
    robot = BipedalRobot(config)
    logger.info("  ✓ BipedalRobot created\n")
    
    # Connect
    logger.info("[3/5] Connecting to motors...")
    try:
        # Bypass firmware check
        robot.bus.connect(handshake=False)
        robot.configure()
        logger.info("  ✓ Connected\n")
    except Exception as e:
        if "firmware" in str(e).lower():
            logger.warning(f"⚠️  Firmware mismatch (ignored)")
            try:
                robot.bus.connect(handshake=False)
                robot.configure()
                logger.info("  ✓ Connected\n")
            except Exception as e2:
                logger.error(f"✗ Failed to connect: {e2}")
                return False
        else:
            logger.error(f"✗ Failed to connect: {e}")
            return False
    
    try:
        # Setup calibration in bus
        logger.info("[4/5] Setting up LeRobot calibration...")
        setup_calibration_in_bus(robot, home_positions, motor_limits)
        logger.info("")
        
        # Read current positions
        logger.info("Current motor positions:")
        logger.info("-" * 80)
        logger.info(f"{'Motor ID':<10} {'Name':<20} {'Current':<12} {'HOME':<12} {'Min':<8} {'Max':<8}")
        logger.info("-" * 80)
        
        for motor_name in sorted(robot.bus.motors.keys()):
            try:
                current = robot.bus.read("Present_Position", motor_name, normalize=False)
                motor_id = robot.bus.motors[motor_name].id
                motor_id_str = str(motor_id)
                home = int(home_positions.get(motor_id, 2048))
                
                limits = motor_limits.get(motor_id_str, {})
                range_min = limits.get("min", 0)
                range_max = limits.get("max", 4095)
                
                print(f"{motor_id:<10} {motor_name:<20} {current:<12.0f} {home:<12} {range_min:<8} {range_max:<8}")
            except Exception as e:
                logger.warning(f"  ✗ Error reading {motor_name}: {e}")
        
        logger.info("-" * 80 + "\n")
        
        # Ask for confirmation
        if confirm:
            logger.info("❓ Ready to move all motors to HOME position?")
            logger.info(f"   P coefficient: {p_coef}")
            logger.info(f"   D coefficient: {d_coef}\n")
            while True:
                user_input = input("👉 Continue? [Y/N]: ").strip().lower()
                if user_input in ['y', 'yes', 'có', 'c']:
                    logger.info("\n✓ Starting movement...\n")
                    break
                elif user_input in ['n', 'no', 'không', 'k']:
                    logger.info("\n✗ Cancelled.\n")
                    return False
        
        # Move motors to HOME (linear calibration, no wrap-around)
        logger.info("[5/5] Moving motors to HOME...\n")
        logger.info("-" * 80)
        
        for motor_name in sorted(robot.bus.motors.keys()):
            try:
                motor_id = robot.bus.motors[motor_name].id
                motor_id_str = str(motor_id)
                home_pos = int(home_positions.get(motor_id, 2048))
                
                # Get motor limits
                limits = motor_limits.get(motor_id_str, {})
                range_min = limits.get("min", 0)
                range_max = limits.get("max", 4095)
                
                # Get current position
                current_pos = robot.bus.read("Present_Position", motor_name, normalize=False)
                
                # Clamp target to valid range (linear: no wrap-around)
                target_pos = max(range_min, min(range_max, home_pos))
                
                # Set P/I/D coefficients
                robot.bus.write("P_Coefficient", motor_name, p_coef)
                robot.bus.write("I_Coefficient", motor_name, 0)
                robot.bus.write("D_Coefficient", motor_name, d_coef)
                
                # Write target position
                robot.bus.write("Goal_Position", motor_name, target_pos, normalize=False)
                
                # Show direction
                if target_pos > current_pos:
                    direction = "→ CW"
                elif target_pos < current_pos:
                    direction = "← CCW"
                else:
                    direction = "= SAME"
                
                status = "✓" if abs(target_pos - home_pos) < 10 else "⚠️ "
                print(f"  {status} Motor {motor_id} ({motor_name}): {direction}")
                print(f"      Current: {current_pos} → Target: {target_pos} (HOME: {home_pos})")
                print(f"      Range: [{range_min}, {range_max}], PID: ({p_coef}, 0, {d_coef})")
            except Exception as e:
                logger.warning(f"  ✗ {motor_name}: {e}")
        
        logger.info("-" * 80 + "\n")
        
        # Wait and verify
        logger.info("⏳ Waiting 5 seconds for motors to reach HOME...\n")
        time.sleep(5)
        
        logger.info("Verifying final positions:")
        logger.info("-" * 80)
        logger.info(f"{'Motor ID':<10} {'Name':<20} {'Current':<12} {'HOME':<12} {'Status':<15}")
        logger.info("-" * 80)
        
        all_ok = True
        for motor_name in sorted(robot.bus.motors.keys()):
            try:
                current = robot.bus.read("Present_Position", motor_name, normalize=False)
                motor_id = robot.bus.motors[motor_name].id
                home = int(home_positions.get(motor_id, 2048))
                diff = abs(current - home)
                
                if diff < 50:
                    status = "✓ OK"
                elif diff < 100:
                    status = "⚠️  CLOSE"
                else:
                    status = "❌ FAR"
                    all_ok = False
                
                print(f"{motor_id:<10} {motor_name:<20} {current:<12.0f} {home:<12} {status:<15}")
            except Exception as e:
                logger.warning(f"  ✗ {motor_name}: {e}")
                all_ok = False
        
        logger.info("-" * 80 + "\n")
        
        if all_ok:
            logger.info("="*80)
            logger.info("✓ SUCCESS! All motors at HOME")
            logger.info("="*80 + "\n")
        else:
            logger.warning("⚠️  Some motors may not be at HOME yet.\n")
        
        return True
        
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        try:
            robot.disconnect()
            logger.info("✓ Disconnected\n")
        except:
            pass


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Move motors to HOME using LeRobot calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python move_motors_home.py --no-confirm
  python move_motors_home.py --d-coef 8 --no-confirm
  python move_motors_home.py --calib-file config/calibration/rightcalib.json --no-confirm
        """
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--calib-file", default="config/calibration/rightcalib.json", 
                        help="Path to rightcalib.json (degree-based, linear)")
    parser.add_argument("--p-coef", type=int, default=16)
    parser.add_argument("--d-coef", type=int, default=16)
    parser.add_argument("--no-confirm", action="store_true")
    args = parser.parse_args()
    
    result = move_motors_home(
        args.port, args.baudrate, args.calib_file,
        args.p_coef, args.d_coef, confirm=not args.no_confirm
    )
    exit(0 if result else 1)


if __name__ == "__main__":
    main()