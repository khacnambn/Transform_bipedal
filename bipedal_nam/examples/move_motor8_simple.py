#!/usr/bin/env python
"""
Move motor 8 to HOME - Simple version
Uses write_pos_ex with slow speed and low acceleration
No wrap-around complexity - just move and wait
"""

import json
import logging
import time
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bipedal_robot import BipedalConfig, BipedalRobot


def main():
    """Move motor 8 to HOME"""
    
    logger.info("="*80)
    logger.info("🏠 MOVE MOTOR 8 TO HOME - SIMPLE VERSION")
    logger.info("="*80 + "\n")
    
    # Load calibration
    logger.info("[1/3] Loading calibration...")
    try:
        with open("config/calibration/rightcalib_normalized.json", "r") as f:
            calib = json.load(f)
        motor_info = calib["motor_info"]
        info = motor_info["8"]
        home_ticks = info["home_steps"]
        
        logger.info(f"  ✓ Motor 8 HOME: {home_ticks} ticks\n")
    except Exception as e:
        logger.error(f"✗ Failed to load calibration: {e}")
        return False
    
    # Connect
    logger.info("[2/3] Connecting...")
    config = BipedalConfig(port="/dev/ttyACM0")
    robot = BipedalRobot(config)
    
    try:
        robot.bus.connect(handshake=False)
        robot.configure()
        logger.info("  ✓ Connected\n")
    except Exception as e:
        logger.error(f"✗ Failed to connect: {e}")
        return False
    
    try:
        # Read current position
        current_pos = robot.bus.read("Present_Position", "leg_foot_right", normalize=False)
        logger.info(f"[3/3] Moving to HOME")
        logger.info(f"  Current: {current_pos} ticks")
        logger.info(f"  Target:  {home_ticks} ticks")
        logger.info(f"  Distance: {abs(current_pos - home_ticks)} ticks\n")
        
        # Ask for confirmation
        logger.info("❓ Move motor 8 to HOME?")
        while True:
            user_input = input("👉 Continue? [Y/N]: ").strip().lower()
            if user_input in ['y', 'yes', 'có', 'c']:
                logger.info("\n✓ Starting movement...\n")
                break
            elif user_input in ['n', 'no', 'không', 'k']:
                logger.info("\n✗ Cancelled.\n")
                return False
        
        # Move with SLOW speed and LOW acceleration
        logger.info("📍 Moving with real-time tracking:\n")
        logger.info(f"{'Time (s)':<10} {'Position':<15} {'Distance':<15} {'Status':<15}")
        logger.info("-" * 55)
        
        # Send movement command with SLOW parameters
        robot.write_pos_ex(
            "leg_foot_right",
            position=home_ticks,
            speed=30,          # ← SLOW SPEED
            acceleration=2,    # ← LOW ACCELERATION
            normalize=False
        )
        
        start_time = time.time()
        prev_pos = None
        no_move_count = 0
        
        # Monitor position in real-time
        while time.time() - start_time < 12:  # Max 12 seconds
            elapsed = time.time() - start_time
            pos = robot.bus.read("Present_Position", "leg_foot_right", normalize=False)
            distance = abs(pos - home_ticks)
            
            # Check if motor stopped moving
            if prev_pos == pos:
                no_move_count += 1
            else:
                no_move_count = 0
            
            # Status
            if distance < 20:
                status = "✓ AT HOME!"
            elif distance < 50:
                status = "✓ VERY CLOSE"
            elif distance < 100:
                status = "✓ CLOSE"
            else:
                status = "⚠ MOVING"
            
            print(f"{elapsed:<10.1f} {pos:<15} {distance:<15} {status:<15}")
            
            # If motor stopped and close to target
            if no_move_count >= 2 and distance < 50:
                logger.info("-" * 55)
                logger.info("\n✓ Motor reached HOME and stopped!\n")
                break
            
            prev_pos = pos
            time.sleep(0.5)
        
        # Final check
        logger.info("📊 Final status:")
        final_pos = robot.bus.read("Present_Position", "leg_foot_right", normalize=False)
        final_error = abs(final_pos - home_ticks)
        
        logger.info(f"  Final position: {final_pos} ticks")
        logger.info(f"  HOME position:  {home_ticks} ticks")
        logger.info(f"  Error:          {final_error} ticks\n")
        
        if final_error <= 30:
            logger.info("="*80)
            logger.info("✅ SUCCESS! Motor 8 at HOME!")
            logger.info("="*80 + "\n")
            return True
        else:
            logger.warning(f"⚠️  Motor {final_error} ticks away from HOME\n")
            return False
        
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


if __name__ == "__main__":
    result = main()
    exit(0 if result else 1)
