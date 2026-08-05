import json
import time
import numpy as np
from pathlib import Path
import sys
import math

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.bipedal_robot.transformer import TransformerAPI


def load_trajectory(json_file: str) -> dict:
    """Load motion sequence từ JSON"""
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    print(f"📊 Motion loaded:")
    print(f"   File: {json_file}")
    print(f"   Task: {data['metadata']['task']}")
    print(f"   Total steps available: {data['metadata']['total_steps']}")
    print(f"   dt: {data['metadata']['dt_s']*1000:.1f}ms\n")
    
    return data


def replay_first_steps_with_imu_log(robot: TransformerAPI, motion_data: dict, 
                                     num_steps: int = 100, speed_factor: float = 1.0,
                                     log_file: str = None):
    """
    ✅ NEW: Replay motion + Log IMU data để so sánh với training
    
    Args:
        robot: TransformerAPI instance
        motion_data: Motion data from JSON
        num_steps: Number of steps to replay
        speed_factor: Speed multiplier (1.0=normal)
        log_file: Output CSV file for IMU data
    """
    
    steps = motion_data["trajectory"]
    metadata = motion_data["metadata"]
    dt = metadata["dt_s"]
    
    num_steps = min(num_steps, len(steps))
    
    # ✅ THÊM: Prepare logging
    if log_file is None:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_file = f"/home/nam/Transform_Bipedal/imu_logs/replay_{timestamp}.csv"
    
    log_path = Path(log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    
    imu_log = []  # List to store IMU samples
    
    print(f"🎬 MOTION REPLAY WITH IMU LOGGING")
    print(f"{'='*90}")
    print(f"   Total steps available: {len(steps)}")
    print(f"   Steps to replay: {num_steps}")
    print(f"   Duration: {num_steps * dt * speed_factor:.2f}s")
    print(f"   Speed factor: {speed_factor}x")
    print(f"   📝 IMU log: {log_file}")
    print(f"{'='*90}\n")
    
    try:
        t_start = time.time()
        t_sim_start = steps[0]["time_s"]
        
        for idx in range(num_steps):
            step_data = steps[idx]
            
            # Timestamp sync
            t_sim = step_data["time_s"]
            t_elapsed_sim = t_sim - t_sim_start
            t_elapsed_real = time.time() - t_start
            t_expected = t_elapsed_sim / speed_factor
            t_delta = t_expected - t_elapsed_real
            
            if t_delta > 0.001:
                time.sleep(t_delta)
            
            # Extract angles
            angles = step_data["angles_deg"]
            hip_l, hip_r = angles[0], angles[1]
            knee_l, knee_r = angles[2], angles[3]
            ankle_l, ankle_r = angles[4], angles[5]
            
            # ✅ NEW: Get real-time IMU from robot
            state = robot.get_state_with_retry(max_retries=1)
            
            if state:
                roll, pitch, yaw = state["orient"]
                gyro = state.get("gyro", [0.0, 0.0, 0.0])
            else:
                roll, pitch, yaw = 0.0, 0.0, 0.0
                gyro = [0.0, 0.0, 0.0]
            
            # Apply motion
            success = robot.set_joint_angles(
                hip_l, knee_l, ankle_l,
                hip_r, knee_r, ankle_r
            )
            
            if not success:
                print(f"  ❌ Step {idx}: Failed to apply joint angles")
                break
            
            # ✅ THÊM: Log IMU data to memory
            imu_log.append({
                "step": idx,
                "time_s": t_sim,
                "roll_rad": roll,
                "pitch_rad": pitch,
                "yaw_rad": yaw,
                "gx_rad_s": gyro[0],
                "gy_rad_s": gyro[1],
                "gz_rad_s": gyro[2],
                "hip_l_deg": hip_l,
                "hip_r_deg": hip_r,
                "knee_l_deg": knee_l,
                "knee_r_deg": knee_r,
            })
            
            # Log every 10 steps
            if (idx + 1) % 10 == 0:
                sync_error = (t_expected - t_elapsed_real) * 1000
                print(f"Step {idx+1:3d} | "
                      f"Motion: hip_l={hip_l:+6.1f}° knee_l={knee_l:+6.1f}° | "
                      f"IMU: pitch={pitch:+.4f}rad gx={gyro[0]:+.4f}rad/s | "
                      f"Sync: {sync_error:+6.1f}ms")
        
        # ✅ THÊM: Write IMU log to CSV
        print(f"\n📝 Writing IMU data to CSV...")
        _write_imu_csv(log_path, imu_log)
        print(f"✅ IMU log saved: {log_file}")
        
        # ✅ THÊM: Print statistics
        print(f"\n📊 IMU STATISTICS:")
        _print_imu_statistics(imu_log)
        
        print(f"\n✅ Replay complete ({num_steps} steps)!")
        
    except KeyboardInterrupt:
        print(f"\n👋 Interrupted by user at step {idx}")
    
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()


def _write_imu_csv(log_path: Path, imu_log: list):
    """Write IMU data to CSV"""
    import csv
    
    with open(log_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=imu_log[0].keys())
        writer.writeheader()
        writer.writerows(imu_log)


def _print_imu_statistics(imu_log: list):
    """Print IMU statistics"""
    if not imu_log:
        print("No IMU data!")
        return
    
    # Extract columns
    pitches = [s["pitch_rad"] for s in imu_log]
    gx_vals = [s["gx_rad_s"] for s in imu_log]
    gy_vals = [s["gy_rad_s"] for s in imu_log]
    
    print(f"\n  Pitch (rad):")
    print(f"    Min: {min(pitches):+.4f}rad ({min(pitches)*180/math.pi:+.2f}°)")
    print(f"    Max: {max(pitches):+.4f}rad ({max(pitches)*180/math.pi:+.2f}°)")
    print(f"    Range: {(max(pitches)-min(pitches))*1000:.1f}mrad ({(max(pitches)-min(pitches))*180/math.pi:.2f}°)")
    print(f"    Mean: {np.mean(pitches):+.4f}rad")
    print(f"    Std: {np.std(pitches):.4f}rad")
    
    print(f"\n  Gyro GX (rad/s):")
    print(f"    Min: {min(gx_vals):+.4f}rad/s")
    print(f"    Max: {max(gx_vals):+.4f}rad/s")
    print(f"    Range: {max(gx_vals)-min(gx_vals):.4f}rad/s")
    print(f"    Mean: {np.mean(gx_vals):+.4f}rad/s")
    print(f"    Std: {np.std(gx_vals):.4f}rad/s")
    
    print(f"\n  Gyro GY (rad/s):")
    print(f"    Min: {min(gy_vals):+.4f}rad/s")
    print(f"    Max: {max(gy_vals):+.4f}rad/s")
    print(f"    Range: {max(gy_vals)-min(gy_vals):.4f}rad/s")
    print(f"    Mean: {np.mean(gy_vals):+.4f}rad/s")
    print(f"    Std: {np.std(gy_vals):.4f}rad/s")


def main():
    """Main: Initialize robot, then replay first N steps with IMU logging"""
    
    print("=" * 90)
    print("🚶 BIPEDAL ROBOT - REPLAY MOTION WITH IMU LOGGING")
    print("=" * 90)
    
    print("\n[1/3] Initializing Robot...")
    robot = TransformerAPI(
        left_host="mobile2.local",
        left_port=5556,
        right_host="mobile1.local",
        right_port=5555
    )
    
    if not robot.initialize():
        print("❌ Failed to initialize robot")
        return False
    
    print("✅ Robot initialized and in INITIAL POSE\n")
    
    print("[2/3] Loading motion sequence...")
    motion_file = Path(__file__).parent.parent / "trajectory_exports" / "trajectory_20260325_160619.json"
    
    if not motion_file.exists():
        print(f"❌ Motion file not found: {motion_file}")
        return False
    
    motion_data = load_trajectory(str(motion_file))
    
    print("[3/3] Replaying first 300 steps with IMU logging...")
    replay_first_steps_with_imu_log(
        robot, 
        motion_data, 
        num_steps=300,     # ✅ 300 steps = 6 seconds (50Hz)
        speed_factor=0.85,
        log_file=None      # Auto-generate filename
    )
    
    print("\n🏠 Returning to home...")
    try:
        robot.move_legs_to_initial_pose(speed=500)
        robot.move_legs_to_home(speed=500)
    except Exception as e:
        print(f"⚠️  Cleanup error: {e}")
    
    robot.shutdown()
    print("✅ Done!")
    
    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)