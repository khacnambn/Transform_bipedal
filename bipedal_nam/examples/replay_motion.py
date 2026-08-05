import json
import time
import numpy as np
from pathlib import Path
import sys

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


def replay_first_steps(robot: TransformerAPI, motion_data: dict, num_steps: int = 100, speed_factor: float = 1.0):
    """
    ✅ Replay FIRST N steps từ motion sequence with timestamp sync
    
    Args:
        robot: TransformerAPI instance
        motion_data: Motion data from JSON
        num_steps: Number of steps to replay (default: 100)
        speed_factor: 1.0=normal, 0.5=slow, 2.0=fast
    """
    
    steps = motion_data["trajectory"]
    metadata = motion_data["metadata"]
    dt = metadata["dt_s"]
    
    # Limit to available steps
    num_steps = min(num_steps, len(steps))
    
    print(f"🎬 MOTION REPLAY - FIRST {num_steps} STEPS")
    print(f"{'='*80}")
    print(f"   Total steps available: {len(steps)}")
    print(f"   Steps to replay: {num_steps}")
    print(f"   Duration: {num_steps * dt * speed_factor:.2f}s")
    print(f"   Speed factor: {speed_factor}x")
    print(f"{'='*80}\n")
    
    try:
        # Start timer
        t_start = time.time()
        t_sim_start = steps[0]["time_s"]  # Simulation time của step 0
        
        for idx in range(num_steps):
            step_data = steps[idx]
            
            # ✅ SYNC TIMESTAMP
            t_sim = step_data["time_s"]              # Simulation time
            t_elapsed_sim = t_sim - t_sim_start      # Elapsed time in simulation
            t_elapsed_real = time.time() - t_start   # Elapsed time in real-world
            
            # Apply speed factor
            t_expected = t_elapsed_sim / speed_factor  # Expected real time
            t_delta = t_expected - t_elapsed_real       # How much we're ahead/behind
            
            # Sync: sleep if ahead, skip if too far behind
            if t_delta > 0.001:  # >1ms ahead
                time.sleep(t_delta)
            elif t_delta < -0.05:  # >50ms behind
                print(f"  ⚠️  Step {idx}: Behind by {abs(t_delta)*1000:.0f}ms (skipping...)")
            
            # Extract joint angles từ JSON
            angles = step_data["angles_deg"]
            hip_l, hip_r = angles[0], angles[1]
            knee_l, knee_r = angles[2], angles[3]
            ankle_l, ankle_r = angles[4], angles[5]
            
            # Get IMU data (for logging)
            imu = step_data.get("imu", {})
            roll = imu.get("roll_rad", 0)
            pitch = imu.get("pitch_rad", 0)
            
            # Apply to hardware
            success = robot.set_joint_angles(
                hip_l, knee_l, ankle_l,
                hip_r, knee_r, ankle_r
            )
            
            if not success:
                print(f"  ❌ Step {idx}: Failed to apply joint angles")
                break
            
            # Log every 10 steps
            if (idx + 1) % 10 == 0:
                sync_error = (t_expected - t_elapsed_real) * 1000  # ms
                print(f"Step {idx+1:3d} | "
                      f"Sim: {t_sim:6.2f}s | "
                      f"Real: {t_elapsed_real:6.2f}s | "
                      f"Sync: {sync_error:+6.1f}ms | "
                      f"IMU: roll={roll:+.4f}rad pitch={pitch:+.4f}rad | "
                      f"Angles: [{hip_l:+6.1f}°, {knee_l:+6.1f}°, {ankle_l:+6.1f}°]")
        
        print(f"\n✅ Replay complete ({num_steps} steps)!")
        
    except KeyboardInterrupt:
        print(f"\n👋 Interrupted by user at step {idx}")
    
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Main: Initialize robot, then replay first 100 steps"""
    
    print("=" * 80)
    print("🚶 BIPEDAL ROBOT - REPLAY FIRST 100 STEPS")
    print("=" * 80)
    
    # ========================================================================
    # KHỞI TẠO ROBOT (giống như policy_run.py)
    # ========================================================================
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
    
    # ========================================================================
    # LOAD MOTION SEQUENCE
    # ========================================================================
    print("[2/3] Loading motion sequence...")
    motion_file = Path(__file__).parent.parent / "trajectory_exports" / "trajectory_20260325_160619.json"
    
    if not motion_file.exists():
        print(f"❌ Motion file not found: {motion_file}")
        return False
    
    motion_data = load_trajectory(str(motion_file))
    
    # ========================================================================
    # REPLAY FIRST 200 STEPS
    # ========================================================================
    print("[3/3] Replaying first 200 steps...")
    replay_first_steps(robot, motion_data, num_steps=300, speed_factor=0.85)
    
    # ========================================================================
    # RETURN TO HOME
    # ========================================================================
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