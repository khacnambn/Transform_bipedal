import torch
import numpy as np
import time
from pathlib import Path
import sys
import math

sys.path.insert(0, str(Path(__file__).parent.parent)) 

from src.bipedal_robot.transformer import TransformerAPI

# ============================================================================
# IMU SCALING (Orient + Gyro)
# ============================================================================

def scale_imu(orient_raw, gyro_raw,
              orient_min=-1.0, orient_max=1.0,
              gyro_min=-2.0, gyro_max=2.0):
    """
    Args:
        orient_raw: (roll, pitch, yaw) in radians
        gyro_raw: (gx, gy, gz) in rad/s
        orient_min/max: Orient range (training: [-1, 1])
        gyro_min/max: Gyro range (training: [-2, 2] rad/s)
    
    Returns:
        scaled_orient: [roll_scaled, pitch_scaled] (2 values)
        scaled_gyro: [gx_scaled, gy_scaled, gz_scaled] (3 values)
    """
    scaled_orient = []
    scaled_gyro = []
    
    # ============================================================
    # ORIENTATION: Scale roll, pitch ONLY (2 values)
    # ============================================================
    for val in orient_raw[:2]:
        # Clamp to range
        clipped = np.clip(val, orient_min, orient_max)
        
        # Normalize to [-1, 1]
        if orient_max == orient_min:
            normalized = 0.0
        else:
            normalized = (clipped - orient_min) / (orient_max - orient_min) * 2.0 - 1.0
        
        # Final clamp
        normalized = max(-1.0, min(1.0, normalized))
        scaled_orient.append(normalized)
    
    # ============================================================
    # GYRO: Scale gx, gy, gz (3 values, from rad/s)
    # ============================================================
    for val in gyro_raw[:3]:
        # Clamp to range
        clipped = np.clip(val, gyro_min, gyro_max)
        
        # Normalize to [-1, 1]
        if gyro_max == gyro_min:
            normalized = 0.0
        else:
            normalized = (clipped - gyro_min) / (gyro_max - gyro_min) * 2.0 - 1.0
        
        # Final clamp
        normalized = max(-1.0, min(1.0, normalized))
        scaled_gyro.append(normalized)
    
    return scaled_orient, scaled_gyro


def get_gyro_from_state(state):
    """
    Extract gyro data từ state dict
    Gyro được tính từ IMU angular velocity (rad/s)
    
    Returns:
        [gx, gy, gz] in rad/s
    """
    # Nếu server cấp gyro trực tiếp
    if "gyro" in state:
        return state["gyro"]
    
    # Nếu không, dùng angular_vel (từ IMU)
    if "angular_vel" in state:
        return state["angular_vel"]
    
    # Fallback: zeros
    return [0.0, 0.0, 0.0]


# ============================================================================
# OBSERVATION PROCESSING (44D: 8 orient + 12 gyro + 24 action)
# ============================================================================

def process_observations(robot: TransformerAPI, orient_history, gyro_history, action_history):
    """
    Process observations - MATCH TRAINING ENVIRONMENT (44D)
    
    Structure:
    - Orient history: 4 frames × 2 (roll, pitch) = 8 values
    - Gyro history:   4 frames × 3 (gx, gy, gz) = 12 values  ← ADDED
    - Action history: 4 frames × 6 joints = 24 values
    - Total: 44 values ✓
    
    Like transformer_nam_env.py:
        imu_data = torch.cat((self.orient_h[:, :, :2], self.gyro_h), dim=2)
        imu_data = imu_data.reshape(self.num_envs, 20)  # 8 + 12 = 20
        proc_act = self.act_hist.reshape(self.num_envs, 24)
        obs_buffer = torch.cat((imu_data, proc_act), dim=1)  # 20 + 24 = 44
    """
    observations = []

    # ✅ Add orient history (2 values per frame: roll, pitch)
    for orient in orient_history:
        observations.extend(orient)

    # ✅ Add gyro history (3 values per frame: gx, gy, gz) - NEWLY ADDED
    for gyro in gyro_history:
        observations.extend(gyro)

    # Add action history (6 joints per frame)
    for actions in action_history:
        observations.extend(actions)

    # Round values to reduce noise
    observations = [round(val, 4) for val in observations]
    
    # Verify 44D
    if len(observations) != 44:
        print(f"⚠️  WARNING: Expected 44D observations, got {len(observations)}D!")
        print(f"  Orient frames: {len(orient_history)} × 2 = {len(orient_history)*2}D")
        print(f"  Gyro frames:   {len(gyro_history)} × 3 = {len(gyro_history)*3}D")
        print(f"  Action frames: {len(action_history)} × 6 = {len(action_history)*6}D")

    return np.array(observations, dtype=np.float32)


# ============================================================================
# ACTION PROCESSING (match api_example.py)
# ============================================================================

def process_actions(robot, model_actions, last_actions):
    """
    Converts NN action to degrees (delta accumulation)
    
    Model output: 6 values (1 frame)
    """
    # Clip model output (training range)
    actions = np.clip(model_actions, -3.0, 3.0)
    
    # Delta accumulation
    actions = np.array(last_actions) + (actions * 4 / 3)
    
    # Clip to joint limits (6 joints)
    servo_min = [20, 20, -55, -55, 20, 20]
    servo_max = [30, 30, -45, -45, 30, 30]
    
    actions = np.array(actions)
    for i in range(len(actions)):
        actions[i] = np.clip(actions[i], servo_min[i], servo_max[i])
    
    return actions.tolist()
    

def scale_actions_for_obs(robot, actions):
    """Scale actions để dùng trong observation buffer"""
    servo_min = [20, 20, -55, -55, 20, 20]
    servo_max = [30, 30, -45, -45, 30, 30]
    
    scaled = []
    for i, action in enumerate(actions):
        vmin = servo_min[i]
        vmax = servo_max[i]
        
        if vmax == vmin:
            normalized = 0.0
        else:
            normalized = (action - vmin) / (vmax - vmin) * 2 - 1
        
        # Clamp to [-1, 1]
        normalized = max(-1.0, min(1.0, normalized))
        scaled.append(normalized)
    
    return scaled


# ============================================================================
# UPDATE BUFFERS (NOW WITH GYRO)
# ============================================================================

def update_buffers(robot, state_orient, state_gyro, last_actions, 
                   orient_history, gyro_history, action_history):
    """
    Update orientation, gyro & action buffers with scaling
    Match training environment buffer update
    """
    # Scale IMU (orientation + gyro)
    orient_scaled, gyro_scaled = scale_imu(state_orient, state_gyro)
    
    # Scale actions for observation buffer
    actions_scaled = scale_actions_for_obs(robot, last_actions)
    
    # Store into buffers (FIFO - pop(0), append)
    orient_history.pop(0)
    orient_history.append(orient_scaled)
    
    gyro_history.pop(0)              # ← NEW
    gyro_history.append(gyro_scaled)
    
    action_history.pop(0)
    action_history.append(actions_scaled)


# ============================================================================
# POLICY INFERENCE
# ============================================================================

def perform_inference(policy_model, observations):
    """Perform policy inference"""
    with torch.no_grad():
        obs_tensor = torch.tensor(observations, dtype=torch.float32).unsqueeze(0)
        policy_output = policy_model(obs_tensor)
    
    # Extract numpy array from output
    if isinstance(policy_output, torch.Tensor):
        policy_actions = policy_output.squeeze(0).numpy()
    else:
        policy_actions = policy_output[0] if isinstance(policy_output, (list, tuple)) else policy_output
    
    return policy_actions.flatten()


# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

INITIAL_SERVO_POSITIONS = [
    25.0,    # hip_l
    25.0,    # hip_r
    -50.0,    # knee_l
    -50.0,    # knee_r
    25.0,    # ankle_l
    25.0     # ankle_r
]

def main():
    """Main policy control loop - Match training environment"""
    
    print("=" * 80)
    print("🚶 BIPEDAL ROBOT POLICY DEPLOYMENT (44D observations)")
    print("=" * 80)
    
    # ========================================================================
    # KHỞI TẠO ROBOT
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
    # LOAD POLICY MODEL
    # ========================================================================
    print("[2/3] Loading policy model...")
    try:
        policy_model = torch.jit.load(
            "/home/nam/Transform_Bipedal/bipedal_nam/logs/2026-03-26_13-33-46/exported/policy.pt"
        )
        policy_model.eval()
        print("✅ Policy loaded (expecting 44D input)")
    except Exception as e:
        print(f"❌ Failed to load policy: {e}")
        return False
    
    # ========================================================================
    # INITIALIZE BUFFERS (WITH GYRO)
    # ========================================================================
    print("\n[3/3] Initializing observation buffers...")
    
    # ✅ FIX: WAIT for IMU to be ready
    print("⏳ Waiting for IMU to stabilize...")
    imu_ready = False
    wait_start = time.time()
    max_wait = 10
    
    while not imu_ready and (time.time() - wait_start) < max_wait:
        try:
            initial_state = robot.get_state_with_retry(max_retries=1)
            if initial_state is None:
                time.sleep(0.1)
                continue
            
            orient = initial_state["orient"]
            gyro = get_gyro_from_state(initial_state)
            
            if not (abs(orient[0]) < 0.001 and abs(orient[1]) < 0.001 and 
                    abs(gyro[0]) < 0.001 and abs(gyro[1]) < 0.001):
                imu_ready = True
                print(f"✅ IMU ready!")
                break
            
            print(f"  Waiting... ({(time.time() - wait_start):.1f}s)", end="\r")
            time.sleep(0.2)
        
        except Exception as e:
            time.sleep(0.2)
    
    if not imu_ready:
        print(f"\n❌ IMU did not stabilize after {max_wait}s")
        return False
    
    # ========================================================================
    # NOW initialize buffers with real IMU data + HARD-CODED INITIAL POSITIONS
    # ========================================================================
    print("\n📊 Getting initial IMU state...")
    
    try:
        initial_state = robot.get_state_with_retry(max_retries=3)
        if initial_state is None:
            print("❌ Failed to get initial state")
            return False
        
        orient = initial_state["orient"]
        gyro = get_gyro_from_state(initial_state)
    except Exception as e:
        print(f"❌ Failed to get initial state: {e}")
        return False
    
    # Initialize with scaled values
    start_scaled_orient, start_scaled_gyro = scale_imu(orient, gyro)
    
    # ✅ THÊM: Use hard-coded initial servo positions
    start_scaled_actions = scale_actions_for_obs(robot, INITIAL_SERVO_POSITIONS)
    
    orient_history = [start_scaled_orient for _ in range(4)]
    gyro_history = [start_scaled_gyro for _ in range(4)]
    action_history = [start_scaled_actions for _ in range(4)]  
    last_actions = INITIAL_SERVO_POSITIONS.copy()  
    
    print(f"  Initial state:")
    print(f"    Orient (raw): roll={orient[0]:.4f}rad, pitch={orient[1]:.4f}rad")
    print(f"    Gyro (raw):   gx={gyro[0]:.4f}rad/s, gy={gyro[1]:.4f}rad/s, gz={gyro[2]:.4f}rad/s")
    print(f"    ✅ ACTION HISTORY (hard-coded): {INITIAL_SERVO_POSITIONS}")
    print(f"    ✅ ACTION HISTORY (scaled): {[round(x, 4) for x in start_scaled_actions]}")
    print("✅ Buffers initialized (44D observations with INITIAL SERVO POSITIONS)\n")
    
    # ========================================================================
    # MAIN CONTROL LOOP (50ms period)
    # ========================================================================
    print("-" * 80)
    print("Starting control loop (50ms period, 44D observations)")
    print("-" * 80 + "\n")
    
    period = 0.05
    t1 = time.time()
    step = 0
    
    try:
        while True:
            step += 1
            
            # Get state data
            state = robot.get_state_with_retry(max_retries=2)
            
            if state is None:
                print(f"❌ Step {step}: Failed to get state")
                break
            
            roll, pitch, yaw = state["orient"]
            gyro = get_gyro_from_state(state)
            
            # Update buffers (WITH GYRO)
            update_buffers(robot, state["orient"], gyro, last_actions, 
                          orient_history, gyro_history, action_history)
            
            # Calculate observations array (44D)
            observations = process_observations(robot, orient_history, gyro_history, action_history)
            
            if len(observations) != 44:
                print(f"❌ CRITICAL: Observation size mismatch!")
                break
            
            # Perform NN inference
            policy_actions = perform_inference(policy_model, observations)
            
            if len(policy_actions) != 6:
                print(f"❌ CRITICAL: Model output size mismatch!")
                break
            
            # Process new actions (delta accumulation)
            new_actions = process_actions(robot, policy_actions, last_actions)
            last_actions = new_actions
            
            # Execute actions
            hip_l, hip_r = new_actions[0], new_actions[1]
            knee_l, knee_r = new_actions[2], new_actions[3]
            ankle_l, ankle_r = new_actions[4], new_actions[5]
            
            success = robot.set_joint_angles(
                hip_l, knee_l, ankle_l,
                hip_r, knee_r, ankle_r
            )
            
            if not success:
                print(f"❌ Step {step}: Failed to apply joint angles")
                break
            # Log every 2 steps
            if step % 2 == 0:
                print(f"Step {step:5d} | "
                      f"Orient: roll={roll:+6.4f}rad pitch={pitch:+6.4f}rad | "
                      f"Gyro: gx={gyro[0]:+6.4f}rad/s | "
                      f"Actions: {[f'{a:+6.1f}°' for a in new_actions]}")
            
            # Timestep sync
            t1 += period
            sleep_dt = max(0, t1 - time.time())
            time.sleep(sleep_dt)
    
    except KeyboardInterrupt:
        print("\n\n👋 Interrupted by user")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n" + "-" * 80)
        print("🏠 Returning to home...")
        try:
            robot.move_legs_to_initial_pose(speed=500)
            robot.move_legs_to_home(speed=500)
        except Exception as e:
            print(f"⚠️  Cleanup error: {e}")
        
        robot.shutdown()
        print("✅ Done!")
        return True


if __name__ == "__main__":
    main()