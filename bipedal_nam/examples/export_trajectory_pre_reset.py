import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np
import torch
from isaaclab.app import AppLauncher
import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Export trajectory PRE-RESET only")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point", help="Agent config.")
parser.add_argument("--num_episodes", type=int, default=5, help="Number of episodes to try.")
parser.add_argument("--output_dir", type=str, default="trajectory_exports", help="Output directory.")

cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

sys.argv = [sys.argv[0]] + hydra_args
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
from rsl_rl.runners import DistillationRunner, OnPolicyRunner
from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config
import transformer_nam.tasks  # noqa: F401


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg, agent_cfg):
    """Export trajectory PRE-RESET only"""
    task_name = args_cli.task.split(":")[-1]
    
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = 1  # Chỉ 1 env
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
    
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    
    print(f"Loading checkpoint: {resume_path}")
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic
    
    dt = env.unwrapped.step_dt
    
    # ✅ THÊM: Container lưu trajectory PRE-RESET
    trajectory_pre_reset = []
    
    print(f"\n{'='*80}")
    print(f"🎬 EXPORTING TRAJECTORY (PRE-RESET ONLY)")
    print(f"{'='*80}\n")
    
    episode_count = 0
    start_time = datetime.now()
    
    while simulation_app.is_running() and episode_count < args_cli.num_episodes:
        obs = env.get_observations()
        step_count = 0
        episode_trajectory = []
        
        print(f"📍 Episode {episode_count + 1}/{args_cli.num_episodes} starting...")
        
        while simulation_app.is_running():
            with torch.inference_mode():
                actions = policy(obs)
                
                env_internal = env.unwrapped
                real_angles = env_internal.cmd_actions[0].cpu().numpy().tolist()
                
                obs_data = obs["policy"][0]
                roll = obs_data[15].item()
                pitch = obs_data[16].item()
                gx = obs_data[17].item()
                gy = obs_data[18].item()
                gz = obs_data[19].item()
                
                # Raw policy actions
                raw_actions = actions[0].cpu().numpy().tolist()
                
                # ✅ THÊM: Lưu step vào episode trajectory
                step_record = {
                    "step": step_count,
                    "time_s": step_count * dt,
                    "angles_deg": [float(f"{a:.2f}") for a in real_angles],
                    "imu": {
                        "roll_rad": float(f"{roll:.6f}"),
                        "pitch_rad": float(f"{pitch:.6f}"),
                        "gyro": {
                            "gx": float(f"{gx:.6f}"),
                            "gy": float(f"{gy:.6f}"),
                            "gz": float(f"{gz:.6f}")
                        }
                    },
                    "raw_actions": [float(f"{a:.4f}") for a in raw_actions]
                }
                episode_trajectory.append(step_record)
                
                # ✅ THÊM: Detect RESET (done=True)
                obs, _, dones, _ = env.step(actions)
                policy_nn.reset(dones)
                
                step_count += 1
                
                # ✅ THÊM: Check if episode ended
                if dones[0]:
                    print(f"  ✓ Episode ended at step {step_count}")
                    break
        
        # ✅ THÊM: Nếu episode có data, thêm vào trajectory chính
        if episode_trajectory:
            # Cập nhật time_s để continuous
            start_time_s = len(trajectory_pre_reset) * dt
            for step_rec in episode_trajectory:
                step_rec["time_s"] = start_time_s + step_rec["step"] * dt
                step_rec["step"] = len(trajectory_pre_reset)
                trajectory_pre_reset.append(step_rec)
            
            print(f"  ✓ Added {len(episode_trajectory)} steps to trajectory (total: {len(trajectory_pre_reset)})")
        
        episode_count += 1
    
    # ✅ THÊM: Lưu trajectory PRE-RESET
    os.makedirs(args_cli.output_dir, exist_ok=True)
    
    output_data = {
        "metadata": {
            "task": task_name,
            "checkpoint": os.path.basename(resume_path),
            "total_steps": len(trajectory_pre_reset),
            "duration_s": len(trajectory_pre_reset) * dt,
            "dt_s": float(dt),
            "timestamp": start_time.isoformat(),
            "joints": ["Hip_L", "Hip_R", "Knee_L", "Knee_R", "Ankle_L", "Ankle_R"],
            "servo_limits": {
                "Hip": {"max": 35, "min": -35},
                "Knee": {"max": 85, "min": -85},
                "Ankle": {"max": 45, "min": -45}
            },
            "collection_method": "PRE-RESET ONLY (stops at first reset per episode)"
        },
        "trajectory": trajectory_pre_reset
    }
    
    timestamp_str = start_time.strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(args_cli.output_dir, f"trajectory_pre_reset_{timestamp_str}.json")
    
    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"\n{'='*80}")
    print(f"✅ Exported {len(trajectory_pre_reset)} pre-reset steps")
    print(f"   From {episode_count} episodes")
    print(f"   Duration: {len(trajectory_pre_reset) * dt:.2f}s")
    print(f"   File: {output_file}")
    print(f"{'='*80}\n")
    
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()