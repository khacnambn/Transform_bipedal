"""
Test script for IMU API - similar to test_imu_fusion_simple.py
but using the IMUFusion API module
"""

import sys
import time
import math
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "bipedal_nam" / "src"))

from bipedal_robot.sensors.imu import IMUFusion


def print_header(text):
    """Print formatted header"""
    print("\n" + "=" * 80)
    print(text)
    print("=" * 80)


def print_section(text):
    """Print formatted section"""
    print("\n" + "-" * 80)
    print(text)
    print("-" * 80)


def format_quaternion(q, label="Quaternion"):
    """Format quaternion for display"""
    return f"{label}: w={q[0]:+.4f}  x={q[1]:+.4f}  y={q[2]:+.4f}  z={q[3]:+.4f}"


def format_euler(euler, label="Euler"):
    """Format euler angles (in degrees) for display"""
    roll, pitch, yaw = euler
    return f"{label}: Roll={roll:+7.2f}°  Pitch={pitch:+7.2f}°  Yaw={yaw:+7.2f}°"


def format_position(pos, label="Position"):
    """Format position for display"""
    return f"{label}: X={pos[0]:+.6f}  Y={pos[1]:+.6f}  Z={pos[2]:+.6f}"


def main():
    """Main test function"""

    print_header(" IMU FUSION API TEST")

    # Initialize IMUFusion API
    print("\n Initializing IMU Fusion API...")
    try:
        imu_fusion = IMUFusion(
            left_host="mobile2.local",
            left_port=5556,
            right_host="mobile1.local",
            right_port=5555,
        )
        print(" IMU Fusion API initialized successfully")

        #  THÊM: Print rotation matrices để verify
        print("\n" + "=" * 80)
        print("ROTATION MATRICES (IMU → Baselink)")
        print("=" * 80)

        # print("\nLEFT IMU (Rz(-90°)):")
        # print(imu_fusion.left_rot)
        # print("\nRIGHT IMU (Rz(+90°)):")
        # print(imu_fusion.right_rot)
        # print("\n" + "="*80 + "\n")

    except Exception as e:
        print(f" Failed to initialize IMU Fusion API: {e}")
        print("\n Make sure servers are running:")
        print(
            "  Terminal 1: python -m src.leg_server.leg_server_left --port 5556 --debug"
        )
        print(
            "  Terminal 2: python -m src.leg_server.leg_server_right --port 5555 --debug"
        )
        return

    # Wait for servers to warm up
    print("\n Waiting for servers to warm up IMU...")
    for i in range(5, 0, -1):
        print(f"   {i}s remaining...", end="\r")
        time.sleep(1)
    print("    Servers ready!                    ")

    iteration = 0

    try:
        while True:
            iteration += 1

            # Get current time
            current_time = time.strftime("%H:%M:%S")
            print_section(f"⏰ {current_time} [Iteration {iteration}]")

            # Read fused IMU data
            print(" Requesting fused IMU data...")
            imu_data = imu_fusion.get_fused_imu()

            # Check if data is valid
            if imu_data is None:
                print(" Failed to read IMU data - retrying...")
                time.sleep(1)
                continue

            # Extract data
            left_quat = imu_data["left_quat"]
            right_quat = imu_data["right_quat"]
            fused_quat = imu_data["fused_quat"]
            fused_euler = imu_data["fused_euler"]

            #  THÊM: Extract Gyro data
            left_gyro = imu_data["left_gyro"]
            right_gyro = imu_data["right_gyro"]
            fused_gyro = imu_data["fused_gyro"]

            timestamp = imu_data["timestamp"]

            # Display transformed quaternion data
            print("\n TRANSFORMED QUATERNION (Baselink frame):")
            print(f"  {format_quaternion(left_quat, 'LEFT ')}")
            print(f"  {format_quaternion(right_quat, 'RIGHT')}")

            # Display norm check
            norm_left = math.sqrt(sum(x**2 for x in left_quat))
            norm_right = math.sqrt(sum(x**2 for x in right_quat))
            norm_fused = math.sqrt(sum(x**2 for x in fused_quat))

            print("\n QUATERNION NORM CHECK (should be ~1.0):")
            status_left = "✅" if abs(norm_left - 1.0) < 0.01 else "🔴"
            status_right = "✅" if abs(norm_right - 1.0) < 0.01 else "🔴"
            status_fused = "✅" if abs(norm_fused - 1.0) < 0.01 else "🔴"
            print(f"  LEFT:  ||q|| = {norm_left:.6f} {status_left}")
            print(f"  RIGHT: ||q|| = {norm_right:.6f} {status_right}")
            print(f"  FUSED: ||q|| = {norm_fused:.6f} {status_fused}")

            # Display fused quaternion
            print("\n FUSED QUATERNION:")
            print(f"  {format_quaternion(fused_quat, 'FUSED')}")

            # Display Euler angles (converted from fused quaternion)
            print("\n EULER ANGLES (from fused quaternion):")
            # Convert to degrees for display
            roll_deg, pitch_deg, yaw_deg = fused_euler
            roll_deg *= 180 / math.pi
            pitch_deg *= 180 / math.pi
            yaw_deg *= 180 / math.pi
            print(
                f"  FUSED: Roll={roll_deg:+7.2f}°  Pitch={pitch_deg:+7.2f}°  Yaw={yaw_deg:+7.2f}°"
            )

            # THÊM: Display GYRO data
            print("\n🌀 GYRO (Angular Velocity) - Baselink Frame [rad/s]:")
            print(
                f"  LEFT:  GX={left_gyro[0]:+.6f}  GY={left_gyro[1]:+.6f}  GZ={left_gyro[2]:+.6f}"
            )
            print(
                f"  RIGHT: GX={right_gyro[0]:+.6f}  GY={right_gyro[1]:+.6f}  GZ={right_gyro[2]:+.6f}"
            )
            print(
                f"  FUSED: GX={fused_gyro[0]:+.6f}  GY={fused_gyro[1]:+.6f}  GZ={fused_gyro[2]:+.6f}"
            )

            #  THÊM: Magnitude of gyro
            gyro_mag = math.sqrt(sum(x**2 for x in fused_gyro))
            print(f"  MAGNITUDE: {gyro_mag:.6f} rad/s")

            # Display timestamp
            print(f"\n⏱  Timestamp: {timestamp:.3f}")

            # Display data in compact format (useful for logging)
            print("\n COMPACT FORMAT (for logging/analysis):")
            print(
                f"  Fused Quat: [{fused_quat[0]:+.4f}, {fused_quat[1]:+.4f}, {fused_quat[2]:+.4f}, {fused_quat[3]:+.4f}]"
            )
            print(
                f"  Fused Euler (deg): [{roll_deg:+7.2f}°, {pitch_deg:+7.2f}°, {yaw_deg:+7.2f}°]"
            )
            print(
                f"  Fused Gyro (rad/s): [{fused_gyro[0]:+.6f}, {fused_gyro[1]:+.6f}, {fused_gyro[2]:+.6f}]"
            )

            # Wait before next iteration
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n Test stopped by user")

    except Exception as e:
        print(f"\n Error during test: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        print("\n🔌 Closing IMU connections...")
        imu_fusion.close()
        print(" IMU connections closed")


if __name__ == "__main__":
    main()
