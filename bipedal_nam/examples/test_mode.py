#!/usr/bin/env python

"""
Test Feetech POSITION mode với điều chỉnh Acceleration và Goal_Velocity
cho một động cơ duy nhất ID = 1, không sửa code trong feetech.py.
"""

import sys
import time
from pathlib import Path

# Thêm lerobot/src vào sys.path (đi lên 1 level từ bipedal_nam → Lekiwi_new → lerobot/src)
ROOT = Path(__file__).resolve().parents[2]
LEROBOT_SRC = ROOT / "lerobot" / "src"
sys.path.insert(0, str(LEROBOT_SRC))

from lerobot.motors import Motor, MotorNormMode  # type: ignore
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode  # type: ignore


def main():
    # Định nghĩa 1 motor ID=1, điều chỉnh model đúng với motor thật của bạn: "sts3215" hoặc "sts3095"
    motors = {
        "test_motor": Motor(
            id=1,
            model="sts3095",  # đổi thành "sts3095" nếu motor ID=1 là sts3095
            norm_mode=MotorNormMode.RANGE_M100_100,
        )
    }

    bus = FeetechMotorsBus(
        port="/dev/ttyACM0",  # đổi nếu đang dùng port khác
        motors=motors,
    )

    motor_name = "test_motor"
    center = 2048  # vị trí trung tâm giả định

    try:
        print(f"[1] Kết nối tới {bus.port} ...")
        # handshake=False để tránh lỗi firmware mismatch nếu nhiều version
        bus.connect()
        print("  ✓ Connected\n")

        # CHECK: thử đọc position của motor ID=1 để xem có phản hồi không
        print("[1.1] Kiểm tra Motor ID=1 (test_motor) có phản hồi không...")
        try:
            pos = bus.read("Present_Position", motor_name, normalize=False)
            print(f"  ✓ Motor ID=1 Present_Position = {pos}")
        except Exception as e:
            print(f"  ✗ Không đọc được Motor ID=1: {e}")
            print("    → Kiểm tra lại: cổng serial, ID servo, nguồn, dây tín hiệu.")
            return

        # Đặt mode = POSITION
        print("\n[2] Set Operating_Mode = POSITION (0)")
        bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
        time.sleep(0.1)

        # Bật torque
        print("[3] Enable torque")
        bus.write("Torque_Enable", motor_name, 1)
        time.sleep(0.1)

        # Đưa về center lần đầu
        print(f"[4] Move to center = {center} (raw ticks)")
        bus.write("Goal_Position", motor_name, center, normalize=False)
        time.sleep(2.0)

        # Đọc lại position hiện tại
        pos_now = bus.read("Present_Position", motor_name, normalize=False)
        print(f"  → Present_Position = {pos_now}")

        # TEST 1: Move chậm (speed nhỏ, acc nhỏ)
        print("\n[5] Test 1: Move slow (speed nhỏ, acc nhỏ)")

        slow_speed = 500    # tăng từ 100
        slow_acc = 20       # tăng từ 5

        print(f"  - Acceleration = {slow_acc}")
        bus.write("Acceleration", motor_name, slow_acc, normalize=False)
        time.sleep(0.05)

        print(f"  - Goal_Velocity = {slow_speed}")
        bus.write("Goal_Velocity", motor_name, slow_speed, normalize=False)
        time.sleep(0.05)

        target1 = center + 800  # tăng từ 400
        print(f"  - Goal_Position = {target1}")
        bus.write("Goal_Position", motor_name, target1, normalize=False)

        print("  → Đợi 5s để quan sát chuyển động chậm...")
        time.sleep(5.0)

        # TEST 2: Move nhanh (speed lớn, acc lớn)
        print("\n[6] Test 2: Move fast (speed lớn, acc lớn)")

        fast_speed = 2000   # tăng từ 1000
        fast_acc = 100      # tăng từ 50

        print(f"  - Acceleration = {fast_acc}")
        bus.write("Acceleration", motor_name, fast_acc, normalize=False)
        time.sleep(0.05)

        print(f"  - Goal_Velocity = {fast_speed}")
        bus.write("Goal_Velocity", motor_name, fast_speed, normalize=False)
        time.sleep(0.05)

        target2 = center - 800  # tăng từ 400
        print(f"  - Goal_Position = {target2}")
        bus.write("Goal_Position", motor_name, target2, normalize=False)

        print("  → Đợi 5s để quan sát chuyển động nhanh...")
        time.sleep(5.0)

        # Đưa về center lần cuối
        print("\n[7] Return to center")
        bus.write("Goal_Position", motor_name, center, normalize=False)
        time.sleep(2.0)

        # Tắt torque
        print("[8] Disable torque")
        bus.write("Torque_Enable", motor_name, 0, normalize=False)
        print("  ✓ Done")

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            bus.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()