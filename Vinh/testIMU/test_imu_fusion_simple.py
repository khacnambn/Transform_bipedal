import zmq
import math
import numpy as np
import time
import json


class SimpleIMUFusion:
    """Đơn giản: Đọc 2 IMU -> Chuyển đổi -> Fusion -> In kết quả"""

    def __init__(
        self,
        left_host="127.0.0.1",
        left_port=5556,
        right_host="127.0.0.1",
        right_port=5555,
    ):
        # ✅ Tạo controller riêng cho mỗi leg (y hệt walking_gait_dual.py)
        self.left = IMUController(left_host, left_port, "LEFT")
        self.right = IMUController(right_host, right_port, "RIGHT")

        # IMU positions (từ URDF)
        self.left_pos = np.array([-2.3585e-05, -0.015, 0.0851])
        self.right_pos = np.array([0, 0.015, 0.0851])

        # Rotation matrices (từ URDF rpy)
        roll_left = 90 * math.pi / 180
        pitch_left = 0 * math.pi / 180
        yaw_left = -90 * math.pi / 180
        self.left_rot = self.euler_to_rot(roll_left, pitch_left, yaw_left)
        # self.left_rot = self.euler_to_rot(math.pi, 0, -math.pi/2)

        roll_right = 90 * math.pi / 180
        pitch_right = 0 * math.pi / 180
        yaw_right = 90 * math.pi / 180
        self.right_rot = self.euler_to_rot(roll_right, pitch_right, yaw_right)
        # self.right_rot = self.euler_to_rot(math.pi, 0, math.pi/2)

        self.left_q_rot = self.euler_to_quat(roll_left, pitch_left, yaw_left)
        self.right_q_rot = self.euler_to_quat(roll_right, pitch_right, yaw_right)

        print("\n" + "=" * 80)
        print("✓ SimpleIMUFusion initialized")
        print(f"  LEFT:  {left_host}:{left_port}")
        print(f"  RIGHT: {right_host}:{right_port}")
        print("=" * 80)

    def quat_normalize(self, q):
        """Normalize quaternion về unit norm"""
        norm = math.sqrt(sum(x**2 for x in q))
        if norm > 1e-10:
            return [x / norm for x in q]
        else:
            return [1, 0, 0, 0]

    def euler_to_rot(self, roll, pitch, yaw):
        """Chuyển Euler angles sang rotation matrix"""
        Rx = np.array(
            [
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)],
            ]
        )

        Ry = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )

        Rz = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )

        return Rz @ Ry @ Rx

    def euler_to_quat(self, roll, pitch, yaw):
        """Chuyển Euler angles sang quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]

    def quat_mult(self, q1, q2):
        """Nhân 2 quaternion"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        result = [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
        return self.quat_normalize(result)

    def quat_conj(self, q):
        """Liên hợp quaternion"""
        return [q[0], -q[1], -q[2], -q[3]]

    def transform_quat_to_baselink(self, q_imu, imu_name):
        """Chuyển quaternion sang Baselink (Xoay 90 độ chung cho cả 2 để khớp với Sim)"""
        angle = math.pi / 2
        axis = np.array([0, 0, 1])

        half_angle = angle / 2
        sin_half = math.sin(half_angle)
        cos_half = math.cos(half_angle)

        q_rot = [cos_half, axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half]
        q_rot = self.quat_normalize(q_rot)
        q_imu = self.quat_normalize(q_imu)

        q_temp = self.quat_mult(q_rot, q_imu)
        q_baselink = self.quat_mult(q_temp, self.quat_conj(q_rot))

        return self.quat_normalize(q_baselink)

    def transform_quat_to_baselink_debug(self, q_imu, imu_name):
        """Debug version - in tất cả các bước (Xoay 90 độ chung cho cả 2 để khớp với Sim)"""
        angle = math.pi / 2
        axis = np.array([0, 0, 1])

        half_angle = angle / 2
        sin_half = math.sin(half_angle)
        cos_half = math.cos(half_angle)

        q_rot = [cos_half, axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half]
        q_rot = self.quat_normalize(q_rot)
        q_imu = self.quat_normalize(q_imu)
        
        q_temp = self.quat_mult(q_rot, q_imu)
        q_baselink = self.quat_mult(q_temp, self.quat_conj(q_rot))

        print(f"\n  🔍 {imu_name.upper()} TRANSFORM DEBUG:")
        print(f"     [Result] q_baselink:         w={q_baselink[0]:+.4f} x={q_baselink[1]:+.4f} y={q_baselink[2]:+.4f} z={q_baselink[3]:+.4f}")
        
        return self.quat_normalize(q_baselink)

    def transform_pos_to_baselink(self, pos, imu_name):
        """Chuyển vị trí từ khung IMU sang Baselink (Bỏ xoay vì trùng trục)"""
        return pos

    def quat_to_euler(self, q):
        """Chuyển quaternion sang Euler angles (độ) - Gimbal-lock resistant"""
        w, x, y, z = q

        # Normalize trước
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm > 1e-10:
            w, x, y, z = w / norm, x / norm, y / norm, z / norm

        # Roll (x rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Gimbal lock
        else:
            pitch = math.asin(sinp)

        # Yaw (z rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll * 180 / math.pi, pitch * 180 / math.pi, yaw * 180 / math.pi

    def fuse_quat(self, q1, q2, weight1=0.5):
        """Fusion 2 quaternion"""
        weight2 = 1 - weight1

        q1 = self.quat_normalize(q1)
        q2 = self.quat_normalize(q2)

        # Cùng hemisphere
        dot = sum(q1[i] * q2[i] for i in range(4))
        if dot < 0:
            q2 = [-x for x in q2]

        # Trung bình
        fused = [q1[i] * weight1 + q2[i] * weight2 for i in range(4)]

        return self.quat_normalize(fused)

    def run(self):
        """Chạy liên tục"""
        print("\n" + "=" * 80)
        print("🔄 IMU FUSION - READING DATA")
        print("=" * 80)

        print("\n⏳ Waiting for servers to warm up IMU...")
        for i in range(5, 0, -1):
            print(f"   {i}s remaining...", end="\r")
            time.sleep(1)
        print("   ✅ Servers ready!                    ")

        try:
            iteration = 0
            while True:
                iteration += 1
                print("\n" + "-" * 80)
                print(f"⏰ {time.strftime('%H:%M:%S')} [Iteration {iteration}]")

                print("\n  📤 Requesting IMU data...")

                left_raw = self.left.read_imu()
                right_raw = self.right.read_imu()

                # Kiểm tra valid
                if not left_raw or not right_raw:
                    print("  ❌ Failed to read IMU data")
                    time.sleep(1)
                    continue

                print(f"  ✓ LEFT raw:  {[f'{x:.4f}' for x in left_raw]}")
                print(f"  ✓ RIGHT raw: {[f'{x:.4f}' for x in right_raw]}")

                # ✅ DEBUG version - in chi tiết
                left_q = self.transform_quat_to_baselink_debug(left_raw, "left")
                right_q = self.transform_quat_to_baselink_debug(right_raw, "right")

                # ✅ THÊM: Check norm sau transform
                norm_left_after = math.sqrt(sum(x**2 for x in left_q))
                norm_right_after = math.sqrt(sum(x**2 for x in right_q))

                print(f"\n  📊 NORM CHECK (after transform):")
                print(
                    f"     LEFT:  ||q|| = {norm_left_after:.6f}",
                    "✅" if abs(norm_left_after - 1.0) < 0.01 else "🔴 BAD!",
                )
                print(
                    f"     RIGHT: ||q|| = {norm_right_after:.6f}",
                    "✅" if abs(norm_right_after - 1.0) < 0.01 else "🔴 BAD!",
                )

                # Fusion
                fused_q = self.fuse_quat(left_q, right_q)

                # Chuyển đổi vị trí
                left_p = self.transform_pos_to_baselink(self.left_pos, "left")
                right_p = self.transform_pos_to_baselink(self.right_pos, "right")

                # Fusion vị trí
                fused_p = (left_p + right_p) / 2

                # In kết quả
                print("\n📍 VỊ TRÍ (Baselink):")
                print(
                    f"  LEFT:   X={left_p[0]:+.6f}  Y={left_p[1]:+.6f}  Z={left_p[2]:+.6f}"
                )
                print(
                    f"  RIGHT:  X={right_p[0]:+.6f}  Y={right_p[1]:+.6f}  Z={right_p[2]:+.6f}"
                )
                print(
                    f"  FUSION: X={fused_p[0]:+.6f}  Y={fused_p[1]:+.6f}  Z={fused_p[2]:+.6f}"
                )

                print("\n🧭 HƯỚNG (Euler - độ):")
                left_euler = self.quat_to_euler(left_q)
                right_euler = self.quat_to_euler(right_q)
                fused_euler = self.quat_to_euler(fused_q)

                print(
                    f"  LEFT:   Roll={left_euler[0]:+7.2f}°  Pitch={left_euler[1]:+7.2f}°  Yaw={left_euler[2]:+7.2f}°"
                )
                print(
                    f"  RIGHT:  Roll={right_euler[0]:+7.2f}°  Pitch={right_euler[1]:+7.2f}°  Yaw={right_euler[2]:+7.2f}°"
                )
                print(
                    f"  FUSION: Roll={fused_euler[0]:+7.2f}°  Pitch={fused_euler[1]:+7.2f}°  Yaw={fused_euler[2]:+7.2f}°"
                )

                print("\n📊 QUATERNION (Baselink):")
                print(
                    f"  LEFT:   w={left_q[0]:+.4f}  x={left_q[1]:+.4f}  y={left_q[2]:+.4f}  z={left_q[3]:+.4f}"
                )
                print(
                    f"  RIGHT:  w={right_q[0]:+.4f}  x={right_q[1]:+.4f}  y={right_q[2]:+.4f}  z={right_q[3]:+.4f}"
                )
                print(
                    f"  FUSION: w={fused_q[0]:+.4f}  x={fused_q[1]:+.4f}  y={fused_q[2]:+.4f}  z={fused_q[3]:+.4f}"
                )

                time.sleep(1)

        except KeyboardInterrupt:
            print("\n\n👋 Stopped!")
        except Exception as e:
            print(f"\n❌ Error: {e}")
            import traceback

            traceback.print_exc()


class IMUController:
    """✅ Giống y hệt SingleLegController từ walking_gait_dual.py"""

    def __init__(self, host: str, port: int, side: str = "LEFT"):
        self.host = host
        self.port = port
        self.side = side

        # ✅ ZeroMQ socket setup
        self.context = zmq.Context()
        self.socket = None
        self.connect()

    def connect(self):
        """Tạo socket mới"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
        self.socket.setsockopt(zmq.LINGER, 0)  # ✅ Quan trọng!

        try:
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            print(f"✓ {self.side} IMU controller connected to {self.host}:{self.port}")
        except Exception as e:
            print(f"✗ {self.side} connection failed: {e}")
            raise

    def read_imu(self):
        """✅ Đọc IMU - RESET socket nếu fail"""
        try:
            # 1️⃣ GỬI request
            self.socket.send_json({"type": "feedback"})

            # 2️⃣ CHỜ response
            response = self.socket.recv_json()

            # 3️⃣ LẤY imu data
            imu_data = response.get("imu", [1, 0, 0, 0])

            # 4️⃣ KIỂM TRA valid
            if len(imu_data) == 4:
                return imu_data
            else:
                print(f"❌ {self.side}: Invalid IMU data length {len(imu_data)}")
                return None

        except zmq.Again:
            print(f"❌ {self.side}: Timeout waiting for IMU")
            # ✅ RESET socket sau timeout
            self.connect()
            return None

        except zmq.error.ZMQError as e:
            print(f"❌ {self.side}: ZMQ Error - {e}")
            # ✅ RESET socket khi có lỗi
            self.connect()
            return None

        except Exception as e:
            print(f"❌ {self.side}: Error - {e}")
            # ✅ RESET socket khi có lỗi
            self.connect()
            return None


def main():
    try:
        fuser = SimpleIMUFusion(
            left_host="mobile2.local",
            left_port=5556,
            right_host="mobile1.local",
            right_port=5555,
        )
        fuser.run()
    except Exception as e:
        print(f"❌ Failed to initialize: {e}")
        print("\n📋 Make sure servers are running:")
        print(
            "  Terminal 1: python -m src.leg_server.leg_server_left --port 5556 --debug"
        )
        print(
            "  Terminal 2: python -m src.leg_server.leg_server_right --port 5555 --debug"
        )


if __name__ == "__main__":
    main()
