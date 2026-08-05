import zmq
import math
import numpy as np
import time

class IMUOffsetFineTune:
    """
    🔬 DIAGNOSTIC: Fine-tune offset NHỎ quanh base transform
    
    ✅ Cải thiện: 
    - Tăng samples: 100 → 500 frames (25 giây)
    - Theo dõi drift theo thời gian
    - Tính standard deviation để đánh giá stability
    """
    
    def __init__(self, left_host="127.0.0.1", left_port=5556, 
                 right_host="127.0.0.1", right_port=5555):
        self.left = IMUController(left_host, left_port, "LEFT")
        self.right = IMUController(right_host, right_port, "RIGHT")
        
        print("\n" + "=" * 80)
        print("🔬 IMU OFFSET FINE-TUNE DIAGNOSTIC (IMPROVED)")
        print("=" * 80)
        print(f"Tìm offset nhỏ (ΔRoll, ΔPitch, ΔYaw) quanh base transform")
        print(f"LEFT:  {left_host}:{left_port}")
        print(f"RIGHT: {right_host}:{right_port}")
        print(f"\nBase transforms (what you want):")
        print(f"  LEFT:  Roll ≈ 90°, Pitch ≈ 0°, Yaw ≈ -90°")
        print(f"  RIGHT: Roll ≈ 90°, Pitch ≈ 0°, Yaw ≈ +90°")
        print(f"\n📈 Improvement: 500 frames @ 20Hz = 25 seconds")
    
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
            cr * cp * sy - sr * sp * cy
        ]
    
    def quat_normalize(self, q):
        """Normalize quaternion"""
        norm = math.sqrt(sum(x**2 for x in q))
        if norm > 1e-10:
            return [x / norm for x in q]
        return [1, 0, 0, 0]
    
    def quat_mult(self, q1, q2):
        """Nhân 2 quaternion"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        result = [
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ]
        return self.quat_normalize(result)
    
    def quat_conj(self, q):
        """Liên hợp quaternion"""
        return [q[0], -q[1], -q[2], -q[3]]
    
    def quat_to_euler(self, q):
        """Chuyển quaternion sang Euler angles (radians)"""
        w, x, y, z = q
        
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        return roll, pitch, yaw
    
    def quat_to_euler_deg(self, q):
        """Lấy Euler angles (độ)"""
        roll, pitch, yaw = self.quat_to_euler(q)
        return roll * 180/math.pi, pitch * 180/math.pi, yaw * 180/math.pi
    
    def normalize_angle(self, angle):
        """Normalize angle to -180...180"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def transform_quat(self, q_imu, base_roll, base_pitch, base_yaw, 
                       offset_roll, offset_pitch, offset_yaw):
        """Transform quaternion với base + offset"""
        total_roll = base_roll + offset_roll
        total_pitch = base_pitch + offset_pitch
        total_yaw = base_yaw + offset_yaw
        
        roll = total_roll * math.pi / 180
        pitch = total_pitch * math.pi / 180
        yaw = total_yaw * math.pi / 180
        
        q_rot = self.euler_to_quat(roll, pitch, yaw)
        q_imu = self.quat_normalize(q_imu)
        q_rot = self.quat_normalize(q_rot)
        
        q_temp = self.quat_mult(q_rot, q_imu)
        q_result = self.quat_mult(q_temp, self.quat_conj(q_rot))
        
        return self.quat_normalize(q_result)
    
    def transform_quat_to_baselink(self, q_imu, imu_name):
        """
        ✅ Transform quaternion từ IMU khung sang Baselink
        Giống y hệt test_imu_fusion_simple.py
        """
        if imu_name == "left":
            # LEFT: Quay -90° quanh Z
            angle = -math.pi / 2
            axis = np.array([0, 0, 1])
        else:  # right
            # RIGHT: Quay +90° quanh Z
            angle = math.pi / 2
            axis = np.array([0, 0, 1])
        
        half_angle = angle / 2
        sin_half = math.sin(half_angle)
        cos_half = math.cos(half_angle)
        
        q_rot = [
            cos_half,
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half
        ]
        
        q_rot = self.quat_normalize(q_rot)
        q_imu = self.quat_normalize(q_imu)
        
        q_temp = self.quat_mult(q_rot, q_imu)
        q_baselink = self.quat_mult(q_temp, self.quat_conj(q_rot))
        
        return self.quat_normalize(q_baselink)
    
    def transform_quat_test(self, q_imu, imu_name, offset_roll, offset_pitch, offset_yaw):
        """
        ✅ KHỚP với test_imu_fusion_simple.py
        Áp dụng: base transform (fixed -90°/+90° Z) + offset
        """
        if imu_name == "left":
            # Base: -90° quanh Z
            base_angle = -math.pi / 2
            axis = np.array([0, 0, 1])
        else:  # right
            # Base: +90° quanh Z
            base_angle = math.pi / 2
            axis = np.array([0, 0, 1])
        
        # Tạo base quaternion từ axis-angle
        half_angle = base_angle / 2
        sin_half = math.sin(half_angle)
        cos_half = math.cos(half_angle)
        
        q_base = [
            cos_half,
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half
        ]
        
        # Thêm offset dưới dạng Euler
        offset_quat = self.euler_to_quat(
            offset_roll * math.pi / 180,
            offset_pitch * math.pi / 180,
            offset_yaw * math.pi / 180
        )
        
        # q_total = q_offset * q_base (kết hợp)
        q_total = self.quat_mult(offset_quat, q_base)
        q_total = self.quat_normalize(q_total)
        
        # Áp dụng: q_result = q_total * q_imu * q_total^-1
        q_imu = self.quat_normalize(q_imu)
        q_temp = self.quat_mult(q_total, q_imu)
        q_baselink = self.quat_mult(q_temp, self.quat_conj(q_total))
        
        return self.quat_normalize(q_baselink)

    def test_offset_search_v2(self):
        """
        TEST: Search offset nhỏ ±10° quanh base transform
        """
        print("\n" + "=" * 80)
        print("🔍 TEST: OFFSET FINE-TUNE (±10° around base, 1° step)")
        print("=" * 80)
        
        print("\n⏳ Waiting for servers...")
        time.sleep(3)
        
        # Thu thập dữ liệu - 500 frames
        print("\n📊 Collecting 500 frames (25 seconds)...")
        left_raws = []
        right_raws = []
        timestamps = []
        start_time = time.time()
        
        frame_count = 0
        for i in range(500):
            left_raw = self.left.read_imu()
            right_raw = self.right.read_imu()
            
            if left_raw and right_raw:
                # ✅ LƯU raw (chưa transform)
                left_raws.append(self.quat_normalize(left_raw))
                right_raws.append(self.quat_normalize(right_raw))
                timestamps.append(time.time() - start_time)
                frame_count += 1
                
                if frame_count % 50 == 0:
                    print(f"\r  Frame {frame_count}/500 ({frame_count*0.05:.1f}s)", end="")
            time.sleep(0.05)
        
        elapsed_time = time.time() - start_time
        print(f"\n\n  ✓ Collected {len(left_raws)} frames in {elapsed_time:.1f} seconds")
        
        # 🔍 DRIFT ANALYSIS với transform (chỉ để theo dõi)
        print("\n🔍 DRIFT ANALYSIS:")
        print("   Chia dữ liệu thành 5 chunks để theo dõi drift theo thời gian")
        
        chunk_size = len(left_raws) // 5
        left_euler_chunks = []
        right_euler_chunks = []
        
        for chunk_idx in range(5):
            start_idx = chunk_idx * chunk_size
            end_idx = start_idx + chunk_size
            
            # Tính trung bình chunk (raw)
            left_chunk_avg = [sum(left_raws[i][j] for i in range(start_idx, end_idx)) / chunk_size 
                             for j in range(4)]
            right_chunk_avg = [sum(right_raws[i][j] for i in range(start_idx, end_idx)) / chunk_size 
                              for j in range(4)]
            
            left_chunk_avg = self.quat_normalize(left_chunk_avg)
            right_chunk_avg = self.quat_normalize(right_chunk_avg)
            
            # ✅ Transform để xem drift
            left_transformed = self.transform_quat_test(left_chunk_avg, "left", 0, 0, 0)
            right_transformed = self.transform_quat_test(right_chunk_avg, "right", 0, 0, 0)
            
            left_euler_chunks.append(self.quat_to_euler_deg(left_transformed))
            right_euler_chunks.append(self.quat_to_euler_deg(right_transformed))
        
        print(f"\n   LEFT Yaw drift (AFTER BASE TRANSFORM):")
        for i, euler in enumerate(left_euler_chunks):
            t = timestamps[i * chunk_size] if i * chunk_size < len(timestamps) else elapsed_time
            print(f"      Chunk {i} (t={t:5.1f}s): Roll={euler[0]:+7.2f}° Pitch={euler[1]:+7.2f}° Yaw={euler[2]:+7.2f}°")
        
        left_yaw_drift = left_euler_chunks[-1][2] - left_euler_chunks[0][2]
        left_roll_drift = left_euler_chunks[-1][0] - left_euler_chunks[0][0]
        print(f"      Total Yaw drift: {left_yaw_drift:+.2f}° over {elapsed_time:.1f}s")
        print(f"      Total Roll drift: {left_roll_drift:+.2f}° over {elapsed_time:.1f}s")
        
        print(f"\n   RIGHT Yaw drift (AFTER BASE TRANSFORM):")
        for i, euler in enumerate(right_euler_chunks):
            t = timestamps[i * chunk_size] if i * chunk_size < len(timestamps) else elapsed_time
            print(f"      Chunk {i} (t={t:5.1f}s): Roll={euler[0]:+7.2f}° Pitch={euler[1]:+7.2f}° Yaw={euler[2]:+7.2f}°")
        
        right_yaw_drift = right_euler_chunks[-1][2] - right_euler_chunks[0][2]
        right_roll_drift = right_euler_chunks[-1][0] - right_euler_chunks[0][0]
        print(f"      Total Yaw drift: {right_yaw_drift:+.2f}° over {elapsed_time:.1f}s")
        print(f"      Total Roll drift: {right_roll_drift:+.2f}° over {elapsed_time:.1f}s")
        
        # Tính trung bình toàn bộ dữ liệu (raw)
        left_avg = [sum(left_raws[i][j] for i in range(len(left_raws))) / len(left_raws) 
                    for j in range(4)]
        right_avg = [sum(right_raws[i][j] for i in range(len(right_raws))) / len(right_raws) 
                     for j in range(4)]
        
        left_avg = self.quat_normalize(left_avg)
        right_avg = self.quat_normalize(right_avg)
        
        # 🔧 Search LEFT offset
        print("\n\n🔧 Searching LEFT offset (±10° around base, 1° step)...")
        
        best_left_error = float('inf')
        best_left_offset = (0, 0, 0)
        
        count = 0
        total = 21 * 21 * 21  # ±10° with 1° step
        
        for droll in range(-10, 11):
            for dpitch in range(-10, 11):
                for dyaw in range(-10, 11):
                    # ✅ Dùng transform_quat_test (khớp với production code)
                    q_transformed = self.transform_quat_test(
                        left_avg, "left",
                        droll, dpitch, dyaw
                    )
                    euler = self.quat_to_euler_deg(q_transformed)
                    
                    # Mục tiêu: Roll ≈ 0°, Pitch ≈ 0°, Yaw ≈ 0°
                    error = abs(euler[0]) + abs(self.normalize_angle(euler[1])) + abs(self.normalize_angle(euler[2]))
                    
                    if error < best_left_error:
                        best_left_error = error
                        best_left_offset = (droll, dpitch, dyaw)
                    
                    count += 1
                    if count % 500 == 0:
                        print(f"\r  Progress: {count}/{total} ... Best error: {best_left_error:.2f}°", end="")
        
        print(f"\n  ✓ Best LEFT: ΔRoll={best_left_offset[0]:+d}° ΔPitch={best_left_offset[1]:+d}° ΔYaw={best_left_offset[2]:+d}° error={best_left_error:.2f}°")
        
        # 🔧 Search RIGHT offset
        print("\n🔧 Searching RIGHT offset (±10° around base, 1° step)...")
        
        best_right_error = float('inf')
        best_right_offset = (0, 0, 0)
        
        count = 0
        
        for droll in range(-10, 11):
            for dpitch in range(-10, 11):
                for dyaw in range(-10, 11):
                    q_transformed = self.transform_quat_test(
                        right_avg, "right",
                        droll, dpitch, dyaw
                    )
                    euler = self.quat_to_euler_deg(q_transformed)
                    
                    error = abs(euler[0]) + abs(self.normalize_angle(euler[1])) + abs(self.normalize_angle(euler[2]))
                    
                    if error < best_right_error:
                        best_right_error = error
                        best_right_offset = (droll, dpitch, dyaw)
                    
                    count += 1
                    if count % 500 == 0:
                        print(f"\r  Progress: {count}/{total} ... Best error: {best_right_error:.2f}°", end="")
        
        print(f"\n  ✓ Best RIGHT: ΔRoll={best_right_offset[0]:+d}° ΔPitch={best_right_offset[1]:+d}° ΔYaw={best_right_offset[2]:+d}° error={best_right_error:.2f}°")
        
        return (0, 0, 0, best_left_offset, best_left_error,
                0, 0, 0, best_right_offset, best_right_error,
                left_yaw_drift, right_yaw_drift, elapsed_time)
    
    def run_full_diagnostic(self):
        """Chạy diagnostic"""
        try:
            result = self.test_offset_search_v2()
            
            (_, _, _, left_offset, left_err,
             _, _, _, right_offset, right_err,
             left_drift, right_drift, total_time) = result
            
            # 📋 Summary
            print("\n\n" + "=" * 80)
            print("📋 FINAL SUMMARY - CALIBRATED OFFSETS")
            print("=" * 80)
            
            print(f"\n✅ ADD THESE OFFSETS TO test_imu_fusion_simple.py:")
            
            print(f"\n   LEFT IMU:")
            print(f"      Base: -90° (Z-axis rotation)")
            print(f"      Offset: ΔRoll={left_offset[0]:+d}°, ΔPitch={left_offset[1]:+d}°, ΔYaw={left_offset[2]:+d}°")
            print(f"      Error: {left_err:.2f}°")
            print(f"      Drift: {left_drift:+.2f}° over {total_time:.1f}s ({left_drift/total_time:.3f}°/s)")
            
            print(f"\n   RIGHT IMU:")
            print(f"      Base: +90° (Z-axis rotation)")
            print(f"      Offset: ΔRoll={right_offset[0]:+d}°, ΔPitch={right_offset[1]:+d}°, ΔYaw={right_offset[2]:+d}°")
            print(f"      Error: {right_err:.2f}°")
            print(f"      Drift: {right_drift:+.2f}° over {total_time:.1f}s ({right_drift/total_time:.3f}°/s)")
            
            print(f"\n📝 To use offsets, add them to Euler angles BEFORE quaternion conversion:")
            print(f"   (See test_imu_fusion_simple.py - update left_rot/right_rot calculations)")
            
            print("\n" + "=" * 80)
            
        except KeyboardInterrupt:
            print("\n\n👋 Stopped!")
        except Exception as e:
            print(f"\n❌ Error: {e}")
            import traceback
            traceback.print_exc()


class IMUController:
    """Giống từ trước"""
    
    def __init__(self, host: str, port: int, side: str = "LEFT"):
        self.host = host
        self.port = port
        self.side = side
        self.context = zmq.Context()
        self.socket = None
        self.connect()
    
    def connect(self):
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, 5000)
        self.socket.setsockopt(zmq.LINGER, 0)
        
        try:
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            print(f"✓ {self.side} IMU: {self.host}:{self.port}")
        except Exception as e:
            print(f"✗ {self.side} failed: {e}")
            raise
    
    def read_imu(self):
        try:
            self.socket.send_json({"type": "feedback"})
            response = self.socket.recv_json()
            imu_data = response.get("imu", [1, 0, 0, 0])
            
            if len(imu_data) == 4:
                return imu_data
            return None
        except:
            self.connect()
            return None


def main():
    try:
        diag = IMUOffsetFineTune(
            left_host="mobile2.local",
            left_port=5556,
            right_host="mobile1.local",
            right_port=5555
        )
        diag.run_full_diagnostic()
    except Exception as e:
        print(f"❌ Failed: {e}")


if __name__ == "__main__":
    main()