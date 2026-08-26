import qwiic_icm20948
import time
import json
import numpy as np
import sys

# CÁC THÔNG SỐ ĐỂ CALIB ACCEL
SAMPLES_PER_FACE = 500  # Số mẫu thu thập cho mỗi mặt (tư thế)
SAMPLE_RATE = 0.02  # Tốc độ đọc (20ms = 50Hz)
G_REFERENCE = 9.80665  # Gia tốc trọng trường chuẩn (m/s^2)

# mảng orientations: trục x: 0, trục y: 1, trục z: 2. sign: 1 hướng lên -1 hướng xuống
ORIENTATIONS = [
    {"name": "X HƯỚNG LÊN TRÊN (X+)", "axis": 0, "sign": 1},
    {"name": "X HƯỚNG XUỐNG DƯỚI (X-)", "axis": 0, "sign": -1},
    {"name": "Y HƯỚNG LÊN TRÊN (Y+)", "axis": 1, "sign": 1},
    {"name": "Y HƯỚNG XUỐNG DƯỚI (Y-)", "axis": 1, "sign": -1},
    {"name": "Z HƯỚNG LÊN TRÊN (Z+)", "axis": 2, "sign": 1},
    {"name": "Z HƯỚNG XUỐNG DƯỚI (Z-)", "axis": 2, "sign": -1},
]

# KHỞI TẠO CẢM BIẾN
IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    print("Không tìm thấy IMU!")
    sys.exit(1)
IMU.begin()

# MA TRẬN M VÀ R: M là Measurment , R là reference
total_measurements = len(ORIENTATIONS) * SAMPLES_PER_FACE
M = np.zeros((3 * total_measurements, 12))
R = np.zeros((3 * total_measurements, 1))

print("\n=== CHƯƠNG TRÌNH HIỆU CHUẨN GIA TỐC KẾ (ACCELEROMETER) ===")

for face in ORIENTATIONS:
    # 3.1: Hướng dẫn người dùng đặt tư thế
    print(f"\n> Hãy xoay IMU sao cho trục {face['name']} dọc theo chiều trọng lực.")
    input("  Nhấn phím [ENTER] khi đã đặt yên vị trí để bắt đầu đo...")
    print(f"  Đang đọc {SAMPLES_PER_FACE} mẫu...")

    samples_collected = 0
    while (
        samples_collected < SAMPLES_PER_FACE
    ):  # samples collected được cộng dồn trong vòng lặp bên dưới, khi đủ 500 mẫu thì thoát vòng while
        if IMU.dataReady():
            IMU.getAgmt()

            # LƯU Ý: Giá trị axRaw đọc từ ICM20948 đang là LSB (ví dụ +-2g -> /16384).
            # Đảm bảo chuyển về m/s^2. Nếu hàm trả về m/s^2 rồi thì không cần chuyển.
            ax = IMU.axRaw
            ay = IMU.ayRaw
            az = IMU.azRaw

            # 3.3: Lắp dữ liệu vào ma trận phương trình
            row_start = 3 * measurement_idx

            # Dòng phương trình cho trục X: ax*SM00 + ay*SM01 + az*SM02 - Bias_x = g (hoặc 0)
            M[row_start + 0, 0:3] = [ax, ay, az]
            M[row_start + 0, 9] = -1.0

            # Dòng phương trình cho trục Y: ax*SM10 + ay*SM11 + az*SM12 - Bias_y = 0 (hoặc g)
            M[row_start + 1, 3:6] = [ax, ay, az]
            M[row_start + 1, 10] = -1.0

            # Dòng phương trình cho trục Z: ax*SM20 + ay*SM21 + az*SM22 - Bias_z = 0 (hoặc g)
            M[row_start + 2, 6:9] = [ax, ay, az]
            M[row_start + 2, 11] = -1.0

            # Cài đặt giá trị mong đợi (Reference g) cho trục đang hướng thẳng đứng, 2 trục kia = 0
            # face['axis'] cho biết trục nào (0=X, 1=Y, 2=Z) đang chịu tác động của g
            R[row_start + face["axis"], 0] = face["sign"] * G_REFERENCE

            measurement_idx += 1
            samples_collected += 1

        time.sleep(SAMPLE_RATE / 2.0)  # Ngủ xíu chờ dữ liệu mới

    print("  Đã đọc xong mặt này!")

# TÍNH TOÁN LÍ THUYẾT
# Giải hệ phương trình M * X = R bằng thuật toán Least Squares của Numpy
# Biến x_hat sẽ trả về vector 12 phần tử tối ưu nhất
x_hat, residuals, rank, s = np.linalg.lstsq(M, R, rcond=None)
x_hat = x_hat.flatten()  # Ép thành mảng 1D cho dễ xử lý

# 9 phần tử đầu là ma trận SM (định dạng thành mảng 3x3)
SM = [
    [x_hat[0], x_hat[1], x_hat[2]],
    [x_hat[3], x_hat[4], x_hat[5]],
    [x_hat[6], x_hat[7], x_hat[8]],
]

# 3 phần tử cuối là Bias
Bias = [x_hat[9], x_hat[10], x_hat[11]]

print("\nTính toán xong! Ma trận SM và Bias:")
print("SM =", np.round(SM, 4).tolist())
print("Bias =", np.round(Bias, 4).tolist())

# --- BƯỚC 5: LƯU RA JSON ---
OUTPUT_FILE = "vinh_accel_calib.json"

calibration_data = {
    "accel": {
        "SM": float_list_to_standard(SM),  # Convert qua kiểu list thuần túy
        "bias": float_list_to_standard(Bias),
    }
}
