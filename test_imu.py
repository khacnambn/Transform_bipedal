import qwiic_icm20948
import time
import math
import json

# ==============================
# Load calibration
# ==============================
with open("imu_calib.json", "r") as f:
    calib = json.load(f)

ax_bias = calib["ax_bias"]
ay_bias = calib["ay_bias"]
az_bias = calib["az_bias"]

ax_scale = calib["ax_scale"]
ay_scale = calib["ay_scale"]
az_scale = calib["az_scale"]

gx_bias = calib.get("gx_bias", 0)
gy_bias = calib.get("gy_bias", 0)
gz_bias = calib.get("gz_bias", 0)

ACCEL_SENSITIVITY = 16384.0
GYRO_SENSITIVITY = calib.get("gyro_sensitivity", 65.536)
G = 9.81

print("Loaded calibration:")
print(calib)

IMU = qwiic_icm20948.QwiicIcm20948()

if not IMU.connected:
    print("Không tìm thấy IMU!")
    exit()

IMU.begin()
print("ICM-20948 ready (calibrated, output in m/s² and °/s)")
print()

# ✅ THÊM: Variables để tích phân YAW
last_time = time.time()
yaw = 0.0  # Current yaw angle in degrees

print("=" * 120)
print("Starting IMU reading with YAW integration...")
print("=" * 120)
print()

while True:
    if IMU.dataReady():
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        IMU.getAgmt()

        # ===== ACCELEROMETER (tính Roll, Pitch) =====
        ax_raw = IMU.axRaw
        ay_raw = IMU.ayRaw
        az_raw = IMU.azRaw

        ax_lsb = (ax_raw - ax_bias) / ax_scale
        ay_lsb = (ay_raw - ay_bias) / ay_scale
        az_lsb = (az_raw - az_bias) / az_scale

        ax = ax_lsb / ACCEL_SENSITIVITY * G
        ay = ay_lsb / ACCEL_SENSITIVITY * G
        az = az_lsb / ACCEL_SENSITIVITY * G

        # ✅ Roll & Pitch từ accelerometer (đã là degree)
        roll_deg = math.atan2(ay, az) * 180 / math.pi
        pitch_deg = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi

        # ===== GYROSCOPE (tích phân YAW) =====
        gx_raw = IMU.gxRaw
        gy_raw = IMU.gyRaw
        gz_raw = IMU.gzRaw

        # ✅ THÊM: Áp dụng bias
        gx_deg_s = (gx_raw - gx_bias) / GYRO_SENSITIVITY
        gy_deg_s = (gy_raw - gy_bias) / GYRO_SENSITIVITY
        gz_deg_s = (gz_raw - gz_bias) / GYRO_SENSITIVITY

        # ✅ THÊM: Tích phân YAW từ gyro Z
        yaw += gz_deg_s * dt  # Tích phân: d(yaw) = gz * dt

        # Normalize yaw về [-180, 180]
        while yaw > 180:
            yaw -= 360
        while yaw < -180:
            yaw += 360

        # ===== IN KẾT QUẢ =====
        print(
            f"Roll={roll_deg:+7.2f}°  "
            f"Pitch={pitch_deg:+7.2f}°  "
            f"Yaw={yaw:+7.2f}°  |  "
            f"ax={ax:+7.3f} m/s²  "
            f"ay={ay:+7.3f} m/s²  "
            f"az={az:+7.3f} m/s²  |  "
            f"gx={gx_deg_s:+6.2f}°/s  "
            f"gy={gy_deg_s:+6.2f}°/s  "
            f"gz={gz_deg_s:+6.2f}°/s"
        )

    time.sleep(0.02)