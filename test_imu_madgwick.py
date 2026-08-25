import qwiic_icm20948
import math
import numpy as np
from ahrs.filters import Madgwick
import json
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

SENSITIVITY = 16384.0  # Accelerometer
G = 9.81 #gia tốc trọng trường của trái đất 

# Load calibration
with open("vinhgyrocalib.json", "r") as f:
    calib = json.load(f)

#TẠM THỜI BỎ QUA PHẦN CALIB ACCEL 
#  Accelerometer calibration
# ax_bias = calib["ax_bias"]
# ay_bias = calib["ay_bias"]
# az_bias = calib["az_bias"]

# ax_scale = calib["ax_scale"]
# ay_scale = calib["ay_scale"]
# az_scale = calib["az_scale"]

ax_bias = 0
ay_bias = 0
az_bias = 0

ax_scale = 1
ay_scale = 1
az_scale = 1


#Lấy thông số bias và sensitivity của gyro từ file calib
gx_bias = calib.get("gx_bias", 0)
gy_bias = calib.get("gy_bias", 0)
gz_bias = calib.get("gz_bias", 0)
GYRO_SENSITIVITY = calib.get("gyro_sensitivity", 65.536)  # ±500°/s range

# Low-pass filter
ACCEL_ALPHA = 0.15
GYRO_ALPHA = 0.15

filtered_ax, filtered_ay, filtered_az = 0.0, 0.0, 0.0
filtered_gx, filtered_gy, filtered_gz = 0.0, 0.0, 0.0

# Initialize IMU
logger.info("Initializing ICM-20948 IMU...")
IMU = qwiic_icm20948.QwiicIcm20948()

if not IMU.connected:
    logger.error("IMU not found!")
    exit(1)

IMU.begin()
logger.info("✓ ICM-20948 initialized")
logger.info(f"✓ Gyroscope Range: ±500°/s (SENSITIVITY = {GYRO_SENSITIVITY})")

# Initialize Madgwick filter
madgwick = Madgwick(frequency=50, beta=0.1)
madgwick.q0 = np.array([1.0, 0.0, 0.0, 0.0])

logger.info("✓ Madgwick filter initialized\n")

try:
    sample_count = 0
    while True:
        if IMU.dataReady():
            IMU.getAgmt()
            
            #Đọc dữ liệu thô từ cảm biến 
            ax_raw, ay_raw, az_raw = IMU.axRaw, IMU.ayRaw, IMU.azRaw
            gx_raw, gy_raw, gz_raw = IMU.gxRaw, IMU.gyRaw, IMU.gzRaw
            
            # TẠM THỜI BỎ QUA PHẦN CALIB ACCEL 
            # ax_lsb = (ax_raw - ax_bias) / ax_scale
            # ay_lsb = (ay_raw - ay_bias) / ay_scale
            # az_lsb = (az_raw - az_bias) / az_scale

            ax_lsb = ax_raw / ax_scale
            ay_lsb = ay_raw / ay_scale
            az_lsb = az_raw / az_scale  
            
            ax = ax_lsb / SENSITIVITY * G
            ay = ay_lsb / SENSITIVITY * G
            az = az_lsb / SENSITIVITY * G
            
            #  Calibrate and convert gyroscope (CORRECT WAY)
            gx_deg = (gx_raw - gx_bias) / GYRO_SENSITIVITY
            gy_deg = (gy_raw - gy_bias) / GYRO_SENSITIVITY
            gz_deg = (gz_raw - gz_bias) / GYRO_SENSITIVITY
            
            gx = gx_deg * math.pi / 180
            gy = gy_deg * math.pi / 180
            gz = gz_deg * math.pi / 180
            
            # Low-pass filter
            filtered_ax = ACCEL_ALPHA * ax + (1 - ACCEL_ALPHA) * filtered_ax
            filtered_ay = ACCEL_ALPHA * ay + (1 - ACCEL_ALPHA) * filtered_ay
            filtered_az = ACCEL_ALPHA * az + (1 - ACCEL_ALPHA) * filtered_az
            
            filtered_gx = GYRO_ALPHA * gx + (1 - GYRO_ALPHA) * filtered_gx
            filtered_gy = GYRO_ALPHA * gy + (1 - GYRO_ALPHA) * filtered_gy
            filtered_gz = GYRO_ALPHA * gz + (1 - GYRO_ALPHA) * filtered_gz
            
            # Update Madgwick filter
            q_new = madgwick.updateIMU(
                q=madgwick.q0,
                gyr=np.array([filtered_gx, filtered_gy, filtered_gz]),
                acc=np.array([filtered_ax, filtered_ay, filtered_az])
            )
            madgwick.q0 = q_new

            qw, qx, qy, qz = q_new
                
             # Tính Roll (X-axis) - Nghiêng trái/phải
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)
    
            # Tính Pitch (Y-axis) - Ngả tới/lui
            sinp = 2 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = math.copysign(90.0, sinp) # Giới hạn ở 90 độ
            else:
                pitch = math.asin(sinp) * (180.0 / math.pi)

            # Tính Yaw (Z-axis) - Xoay vòng quanh trục
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

            sample_count += 1
            
            # Print results
            print(f"\n{'='*70}")
            print(f"Sample #{sample_count}")
            print(f"{'='*70}")
            
            print(f"\n[RAW DATA]")
            print(f"  Accelerometer (raw):  ax={ax_raw:6.0f}, ay={ay_raw:6.0f}, az={az_raw:6.0f}")
            print(f"  Gyroscope (raw):      gx={gx_raw:7.2f} LSB, gy={gy_raw:7.2f} LSB, gz={gz_raw:7.2f} LSB")
            
            print(f"\n[CALIBRATED DATA]")
            print(f"  Accelerometer (m/s²): ax={ax:+.4f}, ay={ay:+.4f}, az={az:+.4f}")
            print(f"  Gyroscope (°/s):      gx={gx_deg:+.2f}, gy={gy_deg:+.2f}, gz={gz_deg:+.2f}")
            print(f"  Gyroscope (rad/s):    gx={filtered_gx:+.4f}, gy={filtered_gy:+.4f}, gz={filtered_gz:+.4f}")
            
            print(f"\n[MADGWICK QUATERNION]")
            print(f"  q = [{q_new[0]:+.6f}, {q_new[1]:+.6f}, {q_new[2]:+.6f}, {q_new[3]:+.6f}]")
            print(f"  (w, x, y, z)")

            print(f"\n[GÓC EULER THỰC TẾ (Độ)]")
            print(f"  Roll (Nghiêng trái/phải): {roll:+.2f}°")
            print(f"  Pitch (Ngả tới/lui):      {pitch:+.2f}°")
            print(f"  Yaw (Xoay tròn):          {yaw:+.2f}°")
            
            time.sleep(0.02)
            
except KeyboardInterrupt:
    logger.info("\nStopped by user")
    print(f"\n{'='*70}")
    print(f"Total samples read: {sample_count}")
    print(f"{'='*70}")