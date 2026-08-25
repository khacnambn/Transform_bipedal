import qwiic_icm20948
import math
import numpy as np
from ahrs.filters import Madgwick
import json
import asyncio
import websockets

# 1. TẢI CALIB
try:
    with open("vinhgyrocalib.json", "r") as f:
        calib = json.load(f)
    gx_bias = calib.get("gx_bias", 0)
    gy_bias = calib.get("gy_bias", 0)
    gz_bias = calib.get("gz_bias", 0)
    GYRO_SENSITIVITY = calib.get("gyro_sensitivity", 65.536)
except:
    gx_bias, gy_bias, gz_bias = 0, 0, 0
    GYRO_SENSITIVITY = 65.536

# 2. KHỞI TẠO IMU VÀ MADGWICK
IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    print("Không tìm thấy IMU!")
    exit(1)
IMU.begin()

madgwick = Madgwick(frequency=50, beta=0.1)
madgwick.q0 = np.array([1.0, 0.0, 0.0, 0.0])

ACCEL_ALPHA = 0.15
GYRO_ALPHA = 0.15
filtered_ax, filtered_ay, filtered_az = 0.0, 0.0, 0.0
filtered_gx, filtered_gy, filtered_gz = 0.0, 0.0, 0.0
SENSITIVITY = 16384.0
G = 9.81

# 3. HÀM XỬ LÝ GỬI DỮ LIỆU QUA WEB
async def imu_handler(websocket):
    global filtered_ax, filtered_ay, filtered_az, filtered_gx, filtered_gy, filtered_gz
    print("🎉 Có Laptop vừa kết nối vào xem 3D!")
    try:
        while True:
            if IMU.dataReady():
                IMU.getAgmt()
                
                ax = (IMU.axRaw / SENSITIVITY) * G
                ay = (IMU.ayRaw / SENSITIVITY) * G
                az = (IMU.azRaw / SENSITIVITY) * G
                
                gx = ((IMU.gxRaw - gx_bias) / GYRO_SENSITIVITY) * (math.pi / 180)
                gy = ((IMU.gyRaw - gy_bias) / GYRO_SENSITIVITY) * (math.pi / 180)
                gz = ((IMU.gzRaw - gz_bias) / GYRO_SENSITIVITY) * (math.pi / 180)
                
                filtered_ax = ACCEL_ALPHA * ax + (1 - ACCEL_ALPHA) * filtered_ax
                filtered_ay = ACCEL_ALPHA * ay + (1 - ACCEL_ALPHA) * filtered_ay
                filtered_az = ACCEL_ALPHA * az + (1 - ACCEL_ALPHA) * filtered_az
                filtered_gx = GYRO_ALPHA * gx + (1 - GYRO_ALPHA) * filtered_gx
                filtered_gy = GYRO_ALPHA * gy + (1 - GYRO_ALPHA) * filtered_gy
                filtered_gz = GYRO_ALPHA * gz + (1 - GYRO_ALPHA) * filtered_gz
                
                q_new = madgwick.updateIMU(
                    q=madgwick.q0,
                    gyr=np.array([filtered_gx, filtered_gy, filtered_gz]),
                    acc=np.array([filtered_ax, filtered_ay, filtered_az])
                )
                madgwick.q0 = q_new
                
                # Gói 4 con số vào JSON và bắn đi
                data = {"w": q_new[0], "x": q_new[1], "y": q_new[2], "z": q_new[3]}
                await websocket.send(json.dumps(data))
                
            await asyncio.sleep(0.02) # Chạy 50Hz
    except websockets.exceptions.ConnectionClosed:
        print(" Laptop đã ngắt kết nối.")

async def main():
    print("==================================================")
    print("🚀 IMU WebSocket Server đã sẵn sàng ở cổng 8765")
    print("Đang chờ bạn mở file HTML trên Laptop...")
    print("==================================================")
    # Khởi động Server (0.0.0.0 nghĩa là cho phép mọi máy tính kết nối)
    async with websockets.serve(imu_handler, "0.0.0.0", 8765):
        await asyncio.Future()  # Chạy mãi mãi

if __name__ == "__main__":
    asyncio.run(main())
