import qwiic_icm20948
import time
import sys

def main():
    print("Khởi tạo cảm biến ICM-20948...")
    IMU = qwiic_icm20948.QwiicIcm20948()
    
    if not IMU.connected:
        print("Lỗi: Không tìm thấy cảm biến IMU, vui lòng kiểm tra lại kết nối I2C.")
        sys.exit(1)
        
    IMU.begin()
    print("✓ Khởi tạo thành công!")
    print("==================================================")
    print("Script kiểm tra trục cảm biến (Accel & Gyro)")
    print("Bấm Ctrl+C để thoát")
    print("==================================================")
    time.sleep(2)

    try:
        while True:
            if IMU.dataReady():
                IMU.getAgmt()
                
                # Đọc dữ liệu Raw Accel
                ax_raw = IMU.axRaw
                ay_raw = IMU.ayRaw
                az_raw = IMU.azRaw
                
                # Đọc dữ liệu Raw Gyro
                gx_raw = IMU.gxRaw
                gy_raw = IMU.gyRaw
                gz_raw = IMU.gzRaw
                
                # Chuyển đổi Accel sang đơn vị g (1g = 16384 theo config mặc định của ICM-20948)
                # Tùy config mà độ phân giải có thể khác, nhưng ta dùng 16384 để ước lượng
                ax_g = ax_raw / 16384.0
                ay_g = ay_raw / 16384.0
                az_g = az_raw / 16384.0

                # Clear màn hình terminal (dùng \033[H\033[J hoặc in đè)
                # Ta in đè dòng bằng \r để dễ nhìn
                print(f"\rAccel (g): X={ax_g:>6.2f} | Y={ay_g:>6.2f} | Z={az_g:>6.2f}    "
                      f"|| Gyro (Raw): X={gx_raw:>6} | Y={gy_raw:>6} | Z={gz_raw:>6}", end="")
                
            time.sleep(0.1) # Dừng 100ms để không in quá nhanh
            
    except KeyboardInterrupt:
        print("\nĐã thoát chương trình.")

if __name__ == '__main__':
    main()
