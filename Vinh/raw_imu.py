#FILE NÀY ĐỂ CHẠY TRÊN PI NHẰM TEST CÁC TRỤC CỦA IMU 
import time
import qwiic_icm20948

print("Đang khởi tạo cảm biến ICM20948...")
IMU = qwiic_icm20948.QwiicIcm20948()

if not IMU.connected:
    print(" Không tìm thấy IMU. Vui lòng kiểm tra lại dây cáp!")
    exit()

IMU.begin()
print(" Sẵn sàng! Hãy thử lật ngửa, lật nghiêng con chip để xem trục nào phản ứng.")
print(" Mẹo: Trục nào chỉa thẳng lên trời sẽ có giá trị xấp xỉ 1.0 (hoặc -1.0)\n")

SENSITIVITY = 16384.0  # Hệ số chia để ra đơn vị 1g

try:
    while True:
        if IMU.dataReady():
            IMU.getAgmt()
            
            # Chia cho SENSITIVITY để hiển thị ra đơn vị g (+/- 1.0)
            ax = IMU.axRaw / SENSITIVITY
            ay = IMU.ayRaw / SENSITIVITY
            az = IMU.azRaw / SENSITIVITY
            
            # In ra màn hình (Ghi đè liên tục để dễ nhìn)
            print(f"Gia tốc (g)    X: {ax:+6.2f}  |  Y: {ay:+6.2f}  |  Z: {az:+6.2f}", end='\r')
            
        time.sleep(0.1)  # Đọc 10 lần 1 giây là đủ để mắt nhìn

except KeyboardInterrupt:
    print("\n\n Đã dừng chương trình!")