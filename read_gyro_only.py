import qwiic_icm20948
import json
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)

# ==========================================
# 1. ĐỌC DỮ LIỆU CALIB TỪ FILE CỦA BẠN
# ==========================================
try:
    with open("vinhgyrocalib.json", "r") as f:
        calib = json.load(f)
    
    gx_bias = calib.get("gx_bias", 0)
    gy_bias = calib.get("gy_bias", 0)
    gz_bias = calib.get("gz_bias", 0)
    GYRO_SENSITIVITY = calib.get("gyro_sensitivity", 65.536)
    logger.info("✅ Đã tải dữ liệu calib từ vinhgyrocalib.json thành công!")
except FileNotFoundError:
    logger.warning("⚠️ Không tìm thấy file vinhgyrocalib.json! Đang dùng thông số mặc định (Bias = 0).")
    gx_bias, gy_bias, gz_bias = 0, 0, 0
    GYRO_SENSITIVITY = 65.536

# ==========================================
# 2. KHỞI TẠO CẢM BIẾN IMU
# ==========================================
logger.info("Đang kết nối IMU...")
IMU = qwiic_icm20948.QwiicIcm20948()

if not IMU.connected:
    logger.error("❌ Không tìm thấy IMU! Hãy kiểm tra lại dây cắm I2C.")
    exit(1)

IMU.begin()
logger.info("✅ Cảm biến đã sẵn sàng!")
logger.info("Bắt đầu đọc tốc độ xoay (Nhấn Ctrl+C để thoát)...\n")

# ==========================================
# 3. VÒNG LẶP ĐỌC GYRO (CHỈ GYRO)
# ==========================================
try:
    while True:
        if IMU.dataReady():
            IMU.getAgmt()
            
            # Đọc số thô của Gyro
            gx_raw = IMU.gxRaw
            gy_raw = IMU.gyRaw
            gz_raw = IMU.gzRaw
            
            # Trừ sai số (Bias) và chia độ nhạy để ra Độ/Giây (°/s)
            gx_deg = (gx_raw - gx_bias) / GYRO_SENSITIVITY
            gy_deg = (gy_raw - gy_bias) / GYRO_SENSITIVITY
            gz_deg = (gz_raw - gz_bias) / GYRO_SENSITIVITY
            
            # In ra màn hình (dùng end='\r' để nó cập nhật trên cùng 1 dòng cho gọn)
            print(f"Tốc độ xoay (°/s):  X = {gx_deg:+.2f}   |   Y = {gy_deg:+.2f}   |   Z = {gz_deg:+.2f}      ", end='\r')
            
            time.sleep(0.02) # 50 lần / giây (50Hz)
            
except KeyboardInterrupt:
    print("\n\nĐã dừng chương trình. Chúc bạn vui vẻ!")
