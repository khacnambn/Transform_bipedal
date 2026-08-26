import qwiic_icm20948
import json #thư viện giúp tạo và đọc file JSON
import time
import math

print(" KEEP IMU COMPLETELY STILL FOR 5 SECONDS!")
print("Don't touch or move it at all!\n")

IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    print("IMU not found!")
    exit(1)

IMU.begin()

# Đợi 5 giây để IMU ổn định
for i in range(5, 0, -1):
    print(f"Starting calibration in {i} seconds...")
    time.sleep(1)

SAMPLES = 1000 #Đọc dữ liệu 1000 lần 
SAMPLE_RATE = 0.02 # mỗi lần đọc cách nhau 20ms (50Hz)
#tổng cộng mất 20 giây để đọc dữ liệu 

print(f"\n Calibrating gyroscope (reading {SAMPLES} samples, ~{SAMPLES * SAMPLE_RATE:.0f}s)...\n")
 
gx_sum, gy_sum, gz_sum = 0, 0, 0 #biến sum để cộng dồn tổng dữ liệu
#biến min max để lưu giá trị lớn nhấtt và nhỏ nhất của dữ liệu  
gx_min, gx_max = float('inf'), float('-inf')
gy_min, gy_max = float('inf'), float('-inf')
gz_min, gz_max = float('inf'), float('-inf')

gx_data, gy_data, gz_data = [], [], [] #tạo mảng để đưa các con số đọc được vào đấy 

for i in range(SAMPLES):
    if IMU.dataReady():
        IMU.getAgmt()
        #Lấy giá trị raw của gyrscope 
        gx_raw = IMU.gxRaw
        gy_raw = IMU.gyRaw
        gz_raw = IMU.gzRaw
        #Cộng dồn giá trị raw của gyro 
        gx_sum += gx_raw
        gy_sum += gy_raw
        gz_sum += gz_raw

        #lưu giá trị raw vào mảng ở trên 
        gx_data.append(gx_raw)
        gy_data.append(gy_raw)
        gz_data.append(gz_raw)

        #hàm min max để tìm giá trị lớn nhất và nhỏ nhất của dữ liệu trong toàn bộ quá trình đọc dữ liêu, bằng cách so sánh gtri hiện tại với min max của vòng trc đó 
        gx_min, gx_max = min(gx_min, gx_raw), max(gx_max, gx_raw)
        gy_min, gy_max = min(gy_min, gy_raw), max(gy_max, gy_raw)
        gz_min, gz_max = min(gz_min, gz_raw), max(gz_max, gz_raw)

        #gom đủ 100 mẫu thì mới in lên 1 lần, tránh việc in quá nhiều lần làm chậm chương trình
        if (i + 1) % 100 == 0:
            percent = (i + 1) / SAMPLES * 100
            elapsed = (i + 1) * SAMPLE_RATE
            print(f"  {i + 1}/{SAMPLES} samples ({percent:.0f}%) - {elapsed:.1f}s elapsed...", end='\r')
    
    time.sleep(SAMPLE_RATE) #bắt chương trình nghỉ 0.02 giây giữa mỗi vòng lặp for 

print(f"\n   Sampling complete!                                            ")

#Tính trung bình cộng giá trị của dữ liệu đọc được, đây chính là bias của gyro
gx_bias = gx_sum / SAMPLES
gy_bias = gy_sum / SAMPLES
gz_bias = gz_sum / SAMPLES

#PHẦN NÀY ĐỂ TÍNH VARIANCE, KHÔNG LIÊN QUAN TỚI BIAS, CHỈ ĐỂ XEM DỮ LIỆU CÓ ỔN ĐỊNH HAY KHÔNG
gx_variance = sum((x - gx_bias)**2 for x in gx_data) / SAMPLES
gy_variance = sum((y - gy_bias)**2 for y in gy_data) / SAMPLES
gz_variance = sum((z - gz_bias)**2 for z in gz_data) / SAMPLES

gx_std = math.sqrt(gx_variance)
gy_std = math.sqrt(gy_variance)
gz_std = math.sqrt(gz_variance)

print(f"\n✅ Gyroscope Bias calculated:")
print(f"  gx_bias = {gx_bias:+.2f}")
print(f"  gy_bias = {gy_bias:+.2f}")
print(f"  gz_bias = {gz_bias:+.2f}")



# Xác định dải đo độ nhạy
max_val = max(abs(gx_min), abs(gx_max), abs(gy_min), abs(gy_max), abs(gz_min), abs(gz_max))

if max_val > 16384:
    GYRO_SENSITIVITY = 16.384
elif max_val > 8192:
    GYRO_SENSITIVITY = 32.768
else:
    GYRO_SENSITIVITY = 65.536

# TẠO MỚI DỮ LIỆU CALIB TỪ ĐẦU (Không đọc file cũ)
calib = {
    "gx_bias": gx_bias,
    "gy_bias": gy_bias,
    "gz_bias": gz_bias,
    "gyro_sensitivity": GYRO_SENSITIVITY
}

# Lưu trực tiếp vào thư mục hiện tại
OUTPUT_FILE = "vinhgyrocalib.json"
with open(OUTPUT_FILE, "w") as f:
    json.dump(calib, f, indent=2)

print(f"\n✓ Calibration saved to {OUTPUT_FILE}")
print(f"\n Summary:")
print(f"  • Samples: {SAMPLES}")
print(f"  • Duration: {SAMPLES * SAMPLE_RATE:.1f}s")
print(f"  • Gyro Sensitivity: {GYRO_SENSITIVITY}")
