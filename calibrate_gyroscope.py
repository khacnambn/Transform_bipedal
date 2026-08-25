import qwiic_icm20948
import json
import time
import math

# Load existing calibration
with open("/home/mobile2/leg2_bipedal/imu/imu_calib.json", "r") as f:
    calib = json.load(f)

print("🔴 KEEP IMU COMPLETELY STILL FOR 5 SECONDS!")
print("Don't touch or move it at all!\n")

IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    print("IMU not found!")
    exit(1)

IMU.begin()

# Wait for stability
for i in range(5, 0, -1):
    print(f"Starting calibration in {i} seconds...")
    time.sleep(1)

# ✅ THAY ĐỔI: Tăng samples từ 100 → 1000
SAMPLES = 1000  # ← Thay từ 100 thành 1000 (20 giây)
SAMPLE_RATE = 0.02  # 20ms = 50Hz

print(f"\n📊 Calibrating gyroscope (reading {SAMPLES} samples, ~{SAMPLES * SAMPLE_RATE:.0f}s)...\n")

gx_sum, gy_sum, gz_sum = 0, 0, 0
gx_min, gx_max = float('inf'), float('-inf')
gy_min, gy_max = float('inf'), float('-inf')
gz_min, gz_max = float('inf'), float('-inf')

# ✅ FIX: Store data để tính std dev
gx_data, gy_data, gz_data = [], [], []

for i in range(SAMPLES):
    if IMU.dataReady():
        IMU.getAgmt()
        gx_raw = IMU.gxRaw
        gy_raw = IMU.gyRaw
        gz_raw = IMU.gzRaw
        
        gx_sum += gx_raw
        gy_sum += gy_raw
        gz_sum += gz_raw
        
        # ✅ FIX: Store raw values
        gx_data.append(gx_raw)
        gy_data.append(gy_raw)
        gz_data.append(gz_raw)
        
        # Track min/max
        gx_min, gx_max = min(gx_min, gx_raw), max(gx_max, gx_raw)
        gy_min, gy_max = min(gy_min, gy_raw), max(gy_max, gy_raw)
        gz_min, gz_max = min(gz_min, gz_raw), max(gz_max, gz_raw)
        
        # ✅ THÊM: Progress bar chi tiết hơn
        if (i + 1) % 100 == 0:
            percent = (i + 1) / SAMPLES * 100
            elapsed = (i + 1) * SAMPLE_RATE
            print(f"  {i + 1}/{SAMPLES} samples ({percent:.0f}%) - {elapsed:.1f}s elapsed...", end='\r')
    
    time.sleep(SAMPLE_RATE)

print(f"\n  ✅ Sampling complete!                                            ")

# Calculate gyroscope bias
gx_bias = gx_sum / SAMPLES
gy_bias = gy_sum / SAMPLES
gz_bias = gz_sum / SAMPLES

# ✅ FIX: Tính Standard Deviation đúng cách
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

print(f"\n📊 Gyroscope Raw Range (LSB):")
print(f"  gx: {gx_min:+.0f} to {gx_max:+.0f} (range: {gx_max - gx_min:.0f})")
print(f"  gy: {gy_min:+.0f} to {gy_max:+.0f} (range: {gy_max - gy_min:.0f})")
print(f"  gz: {gz_min:+.0f} to {gz_max:+.0f} (range: {gz_max - gz_min:.0f})")

print(f"\n" + "="*70)
print("📋 GYROSCOPE SCALE REFERENCE (ICM-20948):")
print("="*70)
print("Range        | LSB per °/s | Full Scale (±°/s)")
print("-" * 70)
print("±250°/s      | 131.072     | 250")
print("±500°/s      | 65.536      | 500")
print("±1000°/s     | 32.768      | 1000")
print("±2000°/s     | 16.384      | 2000")
print("="*70)

print(f"\n🔍 Based on raw values:")
max_val = max(abs(gx_min), abs(gx_max), abs(gy_min), abs(gy_max), abs(gz_min), abs(gz_max))
print(f"  Max absolute value: {max_val:.0f} LSB")

if max_val > 16384:
    print(f"  → Likely using ±2000°/s range (LSB = 16.384)")
    GYRO_SENSITIVITY = 16.384
elif max_val > 8192:
    print(f"  → Likely using ±1000°/s range (LSB = 32.768)")
    GYRO_SENSITIVITY = 32.768
else:
    print(f"  → Likely using ±500°/s range (LSB = 65.536)")
    GYRO_SENSITIVITY = 65.536

print(f"\n✅ Using GYRO_SENSITIVITY = {GYRO_SENSITIVITY}")

# ✅ THÊM: Noise analysis
print(f"\n📈 NOISE ANALYSIS:")
print(f"  gx range: {gx_max - gx_min:.0f} LSB → ~{(gx_max - gx_min) / GYRO_SENSITIVITY:.2f}°/s")
print(f"  gy range: {gy_max - gy_min:.0f} LSB → ~{(gy_max - gy_min) / GYRO_SENSITIVITY:.2f}°/s")
print(f"  gz range: {gz_max - gz_min:.0f} LSB → ~{(gz_max - gz_min) / GYRO_SENSITIVITY:.2f}°/s")
print(f"  gx std: {gx_std:.2f} LSB → ~{gx_std / GYRO_SENSITIVITY:.4f}°/s")
print(f"  gy std: {gy_std:.2f} LSB → ~{gy_std / GYRO_SENSITIVITY:.4f}°/s")
print(f"  gz std: {gz_std:.2f} LSB → ~{gz_std / GYRO_SENSITIVITY:.4f}°/s")

# Update calibration file
calib["gx_bias"] = gx_bias
calib["gy_bias"] = gy_bias
calib["gz_bias"] = gz_bias
calib["gyro_sensitivity"] = GYRO_SENSITIVITY

with open("/home/mobile2/leg2_bipedal/imu/imu_calib.json", "w") as f:
    json.dump(calib, f, indent=2)

print(f"\n✓ Calibration saved to imu_calib.json")
print(f"\n📝 Summary:")
print(f"  • Samples: {SAMPLES}")
print(f"  • Duration: {SAMPLES * SAMPLE_RATE:.1f}s")
print(f"  • gz_bias: {gz_bias:+.2f} LSB")