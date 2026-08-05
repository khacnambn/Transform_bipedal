import sys
sys.path.insert(0, '/home/nam/Transform_Bipedal/bipedal_nam/src')

from bipedal_robot.sensors.imu import IMUFusion

imu = IMUFusion(
    left_host='mobile2.local', left_port=5556,
    right_host='mobile1.local', right_port=5555
)

# Read 5 times
for i in range(5):
    print(f"\n[Sample {i+1}]")
    left_raw = imu.left.read_imu()
    right_raw = imu.right.read_imu()
    
    print(f"  LEFT:  quat={left_raw.get('quat')}, gyro={left_raw.get('gyro')}")
    print(f"  RIGHT: quat={right_raw.get('quat')}, gyro={right_raw.get('gyro')}")
    
    import time
    time.sleep(0.1)