#FILE NÀY ĐƯỢC DÙNG ĐỂ  NHẬN REQUEST DATA TỪ CLIENT LẤY DATA TỪ PI 
import zmq
import time 
import os 
    
    # 1. Cấu hình địa chỉ IP của Pi (thay IP cho đúngvới máy mobile2 của bạn)
server_ip = "mobile2.local" # VD: "localhost" nếu chạycùng máy, hoặc IP của Pi nếu chạy từ laptop
server_port = 5556        # Port 5556 là chân trái,5555 là chân phải
    
    # Thiết lập kết nối
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect(f"tcp://{server_ip}:{server_port}")
socket.setsockopt(zmq.RCVTIMEO, 2000)

print("Bat dau lay data lien tuc (Bam Ctrl+C dedung)...")

while True:
    try:
        socket.send_json({"type": "feedback"})
        response = socket.recv_json()
            
        if response.get("status") == "success":
            servo_pos = response.get('servo_pos')
            gyro = response.get('gyro')
                
            print(f"[{time.strftime('%H:%M:%S')}] Servo: {servo_pos} | Gyro: {[round(g, 2) for g in gyro]}")

        else:
                print("Loi tu server:", response)
                
        time.sleep(0.1) 
            
    except zmq.error.Again:
            print("Timeout! Khong nhan duoc phan hoi.")
    except KeyboardInterrupt:
            print("\nDa dung chuong trinh.")
            break
