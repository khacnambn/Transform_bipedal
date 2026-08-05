import zmq
import json
import time

# Test LEFT server only
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.setsockopt(zmq.RCVTIMEO, 2000)
socket.setsockopt(zmq.LINGER, 0)

print("🔌 Connecting to LEFT server (port 5556)...")
try:
    socket.connect("tcp://mobile2.local:5556")
    print("✓ Connected!")
except Exception as e:
    print(f"✗ Connection failed: {e}")
    exit(1)

print("\n📤 Sending feedback request...")
try:
    socket.send_json({"type": "feedback"})
    print("✓ Request sent!")
    
    print("⏳ Waiting for response (2s timeout)...")
    response = socket.recv_json()
    
    print("\n✅ Response received!")
    print(f"Status: {response.get('status')}")
    print(f"IMU: {response.get('imu')}")
    print(f"Servo positions: {response.get('servo_pos')}")
    
except zmq.Again:
    print("❌ Timeout - no response from server!")
except Exception as e:
    print(f"❌ Error: {e}")

socket.close()
context.term()