import zmq
import json
import time

def test_home():
    """Simple test: Move chân phải về home position"""
    
    # Connect tới server
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")  # ← Connect port 5555
    socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
    
    print("=" * 50)
    print("Test: Move Right Leg Home")
    print("=" * 50 + "\n")
    
    try:
        # 1. Get initial feedback
        print("[1] Reading initial state...")
        socket.send_json({"type": "feedback"})
        response = socket.recv_json()
        
        if response["status"] == "success":
            print("✓ Initial positions:")
            pos = response["servo_pos"]
            print(f"  leg_bub:      {pos[0]}")
            print(f"  leg_hip:      {pos[1]}")
            print(f"  leg_twist:    {pos[2]}")
            print(f"  leg_knee:     {pos[3]}")
            print(f"  leg_foot:     {pos[4]}")
            print(f"  leg_gripper:  {pos[5]}")
        else:
            print(f"✗ Failed to read feedback: {response}")
            return
        
        print()
        
        # 2. Move home
        print("[2] Moving to home position...")
        home_positions = [370, 2612, 2702, 2287, 2155, 2048]  # 6 servos
        
        socket.send_json({
            "type": "move",
            "positions": home_positions
        })
        response = socket.recv_json()
        
        if response["status"] == "success":
            print("✓ Home command sent!")
            print(f"  Target positions: {home_positions}")
        else:
            print(f"✗ Move home failed: {response}")
            return
        
        print()
        
        # 3. Wait a bit for servo to move
        print("[3] Waiting 2 seconds for servo to move...")
        time.sleep(2)
        
        print()
        
        # 4. Read final feedback
        print("[4] Reading final state...")
        socket.send_json({"type": "feedback"})
        response = socket.recv_json()
        
        if response["status"] == "success":
            print("✓ Final positions:")
            pos = response["servo_pos"]
            print(f"  leg_bub:      {pos[0]} (target: {home_positions[0]})")
            print(f"  leg_hip:      {pos[1]} (target: {home_positions[1]})")
            print(f"  leg_twist:    {pos[2]} (target: {home_positions[2]})")
            print(f"  leg_knee:     {pos[3]} (target: {home_positions[3]})")
            print(f"  leg_foot:     {pos[4]} (target: {home_positions[4]})")
            print(f"  leg_gripper:  {pos[5]} (target: {home_positions[5]})")
        else:
            print(f"✗ Failed to read final state: {response}")
        
        print()
        print("=" * 50)
        print("✅ Test complete!")
        print("=" * 50)
        
    except zmq.error.Again:
        print("✗ Timeout: Server không response")
    except json.JSONDecodeError as e:
        print(f"✗ Invalid JSON response: {e}")
    except Exception as e:
        print(f"✗ Error: {e}")
    
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    test_home()