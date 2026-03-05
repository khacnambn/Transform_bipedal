"""
Client controller for MCU server running on Raspberry Pi 5.
Connects via ZeroMQ and sends control commands.
"""

import zmq
import json
import time
from typing import Dict, List, Optional


class MCUClient:
    """Client for controlling MCU server on Raspberry Pi 5."""

    def __init__(self, server_address: str = "localhost", port: int = 5555):
        """
        Initialize MCU client.

        Args:
            server_address: IP address or hostname of Raspberry Pi
            port: ZeroMQ port of MCU server
        """
        self.server_address = server_address
        self.port = port
        self.context = zmq.Context()
        self.socket = None

    def connect(self) -> bool:
        """Connect to MCU server."""
        try:
            self.socket = self.context.socket(zmq.REQ)
            self.socket.connect(f"tcp://{self.server_address}:{self.port}")
            self.socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout
            print(f"✓ Connected to MCU server at {self.server_address}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def send_command(self, command: Dict) -> Optional[Dict]:
        """Send command and receive response."""
        try:
            self.socket.send_json(command)
            response = self.socket.recv_json()
            return response
        except zmq.Again:
            print("✗ Server timeout")
            return None
        except Exception as e:
            print(f"✗ Communication error: {e}")
            return None

    def move(self, positions: List[int]) -> bool:
        """
        Move all servos to specified positions.

        Args:
            positions: List of 8 raw tick values [servo0, servo1, ..., servo7]

        Returns:
            True if successful
        """
        if len(positions) != 8:
            print(f"✗ Expected 8 positions, got {len(positions)}")
            return False

        command = {
            "type": "move",
            "positions": positions,
        }

        response = self.send_command(command)
        if response and response.get("status") == "success":
            print(f"✓ Move command successful")
            return True
        return False

    def get_feedback(self) -> Optional[Dict]:
        """Get current robot state."""
        command = {"type": "feedback"}
        response = self.send_command(command)

        if response and response.get("status") == "success":
            return response.get("state")
        return None

    def home(self) -> bool:
        """Move all servos to home position."""
        command = {"type": "home"}
        response = self.send_command(command)
        return response and response.get("status") == "success"

    def calibrate(self) -> bool:
        """Calibrate all servos."""
        command = {"type": "calibrate"}
        response = self.send_command(command)
        return response and response.get("status") == "success"

    def stop(self) -> bool:
        """Stop all servos."""
        command = {"type": "stop"}
        response = self.send_command(command)
        return response and response.get("status") == "success"

    def disconnect(self):
        """Disconnect from server."""
        if self.socket:
            self.socket.close()
        self.context.term()
        print("✓ Disconnected")


def main():
    """Example usage."""
    # Connect to MCU server on Pi 5
    client = MCUClient(
        server_address="192.168.1.100",  # Change to actual Pi IP
        port=5555
    )

    if not client.connect():
        return

    try:
        # Test home
        print("\n[1] Moving to home position...")
        client.home()
        time.sleep(1)

        # Test feedback
        print("\n[2] Getting feedback...")
        feedback = client.get_feedback()
        if feedback:
            print(f"✓ Servo positions: {feedback['servo_pos']}")
            print(f"✓ Servo temps: {feedback['servo_temp']}")

        # Test move
        print("\n[3] Moving servos...")
        test_positions = [2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048]
        client.move(test_positions)
        time.sleep(1)

        # Test stop
        print("\n[4] Stopping servos...")
        client.stop()

    finally:
        client.disconnect()


if __name__ == "__main__":
    main()