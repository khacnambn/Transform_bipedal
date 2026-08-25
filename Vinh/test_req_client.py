#!/usr/bin/env python3
"""
Client REQ kiểm tra kết nối tới 2 leg server.

CHỈ gửi lệnh "feedback" - đọc thuần, KHÔNG làm động cơ chạy.
Dùng để xác minh mạng/ZMQ/servo/IMU trước khi mở giao diện thanh trượt.

Chạy:  python Vinh/test_req_client.py
       python Vinh/test_req_client.py --loop 10     # đọc 10 vòng
"""

import argparse
import math
import time

import zmq

LEGS = [
    {"side": "LEFT", "host": "mobile2.local", "port": 5556},
    {"side": "RIGHT", "host": "mobile1.local", "port": 5555},
]

JOINTS = ["4 bub", "5 hip", "6 twist", "7 knee", "8 foot", "9 gripper"]


def quat_to_euler_deg(q):
    """Quaternion [w,x,y,z] -> roll/pitch/yaw (độ)."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-10:
        return 0.0, 0.0, 0.0
    w, x, y, z = w / n, x / n, y / n, z / n

    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def probe(ctx, leg, timeout_ms=3000):
    """Gửi 1 request feedback, trả về (thời gian ms, response) hoặc (None, lỗi)."""
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{leg['host']}:{leg['port']}")
    try:
        t0 = time.time()
        sock.send_json({"type": "feedback"})
        resp = sock.recv_json()
        return (time.time() - t0) * 1000, resp
    except zmq.Again:
        return None, f"TIMEOUT sau {timeout_ms}ms - server không trả lời"
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"
    finally:
        sock.close()


def show(leg, ms, resp):
    print(f"\n{'='*72}")
    print(f"  {leg['side']}   tcp://{leg['host']}:{leg['port']}")
    print(f"{'='*72}")

    if ms is None:
        print(f"  ❌ {resp}")
        return False

    print(f"  ✅ Trả lời sau {ms:.1f} ms   |   status = {resp.get('status')}")

    pos = resp.get("servo_pos", [])
    print(f"\n  [SERVO POSITION]  ({len(pos)} giá trị)")
    if not pos:
        print("     (trống)")
    for name, tick in zip(JOINTS, pos):
        deg = tick / 4096 * 360
        warn = "  ⚠️ = 0, đọc lỗi" if tick == 0 else ""
        print(f"     {name:10s} {tick:5d} tick  ({deg:6.1f}°){warn}")

    quat = resp.get("quat", [])
    if len(quat) == 4:
        r, p, y = quat_to_euler_deg(quat)
        print(f"\n  [IMU]")
        print(f"     quat  w={quat[0]:+.4f} x={quat[1]:+.4f} y={quat[2]:+.4f} z={quat[3]:+.4f}")
        print(f"     euler roll={r:+7.2f}°  pitch={p:+7.2f}°  yaw={y:+7.2f}°")
        if quat == [0.0, 0.0, 0.0, 0.0]:
            print("     ⚠️ quat toàn 0 - IMU chưa warm-up hoặc lỗi")

    gyro = resp.get("gyro", [])
    if len(gyro) == 3:
        print(f"     gyro  gx={gyro[0]:+.4f} gy={gyro[1]:+.4f} gz={gyro[2]:+.4f}  rad/s")

    return True


def main():
    ap = argparse.ArgumentParser(description="Test REQ client tới 2 leg server")
    ap.add_argument("--loop", type=int, default=1, help="Số vòng đọc (mặc định 1)")
    ap.add_argument("--delay", type=float, default=1.0, help="Giây giữa 2 vòng")
    args = ap.parse_args()

    ctx = zmq.Context()
    ok_count = {leg["side"]: 0 for leg in LEGS}

    try:
        for i in range(args.loop):
            if args.loop > 1:
                print(f"\n\n########## VÒNG {i+1}/{args.loop} ##########")
            for leg in LEGS:
                ms, resp = probe(ctx, leg)
                if show(leg, ms, resp):
                    ok_count[leg["side"]] += 1
            if i < args.loop - 1:
                time.sleep(args.delay)
    except KeyboardInterrupt:
        print("\n\nDừng bởi người dùng")
    finally:
        print(f"\n{'='*72}")
        for side, n in ok_count.items():
            mark = "✅" if n == args.loop else "❌"
            print(f"  {mark} {side}: {n}/{args.loop} lần thành công")
        print(f"{'='*72}")
        ctx.term()


if __name__ == "__main__":
    main()