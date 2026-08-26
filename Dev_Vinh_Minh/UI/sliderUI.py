#!/usr/bin/env python3
"""
Giao diện thanh trượt điều khiển 12 động cơ (2 chân) qua ZeroMQ.

- Chạy trên LAPTOP, KHÔNG cần tắt leg server -> IMU vẫn chạy bình thường.
- Lệnh move gửi qua cổng PUSH/PULL (port + 100) nên không chặn giao diện.
- Vị trí thật đọc định kỳ qua REQ/REP (port) để đối chiếu.
- servo_limits đọc thẳng từ leg_server_debug/{left,right}.py bằng AST,
  nên sửa limit trong file server rồi lưu là giao diện tự cập nhật.
"""

import ast
import math
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import zmq


# ==============================
# Cấu hình
# ==============================
def _find_src_dir(start: Path) -> Path:
    """
    Đi ngược lên các thư mục cha để tìm bipedal_nam/src/leg_server_debug.
    Nhờ vậy file này đặt ở đâu trong repo cũng chạy được.
    """
    rel = Path("bipedal_nam") / "src" / "leg_server_debug"
    for base in [start, *start.parents]:
        if (base / rel).is_dir():
            return base / rel
    raise FileNotFoundError(f"Không tìm thấy {rel} từ {start}")


SRC_DIR = _find_src_dir(Path(__file__).resolve().parent)

LEGS = [
    {"side": "LEFT", "host": "mobile2.local", "port": 5556, "src": "left.py"},
    {"side": "RIGHT", "host": "mobile1.local", "port": 5555, "src": "right.py"},
]

JOINTS = {4: "bub", 5: "hip", 6: "twist", 7: "knee", 8: "foot", 9: "gripper"}

SEND_HZ = 20  # tần suất tối đa gửi lệnh move khi đang kéo
# Đây là thứ quyết định panel 3D mượt hay giật.
# Đo thực tế: sleep 10ms cho ~67Hz với độ trễ tệ nhất chỉ 20ms - tốt nhất.
# Nghỉ >=20ms bị trùng pha với servo_loop 25Hz của server -> kẹt 280ms, tụt còn 11Hz.
# Nghỉ 0ms đạt 395Hz nhưng độ trễ tệ nhất vọt lên 200ms và tốn CPU vô ích.
POLL_S = 0.01  # chu kỳ đọc feedback -> ~67Hz thực tế
VIEW_HZ = 30  # tần suất vẽ lại panel 3D
TICKS_PER_REV = 4096

# Tốc độ dùng riêng cho nút Home - luôn chậm, không phụ thuộc ô nhập
HOME_SPEED = 700
HOME_ACCEL = 70

ctx = zmq.Context.instance()


def ticks_to_deg(ticks):
    """Quy đổi thô tick -> độ (không dùng calib, chỉ để tham khảo)."""
    return ticks / TICKS_PER_REV * 360.0


def parse_servo_limits(py_file: Path):
    """
    Đọc servo_limits trực tiếp từ file server bằng AST.
    Tránh phải chép tay -> không bao giờ lệch với bản đang chạy.
    """
    tree = ast.parse(py_file.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Attribute) and target.attr == "servo_limits":
                raw = ast.literal_eval(node.value)
                return {int(k): (int(v["min"]), int(v["max"])) for k, v in raw.items()}
    raise ValueError(f"Không tìm thấy servo_limits trong {py_file}")


def _const_int_list(node):
    """literal_eval nhưng chấp nhận cả dạng viết tắt [2048] * 6."""
    try:
        return [int(v) for v in ast.literal_eval(node)]
    except (ValueError, TypeError, SyntaxError):
        pass
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mult):
        for seq_node, n_node in ((node.left, node.right), (node.right, node.left)):
            try:
                seq, count = ast.literal_eval(seq_node), ast.literal_eval(n_node)
            except (ValueError, TypeError, SyntaxError):
                continue
            if isinstance(seq, list) and isinstance(count, int):
                return [int(v) for v in seq * count]
    return None


def parse_home_pos(py_file: Path):
    """Đọc home_pos (biến cục bộ trong process_command) từ file server."""
    tree = ast.parse(py_file.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == "home_pos":
                return _const_int_list(node.value)
    return None


HOME_POS = {}
for _cfg in LEGS:
    try:
        HOME_POS[_cfg["side"]] = parse_home_pos(SRC_DIR / _cfg["src"])
    except Exception:
        HOME_POS[_cfg["side"]] = None


# ==============================
# Toán quaternion (giống test_imu_fusion_simple.py để số liệu khớp nhau)
# ==============================
def q_norm(q):
    n = math.sqrt(sum(v * v for v in q))
    return [v / n for v in q] if n > 1e-10 else [1.0, 0.0, 0.0, 0.0]


def q_mult(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return q_norm(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def q_conj(q):
    return [q[0], -q[1], -q[2], -q[3]]


# Xoay +90° quanh Z để đưa khung IMU về baselink (khớp với sim)
_C = math.cos(math.pi / 4)
_S = math.sin(math.pi / 4)
Q_ROT = [_C, 0.0, 0.0, _S]


def to_baselink(q):
    return q_mult(q_mult(Q_ROT, q_norm(q)), q_conj(Q_ROT))


def strip_yaw(q):
    """Bỏ yaw, giữ nguyên roll & pitch. Yaw của Madgwick không có mag là rác."""
    w, x, y, z = q_norm(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return q_norm([cr * cp, sr * cp, cr * sp, -sr * sp])


def q_fuse(a, b):
    a, b = q_norm(a), q_norm(b)
    if sum(p * q for p, q in zip(a, b)) < 0:
        b = [-v for v in b]
    return q_norm([(p + q) / 2 for p, q in zip(a, b)])


def q_to_euler_deg(q):
    w, x, y, z = q_norm(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


def q_to_matrix(q):
    """Cột thứ i của ma trận = trục i của vật thể, biểu diễn trong khung thế giới."""
    w, x, y, z = q_norm(q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


class OrientationView(tk.Canvas):
    """
    Vẽ một CỤC PHẲNG 3D nghiêng theo orientation của IMU.

    Không dùng thư viện đồ hoạ nào - tự chiếu 3D->2D rồi vẽ đa giác lên Canvas,
    sắp xếp mặt theo độ sâu (painter's algorithm) và tô bóng theo hướng mặt.
    """

    W, H = 210, 190
    AZ, EL = math.radians(38), math.radians(20)  # góc nhìn camera
    # Nửa kích thước cục phẳng. Cạnh DÀI nằm dọc trục Y, cạnh NGẮN dọc trục X.
    HX, HY, HZ = 0.62, 1.00, 0.11
    LIGHT = (0.35, 0.30, 0.89)  # hướng nguồn sáng

    # 8 đỉnh hộp: bit 0=X, 1=Y, 2=Z
    CORNERS = [
        (sx * 1.0, sy * 1.0, sz * 1.0)
        for sz in (-1, 1)
        for sy in (-1, 1)
        for sx in (-1, 1)
    ]
    # mỗi mặt: (4 chỉ số đỉnh, pháp tuyến, màu nền)
    FACES = [
        ((4, 5, 7, 6), (0, 0, 1), (90, 150, 220)),  # trên
        ((0, 2, 3, 1), (0, 0, -1), (55, 80, 120)),  # dưới
        ((1, 3, 7, 5), (1, 0, 0), (200, 90, 100)),  # +X (mũi)
        ((0, 4, 6, 2), (-1, 0, 0), (120, 60, 70)),  # -X
        ((2, 6, 7, 3), (0, 1, 0), (90, 180, 130)),  # +Y
        ((0, 1, 5, 4), (0, -1, 0), (60, 110, 85)),  # -Y
    ]

    def __init__(self, master, title):
        super().__init__(
            master, width=self.W, height=self.H, bg="#15171c", highlightthickness=0
        )
        self.title = title
        self.scale = min(self.W, self.H) * 0.30
        self._last = None
        self.draw(None)

    # ---------- chiếu 3D -> 2D ----------

    def _proj(self, v):
        x, y, z = v
        sx = -x * math.sin(self.AZ) + y * math.cos(self.AZ)
        sy = -(x * math.cos(self.AZ) + y * math.sin(self.AZ)) * math.sin(
            self.EL
        ) + z * math.cos(self.EL)
        return self.W / 2 + sx * self.scale, self.H / 2 + 8 - sy * self.scale

    def _depth(self, v):
        """Càng lớn càng gần camera."""
        x, y, z = v
        return (
            x * math.cos(self.EL) * math.cos(self.AZ)
            + y * math.cos(self.EL) * math.sin(self.AZ)
            + z * math.sin(self.EL)
        )

    @staticmethod
    def _shade(rgb, k):
        return "#%02x%02x%02x" % tuple(max(0, min(255, int(c * k))) for c in rgb)

    # ---------- vẽ ----------

    def draw(self, quat, force=False):
        # Bỏ qua nếu góc gần như không đổi -> tiết kiệm cho Canvas, hết giật
        if not force and quat is not None and self._last is not None:
            if max(abs(a - b) for a, b in zip(quat, self._last)) < 2e-4:
                return
        self._last = list(quat) if quat else None

        self.delete("all")
        self.create_text(
            self.W / 2,
            11,
            text=self.title,
            fill="#c8c8c8",
            font=("TkDefaultFont", 9, "bold"),
        )

        # lưới nền nằm ngang làm mốc "mặt đất"
        for t in (-1.3, 0.0, 1.3):
            self.create_line(
                *self._proj((-1.3, t, -0.55)),
                *self._proj((1.3, t, -0.55)),
                fill="#2a2d34",
            )
            self.create_line(
                *self._proj((t, -1.3, -0.55)),
                *self._proj((t, 1.3, -0.55)),
                fill="#2a2d34",
            )

        if quat is None:
            self.create_text(
                self.W / 2, self.H / 2, text="chưa có dữ liệu", fill="#666"
            )
            return

        R = q_to_matrix(quat)

        def rot(v):
            return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]

        verts = [
            rot((x * self.HX, y * self.HY, z * self.HZ)) for x, y, z in self.CORNERS
        ]

        # sắp mặt từ xa tới gần rồi tô đè lên nhau
        faces = []
        for idx, normal, rgb in self.FACES:
            pts = [verts[i] for i in idx]
            centroid = [sum(p[i] for p in pts) / 4 for i in range(3)]
            n = rot(normal)
            lit = max(0.0, sum(n[i] * self.LIGHT[i] for i in range(3)))
            faces.append(
                (self._depth(centroid), pts, self._shade(rgb, 0.40 + 0.75 * lit))
            )

        for _, pts, color in sorted(faces, key=lambda f: f[0]):
            flat = [c for p in pts for c in self._proj(p)]
            self.create_polygon(flat, fill=color, outline="#0d0f12", width=1)

        # 3 trục cắm ra từ tâm cục phẳng
        origin = self._proj((0, 0, 0))
        axes = [(0, "#ff5566", "X"), (1, "#3ddc84", "Y"), (2, "#5aa9ff", "Z")]
        axes.sort(key=lambda a: self._depth([R[i][a[0]] for i in range(3)]))
        for col, color, label in axes:
            v = [R[i][col] * 1.45 for i in range(3)]
            tip = self._proj(v)
            self.create_line(
                *origin, *tip, fill=color, width=3, arrow="last", arrowshape=(9, 11, 4)
            )
            self.create_text(
                *self._proj([c * 1.72 for c in v]),
                text=label,
                fill=color,
                font=("TkDefaultFont", 8, "bold"),
            )


class FeedbackPoller(threading.Thread):
    """Thread riêng đọc feedback qua REQ/REP. Socket ZMQ không thread-safe nên phải tách."""

    def __init__(self, host, port, out_q, side):
        super().__init__(daemon=True)
        self.host, self.port, self.out_q, self.side = host, port, out_q, side
        self.running = True

    def _new_socket(self):
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.RCVTIMEO, 2000)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://{self.host}:{self.port}")
        return sock

    def run(self):
        sock = self._new_socket()
        while self.running:
            try:
                sock.send_json({"type": "feedback"})
                resp = sock.recv_json()
                self.out_q.put((self.side, "ok", resp))  # cả dict: servo_pos + quat
            except Exception as exc:
                self.out_q.put((self.side, "err", str(exc)))
                sock.close()  # REQ hỏng lockstep sau timeout -> phải tạo lại
                sock = self._new_socket()
            time.sleep(POLL_S)
        sock.close()


class LegPanel(ttk.LabelFrame):
    """Một cột: 6 thanh trượt + nút điều khiển cho một chân."""

    def __init__(self, master, cfg, status_q):
        self.side = cfg["side"]
        self.host, self.port = cfg["host"], cfg["port"]
        super().__init__(
            master, text=f"{self.side}  —  {self.host}:{self.port}", padding=8
        )

        self.limits = parse_servo_limits(SRC_DIR / cfg["src"])
        self.vars = {}
        self.actual_lbl = {}
        self.enabled = tk.BooleanVar(value=False)
        self.blocked = False  # True khi limit trong file lệch với limit trên Pi
        self.last_quat = None  # quaternion thô mới nhất từ IMU chân này
        # Vị trí ĐÃ RA LỆNH lần cuối (khác với vị trí đo được - servo có vùng chết).
        # Gửi lại số ĐO cho khớp không đụng tới sẽ đổi đích của nó -> khớp tự dịch.
        self.goal = None
        self.recv_count = 0  # đếm gói feedback nhận được, dùng tính Hz thật
        self.last_sent = None
        self.last_send_t = 0.0

        # PUSH socket: gửi move, không chờ trả lời -> giao diện không bao giờ treo
        self.push = ctx.socket(zmq.PUSH)
        self.push.setsockopt(zmq.SNDHWM, 5)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(f"tcp://{self.host}:{self.port + 100}")

        # REQ socket riêng cho lệnh một lần (home / config) - dùng từ main thread
        self.req = ctx.socket(zmq.REQ)
        self.req.setsockopt(zmq.RCVTIMEO, 3000)
        self.req.setsockopt(zmq.LINGER, 0)
        self.req.connect(f"tcp://{self.host}:{self.port}")

        self._build_rows()
        self._build_controls()

        self.poller = FeedbackPoller(self.host, self.port, status_q, self.side)
        self.poller.start()

    def _build_rows(self):
        for row, mid in enumerate(sorted(JOINTS)):
            lo, hi = self.limits[mid]
            span = hi - lo

            name = f"{mid} {JOINTS[mid]}"
            # Cảnh báo khớp bị khoá gần cứng - kéo cũng không nhúc nhích
            if span < 50:
                name += " 🔒"

            ttk.Label(self, text=name, width=11).grid(
                row=row, column=0, sticky="w", pady=2
            )

            var = tk.IntVar(value=lo)
            self.vars[mid] = var

            scale = tk.Scale(
                self,
                from_=lo,
                to=hi,
                variable=var,
                orient="horizontal",
                length=250,
                showvalue=0,
                resolution=1,
                state="normal" if span >= 50 else "disabled",
            )
            scale.grid(row=row, column=1, padx=4)

            ttk.Label(self, textvariable=var, width=6, anchor="e").grid(
                row=row, column=2
            )
            ttk.Label(self, text=f"[{lo}–{hi}]", width=12, foreground="#888").grid(
                row=row, column=3
            )

            lbl = ttk.Label(self, text="thật: —", width=26, foreground="#0a6")
            lbl.grid(row=row, column=4, sticky="w")
            self.actual_lbl[mid] = lbl

    def _build_controls(self):
        bar = ttk.Frame(self)
        bar.grid(row=99, column=0, columnspan=5, sticky="we", pady=(10, 0))

        ttk.Checkbutton(
            bar, text="BẬT gửi lệnh", variable=self.enabled, command=self._on_enable
        ).pack(side="left", padx=(0, 10))

        ttk.Button(bar, text="Đồng bộ", command=self.sync_from_robot).pack(
            side="left", padx=2
        )
        ttk.Button(bar, text="Home", command=self.go_home).pack(side="left", padx=2)
        ttk.Button(bar, text="DỪNG", command=self.stop).pack(side="left", padx=2)

        ttk.Label(bar, text="  speed").pack(side="left")
        self.speed_var = tk.StringVar(value="600")
        ttk.Entry(bar, textvariable=self.speed_var, width=6).pack(side="left")

        ttk.Label(bar, text=" accel").pack(side="left")
        self.accel_var = tk.StringVar(value="50")
        ttk.Entry(bar, textvariable=self.accel_var, width=5).pack(side="left")

        ttk.Button(bar, text="Áp dụng", command=self.apply_config).pack(
            side="left", padx=4
        )

        self.status = ttk.Label(self, text="● chưa kết nối", foreground="#c00")
        self.status.grid(row=100, column=0, columnspan=5, sticky="w", pady=(6, 0))

    # ---------- hành động ----------

    def _on_enable(self):
        # Bật là đồng bộ ngay: tránh cú nhảy khi thanh trượt đang lệch vị trí thật.
        # sync_from_robot() tự tắt lại nếu phát hiện limit lệch với Pi.
        if not self.enabled.get():
            return
        if not self.sync_from_robot():
            return

        if self.goal is not None:
            # Đã biết đích -> quay lại ĐÚNG lệnh cũ thay vì lấy số đo. Lấy số đo sẽ
            # nhồi sai số vùng chết vào đích -> kéo 1 khớp là 5 khớp kia cùng dịch.
            self._apply_goal()
            return

        # Chưa từng ra lệnh trong phiên này -> không biết servo đang giữ đích nào.
        # Đành lấy số đo, và lần kéo đầu tiên sẽ làm các khớp dịch đúng bằng sai
        # số vùng chết của chúng. Bấm Home trước là hết, vì Home đặt đích đã biết.
        self.goal = self._current()
        self.status.config(
            text="⚠ Chưa biết đích servo (chưa Home lần nào). Lần kéo đầu có thể "
            "làm vài khớp dịch nhẹ — bấm Home trước để tránh.",
            foreground="#a60",
        )

    def _apply_goal(self):
        if self.goal is None:
            return
        for mid, val in zip(sorted(JOINTS), self.goal):
            lo, hi = self.limits[mid]
            self.vars[mid].set(max(lo, min(hi, val)))
        self.last_sent = self._current()

    def _req(self, msg):
        try:
            self.req.send_json(msg)
            return self.req.recv_json()
        except Exception as exc:
            self.status.config(text=f"● lỗi: {exc}", foreground="#c00")
            self.req.close()
            self.req = ctx.socket(zmq.REQ)
            self.req.setsockopt(zmq.RCVTIMEO, 3000)
            self.req.setsockopt(zmq.LINGER, 0)
            self.req.connect(f"tcp://{self.host}:{self.port}")
            return None

    def sync_from_robot(self):
        """
        Kéo vị trí THẬT của servo về thanh trượt.

        AN TOÀN: nếu vị trí thật nằm NGOÀI limit đọc từ file, nghĩa là limit
        trên Pi khác limit trong file này. Lúc đó lệnh move đầu tiên sẽ kéo
        servo về biên gần nhất -> có thể giật hàng trăm tick. Chặn luôn.
        """
        resp = self._req({"type": "feedback"})
        if not resp:
            return False

        pos = resp.get("servo_pos", [])

        # Nếu server có trả về "target" (đích nó đang giữ) thì dùng luôn làm goal.
        # Server hiện chưa trả; để sẵn đây, thêm 1 dòng bên Pi là tự hoạt động:
        #     "target": self.target_positions,
        tgt = resp.get("target", [])
        if self.goal is None and len(tgt) == 6 and all(t > 0 for t in tgt):
            self.goal = list(tgt)

        out_of_range = []
        no_reading = []

        for i, mid in enumerate(sorted(JOINTS)):
            if i >= len(pos) or pos[i] <= 0:
                # Không có số đọc hợp lệ -> slider vẫn nằm ở giá trị khởi tạo
                # (limit MIN). Gửi đi là khớp đó lao thẳng xuống biên dưới.
                no_reading.append(mid)
                continue
            lo, hi = self.limits[mid]
            if not lo <= pos[i] <= hi:
                out_of_range.append((mid, pos[i], lo, hi))
            self.vars[mid].set(max(lo, min(hi, pos[i])))

        self.last_sent = self._current()

        if no_reading:
            self.blocked = True
            self.enabled.set(False)
            names = ", ".join(f"{m} {JOINTS[m]}" for m in no_reading)
            self.status.config(
                text=f"⛔ KHÔNG ĐỌC ĐƯỢC VỊ TRÍ — đã khoá: {names}. "
                f"Gửi lệnh sẽ kéo khớp này về biên dưới.",
                foreground="#c00",
            )
            return False

        if out_of_range:
            self.blocked = True
            self.enabled.set(False)
            worst = max(
                out_of_range, key=lambda t: min(abs(t[1] - t[2]), abs(t[1] - t[3]))
            )
            mid, p, lo, hi = worst
            jump = p - (lo if abs(p - lo) < abs(p - hi) else hi)
            detail = ", ".join(
                f"{m} {JOINTS[m]}={p}∉[{lo},{hi}]" for m, p, lo, hi in out_of_range
            )
            self.status.config(
                text=f"⛔ LIMIT LỆCH VỚI PI — đã khoá. {detail}. "
                f"Nếu gửi, {JOINTS[mid]} sẽ giật {abs(jump)} tick ({abs(jump)/4096*360:.0f}°)",
                foreground="#c00",
            )
            return False

        self.blocked = False
        return True

    def go_home(self):
        """
        Gửi lệnh home của server, NHƯNG cảnh báo trước nếu home nằm ngoài
        limit hiện tại - server sẽ clamp và ra tư thế không như mong đợi.
        """
        home = HOME_POS.get(self.side)
        if home:
            clamped = []
            for i, mid in enumerate(sorted(JOINTS)):
                lo, hi = self.limits[mid]
                if not lo <= home[i] <= hi:
                    clamped.append(
                        f"{mid} {JOINTS[mid]}: {home[i]} → {max(lo, min(hi, home[i]))}"
                    )
            if clamped:
                ok = messagebox.askokcancel(
                    "Home nằm ngoài limit",
                    f"{self.side}: home trong file server đã cũ so với limit hiện tại.\n\n"
                    + "\n".join(clamped)
                    + "\n\nServer sẽ clamp về biên. Vẫn chạy?",
                )
                if not ok:
                    return
        # Về home LUÔN chạy chậm cho an toàn, bất kể ô speed/accel đang là bao nhiêu.
        self._req({"type": "config", "speed": HOME_SPEED, "acceleration": HOME_ACCEL})
        self._req({"type": "home"})
        self.status.config(
            text=f"● đang về home (speed {HOME_SPEED}, accel {HOME_ACCEL})…",
            foreground="#a60",
        )
        # Home vừa đặt đích = home_pos cho cả 6 khớp -> ghi nhớ làm goal.
        # Nhờ vậy lệnh move sau đó gửi lại đúng 2048 cho khớp không đụng tới,
        # trùng với đích servo đang giữ -> chúng đứng im.
        self.goal = list(home) if home else None
        # speed 200 chậm hơn nhiều -> phải chờ lâu hơn (40 x 300ms = 12s)
        self._settle_sync(tries=40, on_done=self._after_home)

    def _after_home(self):
        self._apply_goal()
        self._restore_config()

    def _restore_config(self):
        """Sau khi về home xong, trả tốc độ lại đúng con số trong ô nhập."""
        try:
            speed, accel = int(self.speed_var.get()), int(self.accel_var.get())
        except ValueError:
            return
        self._req({"type": "config", "speed": speed, "acceleration": accel})
        self.status.config(
            text=f"● đã về home — speed trả lại {speed}/{accel}", foreground="#0a6"
        )

    def _settle_sync(self, tries, prev=None, on_done=None, moved=False, waited=0):
        """
        Đọc lại vị trí mỗi 300ms, chốt slider khi servo đã đứng hẳn.

        Phải THẤY servo chuyển động rồi mới chấp nhận "đứng yên". Nếu chỉ so hai
        lần đọc liên tiếp, lệnh home vừa gửi mà servo chưa kịp nhúc nhích sẽ cho
        hai số giống nhau -> tưởng xong ngay, chốt slider vào vị trí CŨ và trả
        speed về sớm. Có 1.5s ân hạn cho trường hợp khớp vốn đã ở đúng home.
        """
        if tries <= 0:
            self.sync_from_robot()
            if on_done:
                on_done()
            return

        resp = self._req({"type": "feedback"})
        pos = resp.get("servo_pos", []) if resp else []

        if pos and prev is not None and pos != prev:
            moved = True

        if pos and pos == prev and (moved or waited >= 5):
            self.sync_from_robot()
            if on_done:
                on_done()
            return

        self.after(
            300, lambda: self._settle_sync(tries - 1, pos, on_done, moved, waited + 1)
        )

    def stop(self):
        """
        DỪNG = giữ nguyên tư thế hiện tại.

        KHÔNG dùng lệnh "stop" của server: nó gửi self.target_positions, mà biến
        này khởi tạo [0]*6 và chỉ được cập nhật sau lệnh move đầu tiên. Nếu chưa
        move lần nào, [0]*6 bị clamp thành limit MIN -> toàn bộ khớp lao về biên
        dưới (có khớp giật hơn 100°). Thay vào đó đọc vị trí THẬT rồi ghim lại.
        """
        self.enabled.set(False)
        resp = self._req({"type": "feedback"})
        if not resp:
            return
        pos = resp.get("servo_pos", [])
        if len(pos) != 6:
            self.status.config(
                text="● DỪNG: không đọc được vị trí, không ghim", foreground="#c00"
            )
            return

        # Khớp nào đọc lỗi thì giữ nguyên giá trị slider đang có, KHÔNG bỏ luôn cả
        # lệnh - nút DỪNG phải làm được việc kể cả khi feedback hỏng một khớp.
        bad = []
        hold = []
        for i, mid in enumerate(sorted(JOINTS)):
            lo, hi = self.limits[mid]
            if pos[i] > 0:
                hold.append(max(lo, min(hi, pos[i])))
            else:
                bad.append(mid)
                hold.append(self.vars[mid].get())
        try:
            self.push.send_json({"type": "move", "positions": hold}, zmq.NOBLOCK)
        except zmq.Again:
            pass
        for mid, val in zip(sorted(JOINTS), hold):
            self.vars[mid].set(val)
        self.last_sent = hold
        self.goal = hold  # DỪNG cũng là một lệnh -> cập nhật đích
        if bad:
            names = ", ".join(f"{m} {JOINTS[m]}" for m in bad)
            self.status.config(
                text=f"● ĐÃ DỪNG — nhưng {names} không có feedback, giữ theo slider",
                foreground="#c00",
            )
        else:
            self.status.config(
                text="● ĐÃ DỪNG — ghim tại vị trí hiện tại", foreground="#a60"
            )

    def apply_config(self):
        try:
            speed, accel = int(self.speed_var.get()), int(self.accel_var.get())
        except ValueError:
            self.status.config(
                text="● speed/accel phải là số nguyên", foreground="#c00"
            )
            return
        self._req({"type": "config", "speed": speed, "acceleration": accel})

    # ---------- vòng gửi ----------

    def _current(self):
        return [self.vars[mid].get() for mid in sorted(JOINTS)]

    def maybe_send(self):
        """Chỉ gửi khi có thay đổi thật, và không quá SEND_HZ lần/giây."""
        if self.blocked or not self.enabled.get():
            return
        now = time.time()
        if now - self.last_send_t < 1.0 / SEND_HZ:
            return
        cur = self._current()
        if cur == self.last_sent:
            return
        try:
            self.push.send_json({"type": "move", "positions": cur}, zmq.NOBLOCK)
            self.last_sent = cur
            self.goal = cur  # ghi nhớ ĐÍCH đã ra lệnh
            self.last_send_t = now
        except zmq.Again:
            pass  # hàng đợi đầy -> bỏ frame này, frame sau sẽ mang giá trị mới nhất

    def on_feedback(self, kind, resp):
        if kind == "err":
            self.status.config(text=f"● mất kết nối: {resp}", foreground="#c00")
            return

        self.recv_count += 1
        payload = resp.get("servo_pos", []) if isinstance(resp, dict) else resp

        # Quaternion thô từ IMU, để panel 3D dùng
        quat = resp.get("quat", []) if isinstance(resp, dict) else []
        if len(quat) == 4 and any(quat):
            self.last_quat = list(quat)

        # Khi CHƯA bật gửi lệnh, thanh trượt tự bám theo vị trí thật của robot.
        # Nhờ vậy sau Home / di chuyển bằng tay, slider luôn khớp - không cần
        # bấm Đồng bộ. Khi đã bật thì KHÔNG đụng vào, để người dùng làm chủ.
        follow = (
            not self.enabled.get()
            and not self.blocked
            and len(payload) == 6
            and all(p > 0 for p in payload)
        )
        if follow:
            for i, mid in enumerate(sorted(JOINTS)):
                lo, hi = self.limits[mid]
                self.vars[mid].set(max(lo, min(hi, payload[i])))
            self.last_sent = self._current()
            self.status.config(
                text="● đã kết nối — slider đang bám robot", foreground="#0a6"
            )
        else:
            self.status.config(text="● đã kết nối", foreground="#0a6")

        for i, mid in enumerate(sorted(JOINTS)):
            if i < len(payload):
                tick = payload[i]
                delta = tick - self.vars[mid].get()
                self.actual_lbl[mid].config(
                    text=f"thật: {tick:5d} ({ticks_to_deg(tick):6.1f}°) Δ{delta:+5d}"
                )

    def shutdown(self):
        self.poller.running = False
        self.push.close()
        self.req.close()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Điều khiển động cơ — 2 chân")
        self.status_q = queue.Queue()

        top = ttk.Frame(self, padding=(8, 6))
        top.grid(row=0, column=0, columnspan=2, sticky="we")

        ttk.Button(top, text="⌂  HOME CẢ 2 CHÂN", command=self.home_all).pack(
            side="left", padx=(0, 12)
        )

        self.all_status = ttk.Label(top, text="", foreground="#a60")
        self.all_status.pack(side="left")

        ttk.Label(
            self,
            text="Thanh trượt tính bằng TICK, giới hạn đọc từ leg_server_debug/*.py. "
            "Phải bấm 'BẬT gửi lệnh' thì động cơ mới chạy.",
            foreground="#555",
            padding=(8, 0),
        ).grid(row=1, column=0, columnspan=2, sticky="w")

        self.panels = []
        for col, cfg in enumerate(LEGS):
            try:
                panel = LegPanel(self, cfg, self.status_q)
            except Exception as exc:
                ttk.Label(self, text=f"{cfg['side']}: lỗi đọc limit — {exc}").grid(
                    row=2, column=col
                )
                continue
            panel.grid(row=2, column=col, padx=8, pady=8, sticky="n")
            self.panels.append(panel)

        self._rate_t0 = time.time()
        self._build_orientation_row()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(1000 // SEND_HZ, self.tick)

    def home_all(self):
        """
        Đưa CẢ 2 CHÂN về home cùng lúc.

        Hai chân là 2 server độc lập nên lệnh đi song song, không phải chờ chân
        này xong mới tới chân kia. Mỗi panel tự chạy vòng _settle_sync riêng.
        """
        if not self.panels:
            return
        for panel in self.panels:
            panel.go_home()
        sides = " + ".join(p.side for p in self.panels)
        self.all_status.config(text=f"đang đưa {sides} về home…", foreground="#a60")
        self.after(500, self._watch_home_all)

    def _watch_home_all(self):
        """Chờ tới khi cả 2 chân báo xong rồi mới xoá dòng trạng thái chung."""
        busy = [p.side for p in self.panels if "đang về home" in p.status.cget("text")]
        if busy:
            self.all_status.config(
                text=f"đang đưa {' + '.join(busy)} về home…", foreground="#a60"
            )
            self.after(400, self._watch_home_all)
        else:
            self.all_status.config(text="✓ cả 2 chân đã về home", foreground="#0a6")

    def _build_orientation_row(self):
        """Hàng dưới: 3 cục phẳng 3D thể hiện orientation đọc từ IMU."""
        box = ttk.LabelFrame(
            self, text="Orientation từ IMU (khung baselink, đã bỏ yaw)", padding=8
        )
        box.grid(row=3, column=0, columnspan=2, padx=8, pady=(0, 8), sticky="we")

        self.views = {}
        self.view_lbl = {}
        for col, name in enumerate(["LEFT", "RIGHT", "FUSED"]):
            view = OrientationView(box, name)
            view.grid(row=0, column=col, padx=6)
            lbl = ttk.Label(box, text="—", font=("TkFixedFont", 9), foreground="#555")
            lbl.grid(row=1, column=col, pady=(2, 0))
            self.views[name] = view
            self.view_lbl[name] = lbl

        self.rate_lbl = ttk.Label(box, text="—", foreground="#888")
        self.rate_lbl.grid(row=2, column=0, columnspan=3, pady=(6, 0))
        self.after(1000 // VIEW_HZ, self._view_loop)

    def _view_loop(self):
        """Vòng vẽ 3D riêng, tách khỏi vòng gửi lệnh để chỉnh tần số độc lập."""
        self._update_orientation()
        self.after(1000 // VIEW_HZ, self._view_loop)

    def _update_orientation(self):
        raw = {p.side: p.last_quat for p in self.panels}

        # Đưa về khung baselink rồi bỏ yaw -> hai chân so sánh được với nhau
        proc = {
            side: strip_yaw(to_baselink(q)) if q else None for side, q in raw.items()
        }
        left, right = proc.get("LEFT"), proc.get("RIGHT")
        proc["FUSED"] = q_fuse(left, right) if left and right else (left or right)

        for name in ("LEFT", "RIGHT", "FUSED"):
            q = proc.get(name)
            self.views[name].draw(q)
            if q:
                r, p, _ = q_to_euler_deg(q)
                self.view_lbl[name].config(
                    text=f"R{r:+6.2f}°  P{p:+6.2f}°", foreground="#333"
                )

        # Lệch pitch giữa 2 chân - con số đang phải theo dõi
        if left and right:
            dp = q_to_euler_deg(left)[1] - q_to_euler_deg(right)[1]
            dr = q_to_euler_deg(left)[0] - q_to_euler_deg(right)[0]
            self.view_lbl["FUSED"].config(
                text=f"L-R:  dR{dr:+5.2f}°  dP{dp:+5.2f}°",
                foreground="#c00" if abs(dp) > 2 else "#333",
            )

        # Tần số THẬT của dữ liệu IMU về từ mỗi Pi (không phải tần số vẽ)
        now = time.time()
        if now - self._rate_t0 >= 1.0:
            parts = []
            for p in self.panels:
                hz = p.recv_count / (now - self._rate_t0)
                parts.append(f"{p.side} {hz:.0f} Hz")
                p.recv_count = 0
            self._rate_t0 = now
            self.rate_lbl.config(text="Dữ liệu IMU về:  " + "   |   ".join(parts))

    def tick(self):
        for panel in self.panels:
            panel.maybe_send()
        while True:
            try:
                side, kind, payload = self.status_q.get_nowait()
            except queue.Empty:
                break
            for panel in self.panels:
                if panel.side == side:
                    panel.on_feedback(kind, payload)
        self.after(1000 // SEND_HZ, self.tick)

    def on_close(self):
        for panel in self.panels:
            panel.shutdown()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
