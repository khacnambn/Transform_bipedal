# Bipedal Robot — Kiến trúc đồng bộ 2 chân module hóa

## Tổng quan dự án

Robot đi 2 chân (bipedal), thiết kế **module hóa** — mỗi chân là 1 module điện tử độc lập hoàn toàn. Mỗi module có thể chạy standalone (1 chân riêng cho 1 tác vụ) hoặc ghép 2 module lại thành robot bipedal.

**Target**: robot ~3 kg, dáng đi walking gait, điều khiển bằng RL policy hoặc inverse kinematics.

---

## Kiến trúc phần cứng (mỗi chân)

### Sơ đồ khối

```
┌─────────────────────────────────────────────────────────┐
│                     MỖI CHÂN MODULE                     │
│                                                         │
│  ┌──────────┐    UART/USB     ┌────────────────────┐    │
│  │ SBC      │ ◄─────────────► │ STM32 (F4 hoặc H7) │   │
│  │ Rasp Pi  │   lệnh/data     │                    │    │
│  │          │                  │  ├─ SPI/I2C ► IMU  │    │
│  │ chạy     │                  │  │  (ICM-20948)    │    │
│  │ policy   │                  │  │                  │    │
│  │ (RL/IK)  │                  │  ├─ UART ► Waveshare│   │
│  └──────────┘                  │  │  Servo Adapter   │    │
│       │                        │  │  ├► STS3120 ×3   │    │
│       │ WiFi/WebSocket         │  │  └► STS3215 ×3   │    │
│       │ (lên PC nếu cần)       │  │    (daisy-chain) │    │
│       │                        │  │                  │    │
│       │                        │  ├─ SYNC pin ──────┼──── pogo pin
│       │                        │  ├─ UART TX ───────┼──── pogo pin  ► tới chân kia
│       │                        │  ├─ UART RX ───────┼──── pogo pin
│       │                        │  └─ GND ───────────┼──── pogo pin
│       │                        └────────────────────┘    │
│                                                         │
│  ┌──────────────────────────────────────────┐           │
│  │ Nguồn: Pin 12V 3A                        │           │
│  │  ├─ Branch 1: UBEC 5V 5A → Pi            │           │
│  │  └─ Branch 2: 12V trực tiếp → servo bus  │           │
│  └──────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────┘
```

### Linh kiện chính

| Thành phần | Model | Ghi chú |
|---|---|---|
| SBC | Raspberry Pi | Chạy Linux, xử lý high-level (policy, planning) |
| MCU | STM32F4 (hoặc H7) | Bare-metal, real-time control loop |
| IMU | ICM-20948 | 9-DOF (accel + gyro + mag), giao tiếp SPI hoặc I2C |
| Servo lớn | Feetech STS3120 ×3/chân | 12V, 120 kg·cm, TTL serial bus |
| Servo nhỏ | Feetech STS3215 ×3/chân | 12V, 30 kg·cm, TTL serial bus |
| Servo driver | Waveshare Bus Servo Adapter (A) | Passive UART adapter, KHÔNG có MCU onboard |
| Nguồn | Pin 12V 3A + UBEC 5V 5A | Mỗi chân có nguồn riêng |

### Servo topology

6 servo mỗi chân, daisy-chain nối tiếp từ ankle → hip trên 1 bus TTL half-duplex duy nhất qua Waveshare adapter. STM32 gửi lệnh vị trí qua UART (ở chế độ half-duplex) → adapter chuyển thành tín hiệu TTL bus → tất cả servo nhận lệnh theo ID.

---

## HARD CONSTRAINT: Module hóa & dock/undock

**Đây là ràng buộc không thể thay đổi** — 2 chân phải hoàn toàn độc lập về điện. Khi ghép (dock) thành bipedal, chúng kết nối qua **pogo-pin dock connector**. Khi tách (undock), mỗi chân chạy standalone.

Hệ quả:

- Không có dây cố định xuyên qua 2 chân.
- Đồng bộ chỉ cần thiết khi docked (bipedal mode).
- Khi undocked, mỗi chân tự quản — không cần sync.
- **Mất sync khi đang đi bipedal = ngã** → safety-critical.

---

## Tại sao KHÔNG dùng CAN bus

### Vấn đề termination khi dock/undock

CAN bus yêu cầu **2 điện trở 120Ω ở 2 đầu bus** để phối hợp trở kháng (impedance matching).

**Khi docked** (4 node — 2 STM32 mỗi bên):

```
[A1]──120Ω──bus──[A2]───connector───[B1]──bus──120Ω──[B2]
                          ✓ OK: 120Ω ở 2 đầu ngoài cùng
```

**Khi undocked** (mỗi bên 2 node, đầu connector hở):

```
Side A:  [A1]──120Ω──bus──[A2]──đầu hở    ← thiếu termination
Side B:  đầu hở──[B1]──bus──120Ω──[B2]    ← thiếu termination
```

Đầu hở → phản xạ tín hiệu → bit error → không đáng tin cậy. Nếu thêm 120Ω ở đầu connector mỗi bên thì khi docked lại bị **over-terminated** (3 điểm termination → suy hao tín hiệu).

### Các lý do khác

- CAN cần transceiver IC riêng (MCP2551, SN65HVD230...) — thêm linh kiện.
- CAN là half-duplex, shared bus — overkill cho point-to-point 2 node.
- Arbitration, CRC, bit stuffing — overhead không cần thiết.
- Độ chính xác sync qua CAN message timestamp: ~10–50 µs, kém hơn hardware timer capture (~1–5 µs).

---

## Phương án chọn: UART + Hardware SYNC Pulse

### Tại sao UART + SYNC?

| Tiêu chí | CAN bus | UART + SYNC pulse |
|---|---|---|
| Số dây qua dock | 3 (CAN_H, CAN_L, GND) | 4 (TX, RX, SYNC, GND) |
| IC phụ | CAN transceiver ×2 | Không cần |
| Termination | Phải switch khi dock/undock | Không cần |
| Độ chính xác sync | ~10–50 µs | ~1–5 µs |
| Duplex | Half-duplex | Full-duplex |
| Bandwidth | 1 Mbps max | 2+ Mbps |
| Firmware phức tạp | CAN stack + filter + mailbox | UART DMA + 1 ISR |

### Kết nối vật lý qua pogo-pin dock

```
STM32_A (chân trái)              STM32_B (chân phải)
┌──────────┐                     ┌──────────┐
│   TX_A ──┼──── pogo pin ──────┼── RX_B   │
│   RX_A ──┼──── pogo pin ──────┼── TX_B   │
│  SYNC ───┼──── pogo pin ──────┼── SYNC   │
│   GND ───┼──── pogo pin ──────┼── GND    │
└──────────┘                     └──────────┘

(SYNC: Output Compare trên master, Input Capture trên slave)
(TX/RX: UART cross-link, 1–2 Mbps, DMA transfer)
```

UART 3.3V TTL, 2 STM32 cùng logic level → nối thẳng, không cần level shifter hay transceiver.

---

## Kiến trúc đồng bộ 3 tầng

```
┌──────────────────────────────────────────────────────┐
│  TẦNG 3 — SERVO SYNC                                │
│  Cả 2 bên ra lệnh servo tại cùng master tick        │
│  Yêu cầu: clock đã sync + data đã trao đổi xong    │
├──────────────────────────────────────────────────────┤
│  TẦNG 2 — DATA SYNC                                 │
│  Trao đổi IMU data qua UART mỗi chu kỳ              │
│  Yêu cầu: clock đã sync để timestamp có nghĩa       │
├──────────────────────────────────────────────────────┤
│  TẦNG 1 — CLOCK SYNC (nền tảng)                     │
│  SYNC pulse: master phát edge → slave capture        │
│  Kết quả: 2 MCU thống nhất timeline chung            │
└──────────────────────────────────────────────────────┘
```

### Tầng 1 — Clock Sync (SYNC Pulse)

#### Vấn đề

Mỗi STM32 có crystal oscillator riêng. Dù cùng tần số danh định (vd 72 MHz), sai số ±5–50 ppm là bình thường. Ở 5 ppm, sau 1 giây clock slave lệch 360 tick so với master → servo 2 bên ra lệnh lệch dần → robot ngã.

#### Cơ chế

**Master** (fix cứng = STM32 chân trái):

- Cấu hình Timer Output Compare trên chân SYNC.
- Phát rising edge đều đặn mỗi 1ms (tần số control loop = 1 kHz).

**Slave** (STM32 chân phải):

- Cấu hình Timer Input Capture trên chân SYNC.
- Mỗi rising edge, hardware timer tự động capture counter value vào thanh ghi CCR — không cần CPU, chính xác đến 1 clock tick (~14 ns ở 72 MHz).

#### Slave tính toán

```
Edge #1: CCR = 100,000    (local counter khi edge đến)
Edge #2: CCR = 172,003    (1ms master sau)
Edge #3: CCR = 244,006
Edge #4: CCR = 316,009

Chu kỳ đo được: 172,003 - 100,000 = 72,003 slave ticks
Chu kỳ kỳ vọng (nếu slave chính xác): 72,000 ticks
→ Drift: +3 tick/ms (slave nhanh hơn master 41.7 ppm)
```

Slave xây **bảng quy đổi**:

- Master tick 0 ↔ local counter 100,000.
- Mỗi 1ms master = 72,003 local ticks (không phải 72,000).
- Target tick N từ SBC → `local_target = 100,000 + (N × 72,003)`.

#### Liên tục cập nhật

Crystal drift thay đổi theo nhiệt độ → slave đo lại period mỗi edge, dùng **moving average** hoặc **linear regression** lọc nhiễu:

```c
void TIM3_IRQHandler(void) {
    uint32_t capture = TIM3->CCR1;
    uint32_t period = capture - last_capture;
    last_capture = capture;

    // Trung bình trượt lọc nhiễu
    avg_period = avg_period * 0.99 + period * 0.01;
    master_tick_count++;
}
```

#### Kết quả

Slave không chỉnh clock vật lý — crystal không điều chỉnh được. Thay vào đó, slave **đo offset + drift** rồi **tính toán quy đổi** giữa local counter và master tick. Sai số đạt ~1–5 µs.

---

### Tầng 2 — Data Sync (trao đổi IMU qua UART)

Mỗi chu kỳ control, 2 STM32 gửi IMU data cho nhau qua UART cross-link:

```
STM32_A                              STM32_B
   │                                     │
   ├── đọc IMU_A (SPI/I2C)              ├── đọc IMU_B
   │                                     │
   ├── UART TX ──────────────────────►   ├── UART RX: nhận IMU_A
   │                                     │
   ├── UART RX ◄──────────────────────   ├── UART TX: gửi IMU_B
   │                                     │
   └── có cả IMU_A + IMU_B              └── có cả IMU_A + IMU_B
```

#### Packet format

```c
typedef struct {
    uint32_t tick;       // master tick number (4 bytes)
    int16_t  accel[3];   // accelerometer x, y, z (6 bytes)
    int16_t  gyro[3];    // gyroscope x, y, z (6 bytes)
    uint8_t  checksum;   // error detection (1 byte)
} ImuPacket;             // Tổng: 17 bytes
```

17 bytes × 10 bits/byte = 170 bits. Ở UART 1 Mbps → truyền hết trong **170 µs**, dư sức trong chu kỳ 1ms.

#### Tại sao cần trao đổi

Controller (RL policy hoặc IK) cần biết **tư thế toàn thân** — không chỉ IMU chân mình mà cả chân kia. Ví dụ: chân stance cần biết chân swing đang nghiêng bao nhiêu để giữ thăng bằng.

---

### Tầng 3 — Servo Sync (ra lệnh đồng thời)

#### Vấn đề

SBC (Pi) chạy Linux — không real-time. Khi Pi gửi lệnh xuống 2 STM32 qua UART/USB, lệnh đến mỗi bên có thể lệch vài ms. Nếu STM32 thực thi ngay → 2 bên lệch.

#### Giải pháp: lệnh kèm target tick

SBC gửi lệnh kèm **master tick** chỉ định thời điểm thực hiện:

```c
typedef struct {
    uint32_t target_tick;    // "thực hiện tại master tick này"
    int16_t  angles[6];     // góc target cho 6 servo
} ServoCommand;
```

STM32 nhận lệnh → **không chạy ngay** → đợi đến đúng target tick → gửi lệnh servo:

```
Timeline (master ticks):

tick:  4990  4991  ...  4997  4998  4999  5000
         │                     │              │
    STM32_A nhận          STM32_B nhận    CẢ 2 cùng gửi
    (đến sớm,             (đến trễ hơn,   lệnh servo
     buffer + đợi)         vẫn kịp)       → ĐỒNG THỜI
```

SBC không cần real-time — chỉ cần gửi lệnh **đủ sớm** trước target tick. Vì clock 2 bên đã sync (tầng 1), cả 2 biết "tick 5000 là lúc nào" → bắn lệnh servo đồng thời, sai số ~µs.

---

## Full control loop — 1 chu kỳ (1ms)

```
     SBC (Pi)               STM32_A (master)          STM32_B (slave)
        │                         │                          │
        │                   SYNC pulse ──────────────────►   │
        │                         │                          │
        │                  1. Đọc IMU_A                1. Đọc IMU_B
        │                    (SPI, ~200µs)               (SPI, ~200µs)
        │                         │                          │
        │                  2. UART: gửi IMU_A ──────────►    │ nhận IMU_A
        │                     nhận IMU_B ◄──────────────     │ gửi IMU_B
        │                    (~170µs)                   (~170µs)
        │                         │                          │
        │                  3. Gửi {IMU_A, IMU_B}             │
        │ ◄─────────────────  lên SBC                        │
        │                         │                          │
        │  4. Chạy policy         │                          │
        │     (RL hoặc IK)        │                          │
        │     ~vài trăm µs        │                          │
        │                         │                          │
        │  5. Gửi lệnh xuống:    │                          │
        │     {tick=N+k,          │                          │
        │      angles_A[6]} ────► │ buffer, đợi              │
        │     {tick=N+k,          │                          │
        │      angles_B[6]} ────► │────────────────────►     │ buffer, đợi
        │                         │                          │
        │                   tick N+k đến:              tick N+k đến:
        │                   GỬI 6 SERVO ◄──cùng lúc──► GỬI 6 SERVO
```

**Timing budget (ước tính trong 1ms):**

| Giai đoạn | Thời gian | Ghi chú |
|---|---|---|
| Đọc IMU (SPI) | ~200 µs | ICM-20948, DMA |
| Trao đổi IMU qua UART | ~170 µs | 17 bytes × 2 chiều |
| Gửi lên SBC | ~50 µs | UART/USB |
| Policy trên SBC | ~200–500 µs | Tùy model complexity |
| Gửi lệnh xuống STM32 | ~50 µs | |
| **Tổng** | **~670–970 µs** | **Vừa trong 1ms** |

> **Lưu ý:** Nếu policy phức tạp (RL model lớn), có thể cần giảm tần số control loop xuống 500 Hz (2ms) hoặc pipeline xử lý (cycle N policy chạy song song với cycle N+1 IMU read).

---

## Pogo-Pin Dock — Thiết kế vật lý

### Yêu cầu tối thiểu: 4 pin

| Pin | Chức năng | Hướng (A→B) |
|---|---|---|
| GND | Mass chung | Bidirectional |
| SYNC | Clock sync pulse | Master → Slave |
| TX_A / RX_B | UART data A→B | A → B |
| TX_B / RX_A | UART data B→A | B → A |

### Pin bổ sung khuyến nghị: +2 pin

| Pin | Chức năng | Lý do |
|---|---|---|
| DOCK_SENSE | Detect kết nối | Biết khi nào bắt đầu/dừng sync |
| SYNC_REDUNDANT | Backup SYNC | Phòng 1 pin tiếp xúc kém |

**Tổng: 6 pogo pin.**

---

## Critical Issues

### 1. Rung lắc pogo pin khi đi (CRITICAL)

Robot đi → rung → pogo pin có thể **mất tiếp xúc thoáng qua** (contact bounce).

**Hậu quả:**

- SYNC pulse mất 1–2 edge → slave mất đồng bộ tạm thời.
- UART data bị corrupt giữa chừng → packet lỗi.
- Nếu không xử lý → servo 2 bên lệch → ngã.

**Giải pháp:**

- **Schmitt trigger + RC filter** trên dây SYNC: lọc bounce, chỉ cho qua edge sạch.
- **Pogo pin redundant** (2 pin SYNC song song): 1 pin mất tiếp xúc thì còn pin kia.
- **GND chắc chắn**: dùng nhiều pin GND hoặc pin GND lớn → giảm khả năng mất mass.
- **Holdover mode** (xem bên dưới).
- **Cơ khí**: thiết kế dock có cơ cấu khóa (latch), lò xo pogo pin đủ mạnh, giảm rung.

### 2. Holdover — khi mất SYNC tạm thời (CRITICAL)

Nếu SYNC edge không đến trong vài chu kỳ (pogo bounce, hoặc tháo dock):

```
SYNC:  ──┐  ┌──┐  ┌──┐       (mất 3 edge)       ┌──┐  ┌──
          └──┘  └──┘  └────────────────────────────┘  └──┘
                       ↑                            ↑
                  mất tiếp xúc               phục hồi tiếp xúc
```

**Slave phải:**

1. **Detect** mất SYNC: nếu Input Capture không bắt được edge trong >2ms (timeout) → chuyển sang holdover.
2. **Holdover**: tiếp tục chạy theo **drift rate đã biết** (avg_period từ lần đo cuối). Sai số tích lũy dần (~5 µs/s ở 5 ppm) nhưng đủ giữ sync trong vài trăm ms.
3. **Re-sync** khi edge quay lại: cập nhật offset + drift từ edge mới.
4. **Timeout cứng**: nếu mất SYNC > 500ms–1s → coi như undocked → dừng bipedal mode, chuyển standalone.

```c
#define SYNC_TIMEOUT_TICKS  144000  // 2ms ở 72MHz
#define HOLDOVER_MAX_MS     500

void control_loop(void) {
    if (sync_lost_duration > HOLDOVER_MAX_MS) {
        enter_standalone_mode();  // dừng bipedal, an toàn
    } else if (sync_lost) {
        // Holdover: dùng avg_period đã biết để ước tính tick
        estimated_master_tick += 1;  // giả sử 1ms/tick
    } else {
        // Bình thường: dùng capture value chính xác
        update_sync_from_capture();
    }
}
```

### 3. Master/Slave election (MODERATE)

Phương án đơn giản nhất: **fix cứng** — chân trái luôn master, chân phải luôn slave. Đủ tốt cho 2 node.

Nếu cần linh hoạt hơn (2 chân giống hệt, không phân biệt trái/phải):

- Dùng pin DOCK_SENSE + pull-up/pull-down khác nhau mỗi bên.
- Hoặc handshake qua UART sau khi dock: trao đổi ID ngẫu nhiên, ID lớn hơn → master.

### 4. Crystal drift theo nhiệt độ (MODERATE)

Drift rate thay đổi khi board nóng lên (servo hoạt động, Pi tỏa nhiệt). Nếu slave chỉ đo drift 1 lần → dần dần sai.

**Giải pháp**: đo liên tục mỗi SYNC edge, dùng moving average → tự bám theo drift thay đổi. Đây đã là behavior mặc định của tầng 1.

### 5. Feetech servo bus latency (LOW-MODERATE)

Servo STS dùng TTL half-duplex bus. Gửi lệnh cho 6 servo tuần tự:

```
Gửi lệnh servo 1 → đợi ACK → gửi lệnh servo 2 → đợi ACK → ... → servo 6
```

Ở baud rate 1 Mbps, mỗi lệnh + ACK ~100 µs → 6 servo ~600 µs. Thời gian này **phải tính vào timing budget**.

**Tối ưu:** Dùng **sync write** (Feetech hỗ trợ broadcast command — gửi lệnh cho tất cả servo trong 1 packet, không đợi ACK từng con) → giảm xuống ~200 µs cho cả 6 servo.

### 6. Nguồn điện (LOW-MODERATE)

12V 3A battery cho mỗi chân. Budget:

- 6 servo: worst case ~2A (stall) nhưng trung bình ~0.5–1A khi đi.
- Pi: ~0.5A qua UBEC.
- STM32 + IMU: ~50 mA (không đáng kể).

**Rủi ro:** Nếu servo stall (bị kẹt) → kéo dòng cao → pin 3A có thể không đủ → voltage drop → Pi brown-out hoặc STM32 reset → mất sync, mất kiểm soát.

**Giải pháp:** Tách nguồn servo và nguồn logic (Pi + STM32) bằng 2 đường riêng từ pin, thêm tụ lọc lớn (bulk capacitor 1000 µF+) trên đường logic.

### 7. SBC (Pi) — tính không real-time (MODERATE)

Linux trên Pi **không đảm bảo latency**. Policy inference có thể bị jitter vài ms do context switch, GC, I/O.

**Giải pháp:**

- Lệnh từ SBC luôn kèm target tick "thực hiện tại tick N+k" (k = 2–5 chu kỳ buffer).
- STM32 buffer lệnh, thực thi tại đúng tick → jitter của Pi bị **hấp thụ hoàn toàn** bởi buffer.
- Nếu lệnh đến trễ hơn target tick → STM32 giữ lệnh cũ (hold last command) hoặc interpolate.

---

## Phương án thay thế đã đánh giá

### 1. CAN bus

**Kết luận: LOẠI.** Termination không tương thích với dock/undock. Thêm IC, thêm phức tạp, không có lợi thế gì cho 2-node point-to-point.

### 2. ESP-NOW wireless sync (PA2 trong thiết kế trước)

Mỗi chân thêm 1 ESP32 chuyên làm sync qua ESP-NOW (WiFi protocol, không cần router).

- **Ưu điểm**: Không cần dây vật lý → sync cả khi chưa dock (chuẩn bị trước).
- **Nhược điểm**: Thêm 1 MCU (ESP32) mỗi chân, jitter radio ~100–500 µs (kém hơn hardware sync), cần quản lý thêm 1 chip.
- **Kết luận**: Viable nhưng phức tạp hơn pogo-pin UART+SYNC. Phù hợp nếu sau này muốn sync ở khoảng cách xa hoặc không dock được.

### 3. WebSocket qua Pi WiFi

Phương án gốc ban đầu. 2 Pi giao tiếp WebSocket qua WiFi.

- **Kết luận: LOẠI cho sync.** Linux + WiFi + Pi→MCU hop = ms-class latency. Không đủ chính xác cho servo sync. Có thể giữ cho data comms high-level (gửi policy, logging) nhưng không làm clock sync.

---

## Tham khảo

- **MRTP** (Modular Robot Time Protocol) — Naz/Piranda/Bourgeois/Goldstein (FEMTO-ST + CMU). Open-access trên HAL (hal-01948896). Blueprint cho sync giữa các module robot: master election + low-level timestamping + delay compensation + linear regression clock-skew compensation.
- **ESPNowMeshClock library** — Implementation nhẹ của ý tưởng MRTP cho ESP-NOW.
- **US11623345 patent** — Master→slave sync handshake protocol sau khi docking.
- **Mayoral-Vilches H-ROS paper** — PTP/gPTP/TSN qua Ethernet, sub-µs accuracy (con đường wired nếu nâng cấp lên Ethernet sau này).
