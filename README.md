# KTM 790 Duke — CAN Bus to RaceChrono (ESP32 BLE)

Reads KTM 790 Duke 2019 CAN bus frames via ESP32 TWAI and streams decoded
channels to the [RaceChrono](https://racechrono.app/) app over Bluetooth BLE.

## Files

| File | Description |
|------|-------------|
| `esp32/esp32.ino` | Arduino sketch — upload to ESP32 |
| `can_decoder.h` | CAN frame decoder used by unit tests only |
| `test_decoder.cpp` | Unit tests — compile and run on any PC, no hardware needed |

---

## 1. Running unit tests (PC — no hardware needed)

Tests verify the CAN decoding logic on your computer. No ESP32, no Arduino IDE,
no libraries required — only `g++`.

### macOS / Linux

```bash
# from the repo root:
cd Duke790-canbus-racechrono
g++ -std=c++17 -Wall -o test_decoder test_decoder.cpp && ./test_decoder
rm test_decoder   # clean up the binary
```

### Windows (Git Bash or WSL)

```bash
g++ -std=c++17 -Wall -o test_decoder.exe test_decoder.cpp && ./test_decoder.exe
```

### Expected output

```
============================================================
  KTM 790 CAN Decoder — unit tests
============================================================
=== ID 0x120 (RPM + TPS) ===
  [OK] RPM = 4653   ...
  [OK] TPS = 25.9%  ...
...
=== Value range validation (RaceChrono channel bounds) ===
  [OK] RPM                   last=15000.00  range=[0, 15000]
  [OK] Lean deg              last=80.00     range=[-90, 90]
  ...
============================================================
  RESULTS: 40 passed, 0 failed (total 40)
============================================================
```

Exit code `0` = all passed. Exit code `1` = at least one failure.

> **Run tests before every flash to ESP32.** If all 40 pass, the decoding
> logic is correct and the only remaining unknowns are hardware wiring and BLE.

---

## 2. Uploading to ESP32 (Arduino IDE)

### Step 1 — Install Arduino IDE

Download from https://www.arduino.cc/en/software (version 2.x recommended).

### Step 2 — Add ESP32 board support

1. Open Arduino IDE
2. **File → Preferences**
3. In *Additional boards manager URLs* add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. **Tools → Board → Boards Manager** → search `esp32` → install **esp32 by Espressif Systems**

### Step 3 — Install required libraries

In Arduino IDE open **Tools → Manage Libraries** and install both:

| Library | Author | Why |
|---------|--------|-----|
| **arduino-RaceChrono** | timurrrr | RaceChrono BLE protocol |
| **NimBLE-Arduino** | h2zero | BLE stack required by arduino-RaceChrono on ESP32 |

### Step 4 — Open the sketch

**File → Open** → navigate to the `esp32/` sub-folder and select `esp32.ino`.

### Step 5 — Select board and port

| Setting | Value |
|---------|-------|
| **Tools → Board** | ESP32 Dev Module (or your specific variant) |
| **Tools → Port** | the COM/tty port of your connected ESP32 |
| **Tools → Upload Speed** | 921600 |

### Step 6 — Upload

Click the **→ Upload** button (or `Ctrl+U` / `Cmd+U`).

After upload open **Tools → Serial Monitor** at **115200 baud**. You should see:

```
CAN Bus: OK - 500 kbps, listen-only
Bluetooth BLE: waiting for RaceChrono...
```

### Step 7 — Connect RaceChrono

1. Open RaceChrono on your phone
2. **Settings → Connections → Add BLE DIY device**
3. Select **KTM 790 DIY Track**

### Step 8 — Configure CAN-Bus channels in the app

The ESP32 forwards raw CAN frames to RaceChrono — the app decodes them.
Channels must be defined **once** inside the app:

1. In RaceChrono go to **Settings → Connections → KTM 790 DIY Track → CAN-Bus channels**
2. Tap **+** and add each channel from the table in the [RaceChrono channels](#racechrono-channels) section below

---

## Hardware wiring

- ESP32 (any variant with TWAI/CAN peripheral)
- CAN transceiver — recommended: **SN65HVD230** (3.3 V) or MCP2562

| ESP32 pin | Transceiver pin |
|-----------|----------------|
| GPIO 5 | TX (CTX) |
| GPIO 4 | RX (CRX) |
| 3.3 V | VCC |
| GND | GND |
| — | CANH → KTM CAN High |
| — | CANL → KTM CAN Low |

To change pins edit `esp32.ino`:
```cpp
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
```

---

## CAN IDs decoded

| CAN ID | Content | Rate |
|--------|---------|------|
| `0x120` | RPM, Throttle position (TPS) | 20 ms |
| `0x129` | Gear (high nibble D0) | 20 ms |
| `0x12B` | Wheel speed front/rear, Lean angle, Pitch angle | 10 ms |
| `0x290` | Brake pressure front/rear | 10 ms |
| `0x450` | Traction control button (D2 bit 0) | 50 ms |
| `0x540` | Coolant temperature (D6–D7 / 10.0) | 100 ms |

## RaceChrono channels

Configure these channels under **Settings → Connections → KTM 790 DIY Track → CAN-Bus channels** in the RaceChrono app.

> All multi-byte values are **big-endian**.

| Channel name | Unit | CAN ID | Offset (bytes) | Length (bytes) | Type | Multiplier | Notes |
|---|---|---|---|---|---|---|---|
| RPM | rpm | `0x120` | 0 | 2 | unsigned | 1 | 0–15 000 rpm |
| Throttle Position | % | `0x120` | 2 | 1 | unsigned | 100/255 ≈ 0.3922 | 0–100 % |
| Gear | | `0x129` | 0 | 1 | unsigned | 1 | high nibble only — set bit mask `0xF0`, shift right 4 |
| Wheel Speed Front | km/h | `0x12B` | 0 | 2 | unsigned | 1/16 = 0.0625 | |
| Wheel Speed Rear | km/h | `0x12B` | 2 | 2 | unsigned | 1/16 = 0.0625 | |
| Pitch (Wheelie) | deg | `0x12B` | 5 | 2 | signed 12-bit | 1 | bits [11:0] = D5[7:0]\|D6[7:4] |
| Lean Angle | deg | `0x12B` | 6 | 2 | signed 12-bit | 1 | bits [11:0] = D6[3:0]\|D7[7:0] |
| Brake Pressure Front | bar | `0x290` | 0 | 2 | unsigned | 0.1 | |
| Brake Pressure Rear | bar | `0x290` | 2 | 2 | unsigned | 0.1 | |
| MTC Intervention | | `0x450` | 2 | 1 | unsigned | 1 | bit 0 only (1 = active) |
| Engine Temp | °C | `0x540` | 6 | 2 | unsigned | 0.1 | |

> **Gear note:** RaceChrono does not support bit-shift natively. Use multiplier `1` and set the *bit mask* field to `0xF0` with *right shift* `4` if your app version supports it; otherwise leave gear as raw byte (values 0–6 become 0, 16, 32 … — still usable as a relative indicator).

## References

- RaceChrono BLE DIY device: https://github.com/aollin/racechrono-ble-diy-device
- arduino-RaceChrono library: https://github.com/timurrrr/arduino-RaceChrono
