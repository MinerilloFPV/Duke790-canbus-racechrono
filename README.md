# KTM 790 Duke — CAN Bus to RaceChrono (ESP32 BLE)

Reads KTM 790 Duke 2019 CAN bus frames via ESP32 TWAI and streams decoded
channels to the [RaceChrono](https://racechrono.app/) app over Bluetooth BLE.

## Files

| File | Description |
|------|-------------|
| `esp32.ino` | Arduino sketch — upload to ESP32 |
| `can_decoder.h` | CAN frame decoder (shared by sketch and tests) |
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

### Step 3 — Install RaceChrono library

**Tools → Manage Libraries** → search `RaceChrono` → install **arduino-RaceChrono by timurrrr**

### Step 4 — Open the sketch

**File → Open** → navigate to this folder and select `esp32.ino`.

Arduino IDE will automatically include `can_decoder.h` because it is in the
**same folder** as `esp32.ino`. Do not move them apart.

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
4. Channels will appear automatically

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

| Channel | Unit | Source |
|---------|------|--------|
| RPM | rpm | 0x120 D0–D1 |
| Throttle Position | % | 0x120 D2 |
| Gear | — | 0x129 D0 high nibble |
| Wheel Speed Front | km/h | 0x12B D0–D1 / 10 |
| Wheel Speed Rear | km/h | 0x12B D2–D3 / 10 |
| Lean Angle | deg | 0x12B 12-bit signed (D6 low + D7) |
| Pitch (Wheelie) | deg | 0x12B 12-bit signed (D5 + D6 high) |
| Brake Pressure Front | bar | 0x290 D0–D1 / 10 |
| Brake Pressure Rear | bar | 0x290 D2–D3 / 10 |
| MTC Intervention | — | 0x450 D2 bit 0 |
| Engine Temp | °C | 0x540 D6–D7 / 10 |

## References

- RaceChrono BLE DIY device: https://github.com/aollin/racechrono-ble-diy-device
- arduino-RaceChrono library: https://github.com/timurrrr/arduino-RaceChrono
