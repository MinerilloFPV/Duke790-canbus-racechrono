#pragma once
// =============================================================================
// can_decoder.h — CAN frame decoder for KTM 790 Duke 2019 (Bosch ME17.9.21)
//
// Usage in Arduino:  #include "can_decoder.h"
// Usage in tests:    #include "Duke790-canbus-racechrono/can_decoder.h"
//
// processCanFrame() takes an id + 8 data bytes and calls
// rc.updateCanChannel() via global channel indices ch_*.
// This file does not declare rc or ch_* — they must be visible in the
// compilation unit.
// =============================================================================

#include <cstdint>

// -----------------------------------------------------------------------------
// Convert a 12-bit unsigned value to signed (two's complement).
// Used for lean (left/right tilt) and pitch (front/rear tilt) from ID 0x12B.
// -----------------------------------------------------------------------------
static inline int signed12(int v) {
    return (v & 0x800) ? v - 4096 : v;
}

// -----------------------------------------------------------------------------
// processCanFrame — central decoder
//
// Bugs fixed compared to the original GitHub code:
//   0x120 — added TPS from D2 (was missing, incorrectly placed in 0x12A)
//   0x129 — gear from high nibble of D0, not the whole byte
//   0x12A — removed (contains map/state flags, not raw TPS)
//   0x12B — wheels: D0-D1=front, D2-D3=rear (were swapped)
//   0x12B — lean/pitch: 12-bit two's complement from D5-D7 (was D4/D5 int8)
//   0x290 — removed bogus ABS flag from D4 (D4 is always 0x00)
//   0x450 — TC button from D2 bit0 (was D0, which is the fixed header 0x10)
//   0x540 — coolant temp from D6-D7 / 10.0 (was D0-40, D0 is fixed header 0x02)
// -----------------------------------------------------------------------------
static void processCanFrame(uint32_t id, uint8_t* data) {
    switch (id) {

        case 0x120: {
            // D0-D1: RPM (big-endian uint16, 0-15000)
            // D2:    Throttle position (0-255, where 255 = 100%)
            // D3 bit4: kill switch (1=RUN, 0=STOP) — not used by RaceChrono
            // D4 bit0: ride map (0=Mode1, 1=Mode2) — not used by RaceChrono
            // D7: sequence counter — ignored
            uint16_t rpm = (uint16_t)((data[0] << 8) | data[1]);
            float    tps = data[2] / 2.55f;
            rc.updateCanChannel(ch_rpm, (float)rpm);
            rc.updateCanChannel(ch_tps, tps);
            break;
        }

        case 0x129: {
            // D0 high nibble (B7-B4): gear number (0=Neutral, 1-6=gears)
            // D0 bit3: clutch state (1=engaged) — not used by RaceChrono
            // D7: sequence counter — ignored
            uint8_t gear = (data[0] >> 4) & 0x0F;
            rc.updateCanChannel(ch_gear, (float)gear);
            break;
        }

        // 0x12A: throttle state flags (open/closed) + requested map.
        // Does NOT contain raw TPS (TPS is in 0x120 D2) — skip.

        case 0x12B: {
            // D0-D1: front wheel speed (big-endian uint16 / 16.0 = km/h)
            // D2-D3: rear wheel speed  (big-endian uint16 / 16.0 = km/h)
            // D4:    unknown — ignored
            // Tilt (front/rear, pitch): 12-bit signed two's complement
            //   = D5[7:0] as bits [11:4], D6[7:4] as bits [3:0]
            // Lean (left/right):        12-bit signed two's complement
            //   = D6[3:0] as bits [11:8], D7[7:0] as bits [7:0]
            float v_front = (float)(((uint16_t)data[0] << 8) | data[1]) / 16.0f;
            float v_rear  = (float)(((uint16_t)data[2] << 8) | data[3]) / 16.0f;
            int tilt_raw  = (data[5] << 4) | ((data[6] >> 4) & 0x0F);
            int lean_raw  = ((data[6] & 0x0F) << 8) | data[7];
            rc.updateCanChannel(ch_v_front, v_front);
            rc.updateCanChannel(ch_v_rear,  v_rear);
            rc.updateCanChannel(ch_pitch,   (float)signed12(tilt_raw));
            rc.updateCanChannel(ch_lean,    (float)signed12(lean_raw));
            break;
        }

        case 0x290: {
            // D0-D1: front brake pressure (big-endian uint16 / 10.0)
            // D2-D3: rear brake pressure  (big-endian uint16 / 10.0)
            // D4-D7: always 0x00 — ABS flag location unknown (different ID)
            float brk_f = (float)(((uint16_t)data[0] << 8) | data[1]) / 10.0f;
            float brk_r = (float)(((uint16_t)data[2] << 8) | data[3]) / 10.0f;
            rc.updateCanChannel(ch_brk_f, brk_f);
            rc.updateCanChannel(ch_brk_r, brk_r);
            break;
        }

        case 0x450: {
            // D0-D1: fixed header 0x10 0x42 — ignored
            // D2 bit0: TC button (1=pressed, 0=released)
            // D4:      requested map (0x00=Mode1, 0x09=Mode2) — not used
            uint8_t tc = data[2] & 0x01;
            rc.updateCanChannel(ch_mtc_act, (float)tc);
            break;
        }

        case 0x540: {
            // D0:    fixed header 0x02 — ignored
            // D1-D2: RPM (same as 0x120 but at 100ms) — ignored (duplicate)
            // D3 low nibble: gear (same as 0x129) — ignored (duplicate)
            // D4 bit0: sidestand UP/DOWN — not used
            // D5:    always 0x00
            // D6-D7: coolant temperature (big-endian uint16 / 10.0 °C)
            uint16_t raw_temp = (uint16_t)((data[6] << 8) | data[7]);
            float temp = raw_temp / 10.0f;
            rc.updateCanChannel(ch_temp_eng, temp);
            break;
        }
    }
}
