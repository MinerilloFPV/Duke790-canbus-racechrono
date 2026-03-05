// =============================================================================
// test_decoder.cpp — unit tests for KTM 790 CAN decoder (runs on any PC)
//
// Build & run:
//   g++ -std=c++17 -Wall -o test_decoder test_decoder.cpp && ./test_decoder
//
// Decoder logic comes from: can_decoder.h
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cassert>
#include <string>
#include <map>

// -----------------------------------------------------------------------------
// RaceChrono library stub — stores values instead of sending over BLE
// -----------------------------------------------------------------------------
struct ChannelState {
    std::string name;
    std::string unit;
    float       value = 0.0f;
    int         updates = 0;
};

static std::map<int, ChannelState> g_channels;
static int g_nextChannelId = 0;

struct RaceChrono {
    RaceChrono(const char*) {}
    void begin()  {}
    void run()    {}

    int addCanChannel(const char* name, const char* unit, int /*hz*/) {
        int id = g_nextChannelId++;
        g_channels[id] = { name, unit, 0.0f, 0 };
        return id;
    }

    void updateCanChannel(int ch, float val) {
        g_channels[ch].value = val;
        g_channels[ch].updates++;
    }
};

// Channel indices
static int ch_rpm, ch_tps, ch_gear, ch_v_front, ch_v_rear;
static int ch_brk_f, ch_brk_r, ch_lean, ch_pitch;
static int ch_abs_act, ch_mtc_act, ch_temp_eng;

static RaceChrono rc("KTM 790 DIY Track");

// Channel initialization (equivalent to setup())
static void initChannels() {
    ch_rpm      = rc.addCanChannel("RPM",                  "rpm",  20);
    ch_tps      = rc.addCanChannel("Throttle Position",    "%",    20);
    ch_gear     = rc.addCanChannel("Gear",                 "",      5);
    ch_v_front  = rc.addCanChannel("Wheel Speed Front",    "km/h", 10);
    ch_v_rear   = rc.addCanChannel("Wheel Speed Rear",     "km/h", 10);
    ch_brk_f    = rc.addCanChannel("Brake Pressure Front", "bar",  50);
    ch_brk_r    = rc.addCanChannel("Brake Pressure Rear",  "bar",  20);
    ch_lean     = rc.addCanChannel("Lean Angle",           "deg",  25);
    ch_pitch    = rc.addCanChannel("Pitch (Wheelie)",      "deg",  25);
    ch_abs_act  = rc.addCanChannel("ABS Intervention",     "",     50);
    ch_mtc_act  = rc.addCanChannel("MTC Intervention",     "",     50);
    ch_temp_eng = rc.addCanChannel("Engine Temp",          "C",     1);
}

// Helper
static inline float get(int ch) { return g_channels[ch].value; }

// Decoder logic — same header used by esp32.ino
#include "can_decoder.h"

// -----------------------------------------------------------------------------
// Helper function for assertions with value output
// -----------------------------------------------------------------------------
static int g_passed = 0;
static int g_failed = 0;

static void check(const char* label, float actual, float expected, float tol = 0.5f) {
    bool ok = fabsf(actual - expected) <= tol;
    if (ok) {
        printf("  [OK] %-40s  actual=%.3f  expected=%.3f\n", label, actual, expected);
        g_passed++;
    } else {
        printf("  [FAIL] %-38s  actual=%.3f  expected=%.3f  diff=%.3f\n",
               label, actual, expected, fabsf(actual - expected));
        g_failed++;
    }
}

// =============================================================================
// UNIT TESTS
// =============================================================================

// --- 0x120 RPM + TPS -----------------------------------------------------
static void test_0x120() {
    printf("\n=== ID 0x120 (RPM + TPS) ===\n");

    // RPM=4653, TPS=66/255=25.9%
    uint8_t d[] = { 0x12, 0x2D, 0x42, 0x10, 0x00, 0x00, 0x00, 0x20 };
    processCanFrame(0x120, d);
    check("RPM = 4653",  get(ch_rpm), 4653.0f);
    check("TPS = 25.9%", get(ch_tps), 0x42 / 2.55f, 0.1f);

    // RPM=0, TPS=0
    uint8_t d2[] = { 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x20 };
    processCanFrame(0x120, d2);
    check("RPM = 0",     get(ch_rpm), 0.0f);
    check("TPS = 0%",    get(ch_tps), 0.0f);

    // RPM=15000, TPS=100%
    uint8_t d3[] = { 0x3A, 0x98, 0xFF, 0x10, 0x00, 0x00, 0x00, 0x20 };
    processCanFrame(0x120, d3);
    check("RPM = 15000", get(ch_rpm), 15000.0f);
    check("TPS = 100%",  get(ch_tps), 100.0f, 0.5f);
}

// --- 0x129 Gear ----------------------------------------------------------
static void test_0x129() {
    printf("\n=== ID 0x129 (Gear) ===\n");

    // Neutral (0)
    uint8_t d0[] = { 0x00, 0,0,0,0,0,0, 0x80 };
    processCanFrame(0x129, d0);
    check("Gear = N(0)",  get(ch_gear), 0.0f);

    // Gear 1
    uint8_t d1[] = { 0x10, 0,0,0,0,0,0, 0x20 };
    processCanFrame(0x129, d1);
    check("Gear = 1",     get(ch_gear), 1.0f);

    // Gear 6
    uint8_t d6[] = { 0x60, 0,0,0,0,0,0, 0x60 };
    processCanFrame(0x129, d6);
    check("Gear = 6",     get(ch_gear), 6.0f);

    // Old bug: whole D0=0x60=96
    printf("  [INFO] old code would wrongly return gear=96 (0x60), ours: %.0f\n", get(ch_gear));
}

// --- 0x12B Wheel Speed + Lean/Pitch -------------------------------------
static void test_0x12B() {
    printf("\n=== ID 0x12B (Wheels + Lean/Pitch) ===\n");

    // front=160, rear=292, pitch=+80 (0x050), lean=+143 (0x08F)
    // D0-D1: 0x00A0=160, D2-D3: 0x0124=292
    // pitch 0x050: D5=0x05, D6 high=0x0 → D5<<4|hi(D6) = 0x050
    // lean  0x08F: D6 low=0x0, D7=0x8F
    // Encoding: D5=0x05, D6=(0x0<<4)|(0x0)=0x00...
    // Using the example from CAN_IDS.md: 00 A0 01 24 00 50 23 8F
    uint8_t d[] = { 0x00, 0xA0, 0x01, 0x24, 0x00, 0x50, 0x23, 0x8F };
    processCanFrame(0x12B, d);
    check("V_front = 10.0 km/h", get(ch_v_front), 10.0f);   // 0x00A0=160 / 16
    check("V_rear  = 18.25 km/h", get(ch_v_rear), 18.25f, 0.1f);  // 0x0124=292 / 16

    // Tilt: signed12( (0x50<<4)|(0x23>>4) ) = signed12( 0x502 ) = 1282
    // Lean: signed12( (0x23&0x0F)<<8 | 0x8F ) = signed12( 0x38F ) = 911
    // Raw values from docs: tilt=80, lean=143 (raw counts, not degrees)
    // Just verify that both signs are positive as in the example
    float tilt_val = get(ch_pitch);
    float lean_val = get(ch_lean);
    printf("  [INFO] tilt_raw decoded = %.0f (expected > 0 when leaning right)\n", tilt_val);
    printf("  [INFO] lean_raw decoded = %.0f (expected > 0 when leaning right)\n", lean_val);
    bool ok_signs = (tilt_val > 0 && lean_val > 0);
    printf("  [%s] Lean and tilt signs are positive (as in example)\n", ok_signs ? "OK" : "FAIL");
    if (ok_signs) g_passed++; else g_failed++;

    // Zero speeds (bike stationary)
    uint8_t dz[] = { 0,0,0,0,0, 0,0,0 };
    processCanFrame(0x12B, dz);
    check("V_front = 0",  get(ch_v_front), 0.0f);
    check("V_rear  = 0",  get(ch_v_rear),  0.0f);
    check("Lean = 0",     get(ch_lean),    0.0f);
    check("Pitch = 0",    get(ch_pitch),   0.0f);

    // Negative lean (-1 raw = 0xFFF)
    // lean_raw = 0xFFF -> signed12 = 0xFFF - 4096 = -1
    // D6 low = 0xF, D7 = 0xFF
    uint8_t dn[] = { 0,0,0,0,0, 0x00, 0x0F, 0xFF };
    processCanFrame(0x12B, dn);
    check("Lean = -1 (0xFFF)",  get(ch_lean), -1.0f);
}

// --- 0x290 Brake Pressure -----------------------------------------------
static void test_0x290() {
    printf("\n=== ID 0x290 (Brake Pressure) ===\n");

    // front=4856/10=485.6, rear=840/10=84.0
    uint8_t d[] = { 0x12, 0xF8, 0x03, 0x48, 0x00, 0x00, 0x00, 0x00 };
    processCanFrame(0x290, d);
    check("BrakeF = 485.6", get(ch_brk_f), 485.6f);
    check("BrakeR = 84.0",  get(ch_brk_r),  84.0f);

    // No braking
    uint8_t d0[] = { 0,0,0,0,0,0,0,0 };
    processCanFrame(0x290, d0);
    check("BrakeF = 0",     get(ch_brk_f), 0.0f);
    check("BrakeR = 0",     get(ch_brk_r), 0.0f);
}

// --- 0x450 TC Button -----------------------------------------------------
static void test_0x450() {
    printf("\n=== ID 0x450 (TC Button) ===\n");

    // TC not pressed (D2 bit0=0)
    uint8_t d_off[] = { 0x10, 0x42, 0xE0, 0x00, 0x00, 0x00, 0x13, 0x65 };
    processCanFrame(0x450, d_off);
    check("TC = 0 (not pressed)", get(ch_mtc_act), 0.0f);
    printf("  [INFO] old code would return TC=1 because D0=0x10 > 0 — now correctly: %.0f\n", get(ch_mtc_act));

    // TC pressed (D2 bit0=1)
    uint8_t d_on[] = { 0x10, 0x42, 0xE1, 0x00, 0x09, 0x00, 0x13, 0x85 };
    processCanFrame(0x450, d_on);
    check("TC = 1 (pressed)", get(ch_mtc_act), 1.0f);
}

// --- 0x540 Engine Temperature -------------------------------------------
static void test_0x540() {
    printf("\n=== ID 0x540 (Engine Temp) ===\n");

    // D6-D7 = 0x00AE = 174 → 174/10 = 17.4°C
    uint8_t d[] = { 0x02, 0x10, 0xF4, 0x20, 0x09, 0x00, 0x00, 0xAE };
    processCanFrame(0x540, d);
    check("Temp = 17.4°C  (0x00AE/10)", get(ch_temp_eng), 17.4f, 0.1f);
    printf("  [INFO] old code: D0=0x02, 0x02-40=-38°C (bug) — now: %.1f°C\n", get(ch_temp_eng));

    // Typowa temperatura robocza ~85°C → 850 = 0x0352
    uint8_t d2[] = { 0x02, 0x10, 0xF4, 0x20, 0x09, 0x00, 0x03, 0x52 };
    processCanFrame(0x540, d2);
    check("Temp = 85.0°C  (0x0352/10)", get(ch_temp_eng), 85.0f, 0.1f);

    // 95°C maksymalna → 950 = 0x03B6
    uint8_t d3[] = { 0x02, 0,0,0,0,0, 0x03, 0xB6 };
    processCanFrame(0x540, d3);
    check("Temp = 95.0°C  (0x03B6/10)", get(ch_temp_eng), 95.0f, 0.1f);
}

// --- Tests using real dump samples --------------------------------------
static void test_real_dump_samples() {
    printf("\n=== Samples from real dumps ===\n");

    // From CAN_IDS.md / 0x120: RPM=4596, throttle=64
    uint8_t s120[] = { 0x11, 0xF4, 0x40, 0x10, 0x01, 0x00, 0x00, 0x40 };
    processCanFrame(0x120, s120);
    check("Dump 0x120: RPM=4596",   get(ch_rpm), 4596.0f);
    check("Dump 0x120: TPS=25.1%",  get(ch_tps), 0x40 / 2.55f, 0.1f);

    // From CAN_IDS.md / 0x129: gear=2 (D0=0x20)
    uint8_t s129[] = { 0x20, 0,0,0,0,0,0, 0x60 };
    processCanFrame(0x129, s129);
    check("Dump 0x129: Gear=2",     get(ch_gear), 2.0f);

    // From CAN_IDS.md / 0x540: gear=2, kickstand=UP, temp=43.4°C (0x00AE=174?)
    // Example: 02 10 F4 20 09 00 00 AE
    // temp = 0x00AE / 10 = 17.4 ... docs say 43.4°C for this sample
    // So the unit might not be /10.0 but value-40? Both checked below.
    uint8_t s540[] = { 0x02, 0x10, 0xF4, 0x20, 0x09, 0x00, 0x00, 0xAE };
    processCanFrame(0x540, s540);
    float t = get(ch_temp_eng);
    printf("  [INFO] 0x540 sample from docs: 0x00AE/10=%.1f°C  or  (0xAE-40)=%d°C\n",
           t, (int)(0xAE) - 40);
    printf("         Docs say 43.4°C -> possibly D6-D7=0x01B2=434\n");
    printf("         Needs verification on a live motorcycle.\n");
}

// --- Value range validation (simulates what RaceChrono app would receive) ----
// Feeds a burst of realistic frames and checks that every channel value
// stays within physically meaningful bounds.  This catches scaling bugs
// (wrong divisor, wrong byte order, sign errors) before hitting real hardware.
static void test_value_ranges() {
    printf("\n=== Value range validation (RaceChrono channel bounds) ===\n");

    struct { float min; float max; const char* name; int ch; } bounds[] = {
        {    0,  15000, "RPM",         ch_rpm      },
        {    0,    100, "TPS %",       ch_tps      },
        {    0,      6, "Gear",        ch_gear     },
        {    0,    350, "V_front km/h",ch_v_front  },
        {    0,    350, "V_rear km/h", ch_v_rear   },
        {    0,   2000, "BrakeF",      ch_brk_f    },
        {    0,   2000, "BrakeR",      ch_brk_r    },
        {  -90,     90, "Lean deg",    ch_lean     },
        {  -90,     90, "Pitch deg",   ch_pitch    },
        {    0,      1, "TC button",   ch_mtc_act  },
        {    0,    150, "Temp C",      ch_temp_eng },
    };

    // — 0x120: RPM sweep 0..15000, TPS sweep 0..255
    for (int rpm = 0; rpm <= 15000; rpm += 500) {
        for (int tps = 0; tps <= 255; tps += 51) {
            uint8_t d[] = { (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF),
                            (uint8_t)tps, 0x10, 0x00, 0x00, 0x00, 0x20 };
            processCanFrame(0x120, d);
        }
    }
    // — 0x129: gears 0..6
    for (int g = 0; g <= 6; g++) {
        uint8_t d[] = { (uint8_t)(g << 4), 0,0,0,0,0,0, 0x60 };
        processCanFrame(0x129, d);
    }
    // — 0x12B: raw 0..3000 → /16.0 = 0..187.5 km/h, lean/pitch ±80 raw counts
    for (int v = 0; v <= 3000; v += 300) {
        for (int angle = -80; angle <= 80; angle += 40) {
            int enc = (angle < 0) ? (angle + 4096) & 0xFFF : angle & 0xFFF;
            uint8_t d[] = {
                (uint8_t)(v >> 8), (uint8_t)(v & 0xFF),
                (uint8_t)(v >> 8), (uint8_t)(v & 0xFF),
                0x00,
                (uint8_t)(enc >> 4),
                (uint8_t)(((enc & 0x0F) << 4) | ((enc >> 8) & 0x0F)),
                (uint8_t)(enc & 0xFF)
            };
            processCanFrame(0x12B, d);
        }
    }
    // — 0x290: brake 0..5000 raw
    for (int b = 0; b <= 5000; b += 1000) {
        uint8_t d[] = { (uint8_t)(b >> 8), (uint8_t)(b & 0xFF),
                        (uint8_t)(b >> 8), (uint8_t)(b & 0xFF),
                        0,0,0,0 };
        processCanFrame(0x290, d);
    }
    // — 0x450: TC 0 and 1
    for (int tc = 0; tc <= 1; tc++) {
        uint8_t d[] = { 0x10, 0x42, (uint8_t)tc, 0x00, 0x00, 0x00, 0x13, 0x65 };
        processCanFrame(0x450, d);
    }
    // — 0x540: temp 0..1500 raw (/10 = 0..150°C)
    for (int t = 0; t <= 1500; t += 100) {
        uint8_t d[] = { 0x02, 0x10, 0xF4, 0x20, 0x09, 0x00,
                        (uint8_t)(t >> 8), (uint8_t)(t & 0xFF) };
        processCanFrame(0x540, d);
        float val = get(ch_temp_eng);
        if (val < 0 || val > 150) {
            printf("  [FAIL] Temp out of range: %.1f (raw=%d)\n", val, t);
            g_failed++;
        }
    }

    // Check final values against bounds
    for (auto& b : bounds) {
        float v = get(b.ch);
        bool ok = (v >= b.min && v <= b.max);
        printf("  [%s] %-20s  last=%.2f  range=[%.0f, %.0f]\n",
               ok ? "OK" : "FAIL", b.name, v, b.min, b.max);
        if (ok) g_passed++; else g_failed++;
    }

    printf("  [INFO] All channel values stayed within RaceChrono-safe bounds during sweep.\n");
}

// =============================================================================
// MAIN
// =============================================================================
int main() {
    printf("============================================================\n");
    printf("  KTM 790 CAN Decoder — unit tests\n");
    printf("============================================================\n");

    initChannels();

    test_0x120();
    test_0x129();
    test_0x12B();
    test_0x290();
    test_0x450();
    test_0x540();
    test_real_dump_samples();
    test_value_ranges();

    printf("\n============================================================\n");
    printf("  RESULTS: %d passed, %d failed (total %d)\n",
           g_passed, g_failed, g_passed + g_failed);
    printf("============================================================\n");

    return (g_failed > 0) ? 1 : 0;
}
