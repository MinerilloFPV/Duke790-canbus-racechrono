#include <RaceChrono.h>
#include "driver/twai.h"

// --- CAN (TWAI) pins ---
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// --- Debug / logging ---
static constexpr uint32_t HEARTBEAT_MS = 1000;
static constexpr uint32_t NO_CAN_WARN_MS = 3000;

// --- Bitrate scan configuration ---
static constexpr bool SCAN_MODE = true;
static constexpr uint32_t SCAN_PROBE_MS = 3000;  // time per bitrate attempt
static constexpr uint32_t BUS_WAIT_TIMEOUT_MS = 30000; // max wait for bus activity
static constexpr uint32_t RESCAN_BUS_ERR_THRESHOLD = 5000; // bus_err count to trigger rescan

struct BitrateEntry {
    const char* label;
    twai_timing_config_t timing;
};

// Common CAN bitrates to try (most likely first for KTM)
static BitrateEntry BITRATES[] = {
    { "500 kbps", TWAI_TIMING_CONFIG_500KBITS()  },
    { "250 kbps", TWAI_TIMING_CONFIG_250KBITS()  },
    { "1 Mbps",   TWAI_TIMING_CONFIG_1MBITS()    },
    { "800 kbps", TWAI_TIMING_CONFIG_800KBITS()  },
    { "125 kbps", TWAI_TIMING_CONFIG_125KBITS()  },
    { "100 kbps", TWAI_TIMING_CONFIG_100KBITS()  },
    { "50 kbps",  TWAI_TIMING_CONFIG_50KBITS()   },
    { "25 kbps",  TWAI_TIMING_CONFIG_25KBITS()   },
};
static constexpr size_t NUM_BITRATES = sizeof(BITRATES) / sizeof(BITRATES[0]);

static const char* g_activeBitrateLabel = "unknown";
static bool g_scanLocked = false; // true once a valid bitrate is found
static uint32_t g_lastBusErrSnapshot = 0;
static uint32_t g_busErrCheckMs = 0;

// ---------------------------------------------------------------------------
// RaceChrono BLE handler.
// ---------------------------------------------------------------------------
class KtmCanHandler : public RaceChronoBleCanHandler {
public:
    void allowAllPids(uint16_t /*updateIntervalMs*/) override {}
    void denyAllPids() override {}
    void allowPid(uint32_t /*pid*/, uint16_t /*updateIntervalMs*/) override {}
} handler;

static uint32_t g_totalFrames = 0;
static uint32_t g_framesSinceLastHeartbeat = 0;
static uint32_t g_lastHeartbeatMs = 0;
static uint32_t g_lastCanFrameMs = 0;
static uint32_t g_lastNoCanWarnMs = 0;
static bool g_prevBleConnected = false;

static const char* twaiStateToString(twai_state_t state) {
    switch (state) {
        case TWAI_STATE_STOPPED: return "STOPPED";
        case TWAI_STATE_RUNNING: return "RUNNING";
        case TWAI_STATE_BUS_OFF: return "BUS_OFF";
        case TWAI_STATE_RECOVERING: return "RECOVERING";
        default: return "UNKNOWN";
    }
}

static void logTwaiStatus(const char* prefix) {
    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        Serial.printf("[CAN] %s: twai_get_status_info() failed\n", prefix);
        return;
    }

    Serial.printf(
        "[CAN] %s: state=%s rx_queue=%lu tx_queue=%lu rx_missed=%lu rx_overrun=%lu "
        "rx_err=%lu tx_err=%lu bus_err=%lu arb_lost=%lu\n",
        prefix,
        twaiStateToString(status.state),
        (unsigned long)status.msgs_to_rx,
        (unsigned long)status.msgs_to_tx,
        (unsigned long)status.rx_missed_count,
        (unsigned long)status.rx_overrun_count,
        (unsigned long)status.rx_error_counter,
        (unsigned long)status.tx_error_counter,
        (unsigned long)status.bus_error_count,
        (unsigned long)status.arb_lost_count
    );
}

static void logCanFrame(const twai_message_t& msg) {
    Serial.printf(
        "[CAN] RX id=0x%03lX ext=%u rtr=%u dlc=%u data=",
        (unsigned long)msg.identifier,
        msg.extd ? 1 : 0,
        msg.rtr ? 1 : 0,
        msg.data_length_code
    );

    for (uint8_t i = 0; i < msg.data_length_code; ++i) {
        Serial.printf("%02X", msg.data[i]);
        if (i + 1 < msg.data_length_code) Serial.print(" ");
    }
    Serial.println();
}

static void logHeartbeat() {
    const uint32_t now = millis();
    if (now - g_lastHeartbeatMs < HEARTBEAT_MS) {
        return;
    }
    g_lastHeartbeatMs = now;

    Serial.printf(
        "[APP] heartbeat: ble=%s bitrate=%s total_frames=%lu frames_last_1s=%lu last_can_age=%lums\n",
        RaceChronoBle.isConnected() ? "CONNECTED" : "WAITING",
        g_activeBitrateLabel,
        (unsigned long)g_totalFrames,
        (unsigned long)g_framesSinceLastHeartbeat,
        g_lastCanFrameMs == 0 ? 0UL : (unsigned long)(now - g_lastCanFrameMs)
    );
    g_framesSinceLastHeartbeat = 0;

    logTwaiStatus("heartbeat");

    if (g_lastCanFrameMs == 0 || (now - g_lastCanFrameMs) >= NO_CAN_WARN_MS) {
        if (now - g_lastNoCanWarnMs >= NO_CAN_WARN_MS) {
            g_lastNoCanWarnMs = now;
            Serial.println("[CAN] WARNING: no CAN frames seen recently.");
        }
    }
}

// ---------------------------------------------------------------------------
// Wait for bus activity: start at 500 kbps and wait until bus_err > 0 or
// a frame is received. This ensures the scan only runs when there is
// actual CAN traffic (bike ignition ON).
// ---------------------------------------------------------------------------
static void waitForBusActivity() {
    Serial.println("\n[SCAN] Waiting for CAN bus activity (turn ignition ON)...");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    uint32_t startMs = millis();
    uint32_t dotMs = millis();

    while (millis() - startMs < BUS_WAIT_TIMEOUT_MS) {
        // Check for a received frame (would mean 500 kbps is correct)
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(200)) == ESP_OK) {
            Serial.println("\n[SCAN] Got a frame at 500 kbps during wait — bus is active!");
            // 500 kbps works right away
            twai_stop();
            twai_driver_uninstall();
            return;
        }

        // Check bus errors — if they grow, bus is active (wrong bitrate = errors)
        twai_status_info_t status{};
        twai_get_status_info(&status);
        if (status.bus_error_count > 100) {
            Serial.printf("\n[SCAN] Bus activity detected! (bus_err=%lu) — starting scan.\n",
                (unsigned long)status.bus_error_count);
            twai_stop();
            twai_driver_uninstall();
            return;
        }

        if (millis() - dotMs >= 1000) {
            dotMs = millis();
            Serial.printf("[SCAN] Waiting... (%lus, bus_err=%lu)\n",
                (unsigned long)((millis() - startMs) / 1000),
                (unsigned long)status.bus_error_count);
        }
    }

    Serial.println("\n[SCAN] Timeout waiting for bus activity. Proceeding with scan anyway...");
    twai_stop();
    twai_driver_uninstall();
}

// ---------------------------------------------------------------------------
// Bitrate scanner
// ---------------------------------------------------------------------------
static bool tryBitrate(size_t idx) {
    BitrateEntry& entry = BITRATES[idx];
    Serial.printf("\n[SCAN] === Trying %s (%lu ms) ===\n", entry.label, (unsigned long)SCAN_PROBE_MS);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &entry.timing, &f_config) != ESP_OK) {
        Serial.printf("[SCAN] %s: driver install FAILED\n", entry.label);
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.printf("[SCAN] %s: start FAILED\n", entry.label);
        twai_driver_uninstall();
        return false;
    }

    uint32_t frames = 0;
    uint32_t startMs = millis();

    while (millis() - startMs < SCAN_PROBE_MS) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            ++frames;
            logCanFrame(msg);
            Serial.printf("[SCAN] %s: ^^^ GOT FRAME #%lu!\n",
                entry.label, (unsigned long)frames);
        }
    }

    // Check bus errors
    twai_status_info_t status{};
    twai_get_status_info(&status);
    Serial.printf("[SCAN] %s result: frames=%lu bus_err=%lu rx_err=%lu state=%s\n",
        entry.label,
        (unsigned long)frames,
        (unsigned long)status.bus_error_count,
        (unsigned long)status.rx_error_counter,
        twaiStateToString(status.state));

    if (frames > 0) {
        Serial.printf("\n[SCAN] *** SUCCESS: %s works! Received %lu frames ***\n\n",
            entry.label, (unsigned long)frames);
        g_activeBitrateLabel = entry.label;
        g_scanLocked = true;
        return true;
    }

    // No frames — also report if bus was active (errors = signal present but wrong rate)
    if (status.bus_error_count > 0) {
        Serial.printf("[SCAN] %s: bus active but WRONG bitrate (bus_err=%lu)\n",
            entry.label, (unsigned long)status.bus_error_count);
    } else {
        Serial.printf("[SCAN] %s: bus SILENT (no errors, no frames)\n", entry.label);
    }

    twai_stop();
    twai_driver_uninstall();
    return false;
}

static bool scanBitrates() {
    Serial.println("\n[SCAN] ========================================");
    Serial.println("[SCAN] Starting CAN bitrate auto-detection...");
    Serial.printf("[SCAN] Will try %u bitrates, %lu ms each\n",
        (unsigned)NUM_BITRATES, (unsigned long)SCAN_PROBE_MS);
    Serial.println("[SCAN] ========================================\n");

    for (size_t i = 0; i < NUM_BITRATES; ++i) {
        if (tryBitrate(i)) {
            return true;
        }
    }

    Serial.println("\n[SCAN] ========================================");
    Serial.println("[SCAN] FAILED: No valid bitrate found!");
    Serial.println("[SCAN] Possible hardware issues:");
    Serial.println("[SCAN]   1. CAN_H / CAN_L swapped on transceiver");
    Serial.println("[SCAN]   2. TX/RX pins swapped (GPIO 4 <-> 5)");
    Serial.println("[SCAN]   3. Missing GND between ESP32 and bike");
    Serial.println("[SCAN]   4. SN65HVD230 defective or wrong wiring");
    Serial.println("[SCAN] ========================================\n");

    // Fall back to 500 kbps
    Serial.println("[SCAN] Falling back to 500 kbps...");
    g_activeBitrateLabel = "500 kbps (fallback)";
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
    return false;
}

// ---------------------------------------------------------------------------
// Auto-rescan: called from loop() when bus_err grows but no frames received.
// This handles the case where ignition is turned ON after initial scan.
// ---------------------------------------------------------------------------
static void checkAndRescan() {
    if (g_scanLocked || !SCAN_MODE) return;
    if (g_totalFrames > 0) {
        g_scanLocked = true; // frames are coming, no need to rescan
        return;
    }

    uint32_t now = millis();
    if (now - g_busErrCheckMs < 3000) return; // check every 3s
    g_busErrCheckMs = now;

    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) return;

    uint32_t errDelta = status.bus_error_count - g_lastBusErrSnapshot;
    g_lastBusErrSnapshot = status.bus_error_count;

    if (errDelta > RESCAN_BUS_ERR_THRESHOLD) {
        Serial.printf("\n[SCAN] Bus errors growing rapidly (%lu new errors) but 0 frames — re-scanning!\n",
            (unsigned long)errDelta);
        // Stop current driver
        twai_stop();
        twai_driver_uninstall();
        // Run full scan
        scanBitrates();
    }
}

static void waitForConnection() {
    Serial.print("Bluetooth BLE: waiting for RaceChrono");
    while (!RaceChronoBle.waitForConnection(1000)) {
        Serial.print(".");
        logHeartbeat();
    }
    Serial.println(" connected.");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("[APP] =============================================");
    Serial.println("[APP] Booting KTM 790 DIY Track CAN-BLE bridge");
    Serial.printf("[APP] CAN pins: TX=GPIO%d  RX=GPIO%d\n", CAN_TX_PIN, CAN_RX_PIN);
    Serial.println("[APP] Transceiver: SN65HVD230");
    Serial.println("[APP] =============================================");

    // Init BLE
    RaceChronoBle.setUp("KTM 790 DIY Track", &handler);
    RaceChronoBle.startAdvertising();
    Serial.println("[BLE] Advertising started: KTM 790 DIY Track");

    // Init TWAI (CAN)
    if (SCAN_MODE) {
        waitForBusActivity();
        scanBitrates();
        g_busErrCheckMs = millis();
        // snapshot current bus_err for rescan detection
        twai_status_info_t st{};
        if (twai_get_status_info(&st) == ESP_OK) {
            g_lastBusErrSnapshot = st.bus_error_count;
        }
    } else {
        g_activeBitrateLabel = "500 kbps";
        g_scanLocked = true;
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
        twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK
            && twai_start() == ESP_OK) {
            Serial.printf("[CAN] OK - %s, listen-only\n", g_activeBitrateLabel);
            logTwaiStatus("startup");
        } else {
            Serial.println("[CAN] init FAILED!");
        }
    }

    waitForConnection();
    g_prevBleConnected = RaceChronoBle.isConnected();
}

void loop() {
    const bool bleConnected = RaceChronoBle.isConnected();
    if (bleConnected != g_prevBleConnected) {
        Serial.printf("[BLE] state changed: %s\n", bleConnected ? "CONNECTED" : "DISCONNECTED");
        g_prevBleConnected = bleConnected;
    }

    if (!bleConnected) {
        Serial.println("RaceChrono disconnected, waiting...");
        waitForConnection();
        g_prevBleConnected = RaceChronoBle.isConnected();
    }

    twai_message_t msg;

    // Forward every incoming CAN frame to RaceChrono over BLE.
    while (twai_receive(&msg, 0) == ESP_OK) {
        ++g_totalFrames;
        ++g_framesSinceLastHeartbeat;
        g_lastCanFrameMs = millis();

        logCanFrame(msg);

        if (RaceChronoBle.isConnected()) {
            RaceChronoBle.sendCanData(msg.identifier, msg.data,
                                      msg.data_length_code);
        }
    }

    // Auto-rescan if bus errors growing but no frames received
    checkAndRescan();

    logHeartbeat();
}
