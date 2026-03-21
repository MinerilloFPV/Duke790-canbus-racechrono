#include <RaceChrono.h>
#include "driver/twai.h"

// --- CAN (TWAI) pins ---
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// --- Debug / logging ---
static constexpr uint32_t HEARTBEAT_MS = 1000;
static constexpr uint32_t NO_CAN_WARN_MS = 3000;

// ---------------------------------------------------------------------------
// RaceChrono BLE handler.
//
// The arduino-RaceChrono library works at the raw CAN-frame level:
//   - The ESP32 forwards every incoming frame to the RaceChrono app via BLE.
//   - The app decodes frames (configure channels under Settings → Connections
//     → your device → "CAN-Bus channels").
//
// For this simple use-case we allow every PID so no filtering is needed.
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
        "[APP] heartbeat: ble=%s total_frames=%lu frames_last_1s=%lu last_can_age=%lums\n",
        RaceChronoBle.isConnected() ? "CONNECTED" : "WAITING",
        (unsigned long)g_totalFrames,
        (unsigned long)g_framesSinceLastHeartbeat,
        g_lastCanFrameMs == 0 ? 0UL : (unsigned long)(now - g_lastCanFrameMs)
    );
    g_framesSinceLastHeartbeat = 0;

    logTwaiStatus("heartbeat");

    if (g_lastCanFrameMs == 0 || (now - g_lastCanFrameMs) >= NO_CAN_WARN_MS) {
        if (now - g_lastNoCanWarnMs >= NO_CAN_WARN_MS) {
            g_lastNoCanWarnMs = now;
            Serial.println("[CAN] WARNING: no CAN frames seen recently. Check CAN_H/CAN_L wiring, ground, bitrate (500 kbps), and whether the bike is awake/ignition ON.");
        }
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
    Serial.println("[APP] Booting KTM 790 DIY Track bridge...");
    Serial.printf("[APP] CAN pins: TX=%d RX=%d\n", CAN_TX_PIN, CAN_RX_PIN);

    // Init BLE
    RaceChronoBle.setUp("KTM 790 DIY Track", &handler);
    RaceChronoBle.startAdvertising();
    Serial.println("[BLE] Advertising started: KTM 790 DIY Track");

    // Init TWAI (CAN) at 500 kbps, listen-only mode
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK
        && twai_start() == ESP_OK) {
        Serial.println("CAN Bus: OK - 500 kbps, listen-only");
        logTwaiStatus("startup");
    } else {
        Serial.println("CAN Bus: init FAILED!");
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

    logHeartbeat();
}
