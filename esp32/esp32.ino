#include <RaceChrono.h>
#include "driver/twai.h"

// --- CAN (TWAI) pins ---
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

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

static void waitForConnection() {
    Serial.print("Bluetooth BLE: waiting for RaceChrono");
    while (!RaceChronoBle.waitForConnection(1000)) {
        Serial.print(".");
    }
    Serial.println(" connected.");
}

void setup() {
    Serial.begin(115200);

    // Init BLE
    RaceChronoBle.setUp("KTM 790 DIY Track", &handler);
    RaceChronoBle.startAdvertising();

    // Init TWAI (CAN) at 500 kbps, listen-only mode
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK
        && twai_start() == ESP_OK) {
        Serial.println("CAN Bus: OK - 500 kbps, listen-only");
    } else {
        Serial.println("CAN Bus: init FAILED!");
    }

    waitForConnection();
}

void loop() {
    if (!RaceChronoBle.isConnected()) {
        Serial.println("RaceChrono disconnected, waiting...");
        waitForConnection();
    }

    twai_message_t msg;

    // Forward every incoming CAN frame to RaceChrono over BLE.
    while (twai_receive(&msg, 0) == ESP_OK) {
        RaceChronoBle.sendCanData(msg.identifier, msg.data,
                                  msg.data_length_code);
    }
}
