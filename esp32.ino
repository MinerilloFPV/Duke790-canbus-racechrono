#include <RaceChrono.h>
#include "driver/twai.h"
#include "can_decoder.h"  // shared CAN decoder (fixed)

// --- CAN (TWAI) pins ---
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// --- RaceChrono BLE ---
RaceChrono rc("KTM 790 DIY Track");

// Channel indices — also used by can_decoder.h
int ch_rpm, ch_tps, ch_gear, ch_v_front, ch_v_rear;
int ch_brk_f, ch_brk_r, ch_lean, ch_pitch;
int ch_abs_act, ch_mtc_act, ch_temp_eng;

void setup() {
    Serial.begin(115200);

    // Register channels in RaceChrono
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

    rc.begin();
    Serial.println("Bluetooth BLE: waiting for RaceChrono...");
}

void loop() {
    twai_message_t msg;

    // Drain all available frames from the TWAI receive queue
    while (twai_receive(&msg, 0) == ESP_OK) {
        processCanFrame(msg.identifier, msg.data);
    }

    rc.run(); // keep BLE connection alive
}
