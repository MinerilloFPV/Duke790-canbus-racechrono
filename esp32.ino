#include <RaceChrono.h>
#include "driver/twai.h"

// --- CONFIGURACIÓN DE PINES ---
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// --- INICIALIZACIÓN DE RACECHRONO ---
RaceChrono rc("KTM 790 DIY Track");

// IDs de canales para RaceChrono
int ch_rpm, ch_tps, ch_gear, ch_v_front, ch_v_rear;
int ch_brk_f, ch_brk_r, ch_lean, ch_pitch;
int ch_abs_act, ch_mtc_act, ch_temp_eng;

void setup() {
    Serial.begin(115200);

    // 1. DEFINICIÓN DE CANALES EN LA APP
    ch_rpm      = rc.addCanChannel("RPM", "rpm", 20);
    ch_tps      = rc.addCanChannel("Throttle Position", "%", 20);
    ch_gear     = rc.addCanChannel("Gear", "", 5);
    ch_v_front  = rc.addCanChannel("Wheel Speed Front", "km/h", 10);
    ch_v_rear   = rc.addCanChannel("Wheel Speed Rear", "km/h", 10);
    ch_brk_f    = rc.addCanChannel("Brake Pressure Front", "bar", 50);
    ch_brk_r    = rc.addCanChannel("Brake Pressure Rear", "bar", 20);
    ch_lean     = rc.addCanChannel("Lean Angle", "deg", 25);
    ch_pitch    = rc.addCanChannel("Pitch (Wheelie)", "deg", 25);
    ch_abs_act  = rc.addCanChannel("ABS Intervention", "", 50);
    ch_mtc_act  = rc.addCanChannel("MTC Intervention", "", 50);
    ch_temp_eng = rc.addCanChannel("Engine Temp", "C", 1);

    // 2. CONFIGURACIÓN DRIVER CAN (TWAI) - 500 kbps
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        Serial.println("CAN Bus: Sistema iniciado correctamente");
    } else {
        Serial.println("CAN Bus: Error de inicio");
    }

    rc.begin(); // Iniciar Bluetooth BLE
    Serial.println("Bluetooth: Esperando conexión de RaceChrono...");
}

void loop() {
    twai_message_t msg;

    // Procesar mensajes del bus CAN
    while (twai_receive(&msg, 0) == ESP_OK) {
        
        switch (msg.identifier) {
            
            case 0x120: // RPM y Modo
                rc.updateCanChannel(ch_rpm, (msg.data[0] << 8) | msg.data[1]);
                break;

            case 0x129: // Marcha
                rc.updateCanChannel(ch_gear, msg.data[0]); 
                break;

            case 0x12A: // Acelerador (TPS)
                rc.updateCanChannel(ch_tps, msg.data[0] / 2.55);
                break;

            case 0x12B: // Ruedas e IMU (Lean/Pitch)
                rc.updateCanChannel(ch_v_rear, ((msg.data[0] << 8) | msg.data[1]) / 10.0);
                rc.updateCanChannel(ch_v_front, ((msg.data[2] << 8) | msg.data[3]) / 10.0);
                rc.updateCanChannel(ch_lean, (int8_t)(msg.data[4] - 128));
                rc.updateCanChannel(ch_pitch, (int8_t)(msg.data[5] - 128));
                break;

            case 0x290: // Frenos y ABS
                rc.updateCanChannel(ch_brk_f, ((msg.data[0] << 8) | msg.data[1]) / 10.0);
                rc.updateCanChannel(ch_brk_r, ((msg.data[2] << 8) | msg.data[3]) / 10.0);
                // Si el byte 4 cambia al frenar fuerte, es el flag de ABS
                rc.updateCanChannel(ch_abs_act, (msg.data[4] > 0 ? 1 : 0));
                break;

            case 0x450: // Control de Tracción (MTC)
                rc.updateCanChannel(ch_mtc_act, (msg.data[0] > 0 ? 1 : 0));
                break;

            case 0x540: // Temperaturas
                rc.updateCanChannel(ch_temp_eng, msg.data[0] - 40);
                break;
        }
    }

    rc.run(); // Mantener conexión Bluetooth
}
