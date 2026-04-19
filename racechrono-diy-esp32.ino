#include <Arduino.h>

// =========================================================
// 1. CONFIGURACIÓN Y FRECUENCIAS KTM 790
// =========================================================
#define LOG_LEVEL LOG_LEVEL_DEBUG
#define DEVICE_NAME "KTM 790 DIY Track"
#define LOG_LEVEL LOG_LEVEL_INFO
#define RX_PIN 4  
#define TX_PIN 5  
#define SERIAL_BAUD_RATE 115200
#define TWAI_RX_QUEUE_LENGTH 128
#define DEFAULT_UPDATE_RATE_HZ 10

uint8_t getUpdateRateHz(uint32_t can_id) {
    switch (can_id) {
        case 0x120: return 20; // RPM y TPS
        case 0x129: return 5;  // Marcha
        case 0x12B: return 25; // Inclinación y Vel. Ruedas
        case 0x130: return 20; // ABS
        case 0x290: return 50; // FRENOS (Máxima prioridad)
        case 0x450: return 5;  // MTC
        case 0x540: return 1;  // Temperatura
        default: return DEFAULT_UPDATE_RATE_HZ;
    }
}

// =========================================================
// 2. LIBRERÍAS Y OBJETOS
// =========================================================
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>
#include <esp_mac.h>
#include <EasyLogger.h>
#include <RaceChrono.h> // Asegúrate que sea la de timurrrr de GitHub

// El objeto debe llamarse RaceChronoBle para evitar conflictos con el nombre de la clase
//RaceChrono RaceChronoBle; 

bool isBLEStarted = false;
twai_message_t message;
RingbufHandle_t bufferHandle;

using PidExtra = struct {
    uint32_t updateIntervalHz = 1000000 / DEFAULT_UPDATE_RATE_HZ;
    uint32_t lastMessageTime = 0;
};
RaceChronoPidMap<PidExtra> pidMap;

// =========================================================
// 3. MANEJADOR DE COMANDOS (RaceChrono Handler)
// =========================================================
class PrintRaceChronoCommands : public RaceChronoBleCanHandler {
public:
    void allowAllPids(uint16_t updateIntervalMs) {
        pidMap.allowAllPids(updateIntervalMs);
    }
    void denyAllPids() {
        pidMap.reset();
    }
    void allowPid(uint32_t pid, uint16_t updateIntervalMs) {
        if (pidMap.allowOnePid(pid, updateIntervalMs)) {
            void *entry = pidMap.getEntryId(pid);
            PidExtra *pidExtra = pidMap.getExtra(entry);
            pidExtra->lastMessageTime = esp_timer_get_time();
            pidExtra->updateIntervalHz = 1000000 / getUpdateRateHz(pid);
        }
    }
    void handleDisconnect() {
        this->denyAllPids();
    }
} raceChronoHandler;

// =========================================================
// 4. TAREAS (Definidas antes del setup para evitar errores de scope)
// =========================================================

void taskManageBLEConnection(void *) {
    for (;;) {
        if (!isBLEStarted) {
            RaceChronoBle.setUp(DEVICE_NAME, &raceChronoHandler);
            RaceChronoBle.startAdvertising();
            isBLEStarted = true;
        }
        if (!RaceChronoBle.isConnected()) {
            raceChronoHandler.handleDisconnect();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskGetTwaiMessages(void *) {
    for (;;) {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            if (!(message.rtr) && message.data_length_code > 0) {
                void *entry = pidMap.getEntryId(message.identifier);
                if (entry != NULL) {
                    PidExtra *extra = pidMap.getExtra(entry);
                    if ((esp_timer_get_time() - extra->lastMessageTime) >= extra->updateIntervalHz) {
                        xRingbufferSend(bufferHandle, &message, sizeof(message), 0);
                        extra->lastMessageTime += extra->updateIntervalHz;
                    }
                }
            }
        }
    }
}

void taskSendBLEMessages(void *) {
    size_t message_size;
    for (;;) {
        twai_message_t *msg = (twai_message_t *)xRingbufferReceive(bufferHandle, &message_size, portMAX_DELAY);
        if (msg != NULL) {
            RaceChronoBle.sendCanData(msg->identifier, msg->data, msg->data_length_code);
            vRingbufferReturnItem(bufferHandle, (void *)msg);
        }
    }
}

// =========================================================
// 5. SETUP Y LOOP
// =========================================================

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);

    // Iniciar CAN
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
    g_config.rx_queue_len = TWAI_RX_QUEUE_LENGTH;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        Serial.println("CAN OK");
    }

    bufferHandle = xRingbufferCreate(65536, RINGBUF_TYPE_NOSPLIT);
    uint mainCore = xPortGetCoreID();

    // Crear tareas
    xTaskCreatePinnedToCore(taskManageBLEConnection, "taskBLE", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskGetTwaiMessages, "taskCAN", 4096, NULL, 5, NULL, mainCore);
    xTaskCreatePinnedToCore(taskSendBLEMessages, "taskSend", 4096, NULL, 5, NULL, 0);
}

void loop() {
    vTaskDelete(NULL); 
}
