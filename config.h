#ifndef CONFIG_H
#define CONFIG_H

// Nombre del dispositivo Bluetooth (coincide con tu .ino)
#define DEVICE_NAME "KTM 790 DIY Track"

// Nivel de log por serial
#define LOG_LEVEL LOG_LEVEL_INFO

// Pines GPIO para el transceptor CAN (Ajustados a tu ESP32)
#define RX_PIN 4
#define TX_PIN 5

// Velocidad del puerto serie
#define SERIAL_BAUD_RATE 115200

// Tamaño del buffer de recepción CAN
#define TWAI_RX_QUEUE_LENGTH 128

// Frecuencia por defecto (en Hz) para canales no especificados
#define DEFAULT_UPDATE_RATE_HZ 10

// Configuración de frecuencias específicas por ID de CAN
// Esto optimiza el ancho de banda del Bluetooth
uint8_t getUpdateRateHz(uint32_t can_id)
{
    switch (can_id)
    {
    case 0x120: // RPM y TPS
        return 20;

    case 0x129: // Marcha engranada
        return 5;

    case 0x12B: // Velocidades de rueda e Inclinación (IMU)
        return 25;

    case 0x130: // Intervención de ABS
        return 20;

    case 0x290: // Presión de frenos (Necesita mucha resolución)
        return 50;

    case 0x450: // Nivel de control de tracción (MTC)
        return 2;

    case 0x540: // Temperatura del motor
        return 1;

    default:
        return DEFAULT_UPDATE_RATE_HZ;
    }
}

#endif // CONFIG_H
