#ifndef CONFIG_H
#define CONFIG_H

// Bluetooth device name
// If not defined, will default to "RaceChrono aa:bb:cc"
// aa:bb:cc being the last three octets of the MAC address
#define DEVICE_NAME "Duke 790 DIY"

// Serial logging level, select one from the following list :
// LOG_LEVEL_EMERGENCY
// LOG_LEVEL_ALERT
// LOG_LEVEL_CRITICAL
// LOG_LEVEL_ERROR
// LOG_LEVEL_WARNING
// LOG_LEVEL_NOTICE
// LOG_LEVEL_INFO
// LOG_LEVEL_DEBUG
#define LOG_LEVEL LOG_LEVEL_INFO

// GPIO pins connected to the CAN transceiver
#define RX_PIN 11
#define TX_PIN 12

// Serial baud rate
#define SERIAL_BAUD_RATE 115200

// TWAI RX buffer size
#define TWAI_RX_QUEUE_LENGTH 128

// Bluetooth 5.0 support, 1 to enable, 0 to disable
#define CONFIG_BT_NIMBLE_EXT_ADV 1

// Limit the rate of messages sent to RaceChrono to x CAN messages each second
// Can be further refined per PID through the list below
#define DEFAULT_UPDATE_RATE_HZ 10

// An optional list of PIDs and their associated rate limit.
// Limits the update rate of the specified PID to the returned value.
uint8_t getUpdateRateHz(uint32_t can_id)
{
    switch (can_id)
    {
    // An example of how to configure this list is detailed in the README.
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
