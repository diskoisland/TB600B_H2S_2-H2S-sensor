/*!
 *  @file TB600B_H2S_2.h
 *  @brief Arduino library for the ECsense TB600B-H2S-2 hydrogen sulfide sensor.
 *
 *  @author Ross Edwards
 *
 *  @license MIT License — Copyright 2026 Ross Edwards
 *
 *  @section VERSION HISTORY
 *
 *  1.0 - Dec, 2023 - Initial release.
 *  2.0 - Jun, 2026 - Improved frame parser with second-byte validation and
 *        frame timeout. Improved queryLights() with byte-by-byte read and
 *        timeout. switchToActiveUpload() and switchToPassiveUpload() now
 *        return bool. Added checkPresent(). Improved getSensorInfo() with
 *        byte-by-byte read and timeout. Added range validation for temp/RH.
 */

#ifndef TB600B_H2S_2_H
#define TB600B_H2S_2_H

#include <Arduino.h>

// Response types returned by getGasTemperature().
enum TB600B_H2S_2_ResponseType {
    TB600B_H2S_2_GAS_CONCENTRATION,  // Full valid frame with gas, temp, RH
    TB600B_H2S_2_NO_DATA,            // No complete frame available yet
    TB600B_H2S_2_INVALID_RESPONSE    // Bad frame or checksum failure
};

// Sensor metadata returned by getSensorInfo().
struct TB600B_H2S_2_SensorData {
    byte command = 0;
    int sensorType = 0;
    int maxRange = 0;
    byte units = 0;
    byte numberOfDecimals = 0;
};

class TB600B_H2S_2 {
public:
    TB600B_H2S_2(HardwareSerial &serialPort, long baudRate = 9600);

    bool begin();

    // Active-upload mode helpers (useful for manual testing).
    int readSensor();
    int readSensorWithTimeout(unsigned long timeoutMs);
    bool switchToActiveUpload();

    // Passive-upload mode.
    bool switchToPassiveUpload();
    bool passiveDataCommand();

    // Presence / status checks.
    bool queryLights();
    bool checkPresent();

    // Non-blocking passive 0x87 response parser.
    TB600B_H2S_2_ResponseType getGasTemperature(
        int *gasConcentrationPpb,
        float *temperatureC,
        float *relativeHumidity
    );

    // Sensor metadata query.
    bool getSensorInfo(TB600B_H2S_2_SensorData *sensorData);

private:
    HardwareSerial *_serialPort;
    long _baudRate;

    bool validateChecksum(const byte *data, int length);
    int calculateConcentration(byte high, byte low);
    bool trySendCommand(const byte *command, size_t size);

    byte _buffer[13];
    size_t _bufferIndex = 0;
    unsigned long _frameStartTime = 0;
};

#endif  // TB600B_H2S_2_H
