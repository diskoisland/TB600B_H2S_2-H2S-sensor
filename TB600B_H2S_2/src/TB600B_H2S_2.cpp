#include "TB600B_H2S_2.h"

TB600B_H2S_2::TB600B_H2S_2(HardwareSerial& serialPort, long baudRate) {
    _serialPort = &serialPort;
    _baudRate = baudRate;
}

bool TB600B_H2S_2::begin() {
    _serialPort->begin(_baudRate);
    delay(100);
    clearSerialInput();
    return true;
}

void TB600B_H2S_2::clearSerialInput() {
    while (_serialPort->available()) {
        _serialPort->read();
    }
}

bool TB600B_H2S_2::trySendCommand(const byte* command, size_t size) {
    const int maxAttempts = 5;

    for (int attempt = 0; attempt < maxAttempts; attempt++) {
        if (_serialPort->availableForWrite() >= static_cast<int>(size)) {
            _serialPort->write(command, size);
            _serialPort->flush();
            return true;
        }

        delay(10);
    }

    return false;
}

bool TB600B_H2S_2::validateChecksum(const byte* data, int length) {
    byte checksum = 0;

    for (int i = 1; i < length - 1; i++) {
        checksum += data[i];
    }

    checksum = ~checksum + 1;
    return checksum == data[length - 1];
}

int TB600B_H2S_2::calculateConcentration(byte high, byte low) {
    return (high << 8) | low;
}

bool TB600B_H2S_2::checkPresent(uint8_t attempts, unsigned long delayBetweenAttemptsMs) {
    for (uint8_t attempt = 0; attempt < attempts; attempt++) {
        clearSerialInput();

        if (queryLights()) {
            return true;
        }

        delay(delayBetweenAttemptsMs);
    }

    return false;
}

bool TB600B_H2S_2::queryLights() {
    byte command[] = {
        0xFF, 0x01, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75
    };

    const int maxAttempts = 5;
    const int delayBetweenAttemptsMs = 200;

    for (int attempt = 0; attempt < maxAttempts; attempt++) {
        clearSerialInput();

        if (!trySendCommand(command, sizeof(command))) {
            delay(delayBetweenAttemptsMs);
            continue;
        }

        delay(50);

        if (_serialPort->available() >= 9) {
            byte data[9];

            for (int i = 0; i < 9; i++) {
                data[i] = _serialPort->read();
            }

            if (data[0] == 0xFF && data[1] == 0x8A && validateChecksum(data, 9)) {
                return true;
            }
        }

        delay(delayBetweenAttemptsMs);
    }

    return false;
}

void TB600B_H2S_2::switchToActiveUpload() {
    byte command[] = {
        0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47
    };

    trySendCommand(command, sizeof(command));
    delay(100);
}

void TB600B_H2S_2::switchToPassiveUpload() {
    byte command[] = {
        0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46
    };

    trySendCommand(command, sizeof(command));
    delay(100);
    clearSerialInput();
}

void TB600B_H2S_2::passiveDataCommand() {
    byte command[] = {
        0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78
    };

    trySendCommand(command, sizeof(command));
}

TB600B_H2S_2_ResponseType TB600B_H2S_2::getGasTemperature(
    int* gasConcentrationPpb,
    float* temperatureC,
    float* relativeHumidity
) {
    while (_serialPort->available()) {
        byte c = _serialPort->read();

        if (_bufferIndex == 0 && c != 0xFF) {
            continue;
        }

        if (_bufferIndex < RESPONSE_BUFFER_SIZE) {
            _buffer[_bufferIndex++] = c;
        } else {
            _bufferIndex = 0;
            return TB600B_H2S_2_INVALID_RESPONSE;
        }

        if (_bufferIndex == RESPONSE_BUFFER_SIZE) {
            _bufferIndex = 0;

            if (!validateChecksum(_buffer, RESPONSE_BUFFER_SIZE)) {
                return TB600B_H2S_2_INVALID_RESPONSE;
            }

            if (_buffer[1] != 0x87) {
                return TB600B_H2S_2_INVALID_RESPONSE;
            }

            if (gasConcentrationPpb != nullptr) {
                *gasConcentrationPpb = calculateConcentration(_buffer[6], _buffer[7]);
            }

            if (temperatureC != nullptr) {
                int16_t rawTemp = (int16_t)((_buffer[8] << 8) | _buffer[9]);
                *temperatureC = rawTemp / 100.0f;
            }

            if (relativeHumidity != nullptr) {
                uint16_t rawRh = (uint16_t)((_buffer[10] << 8) | _buffer[11]);
                *relativeHumidity = rawRh / 100.0f;
            }

            return TB600B_H2S_2_GAS_CONCENTRATION;
        }
    }

    return TB600B_H2S_2_NO_DATA;
}

bool TB600B_H2S_2::getSensorInfo(TB600B_H2S_2_SensorData* sensorData) {
    if (sensorData == nullptr) {
        return false;
    }

    // Sensor information command.
    // This sensor responds to the one-byte 0xD7 command rather than a 9-byte framed command.
    const byte command[] = {0xD7};

    const int responseLength = 9;
    const unsigned long timeoutMs = 1000;

    clearSerialInput();

    if (!trySendCommand(command, sizeof(command))) {
        return false;
    }

    unsigned long startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (_serialPort->available() >= responseLength) {
            byte data[responseLength];

            for (int i = 0; i < responseLength; i++) {
                data[i] = _serialPort->read();
            }

            if (data[0] == 0xFF && validateChecksum(data, responseLength) && data[1] == 0xD7) {
                sensorData->command = data[1];
                sensorData->sensorType = data[2];
                sensorData->maxRange = (data[3] << 8) | data[4];
                sensorData->units = data[5];
                sensorData->numberOfDecimals = data[6] >> 4;
                return true;
            }
        }

        delay(10);
    }

    return false;
}
