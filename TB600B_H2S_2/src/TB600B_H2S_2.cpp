#include "TB600B_H2S_2.h"

TB600B_H2S_2::TB600B_H2S_2(HardwareSerial &serialPort, long baudRate) {
    _serialPort = &serialPort;
    _baudRate = baudRate;
}

bool TB600B_H2S_2::begin() {
    _serialPort->begin(_baudRate);
    _serialPort->setTimeout(100);
    delay(100);

    while (_serialPort->available()) {
        _serialPort->read();
    }

    return _serialPort->availableForWrite() > 0;
}

bool TB600B_H2S_2::trySendCommand(const byte *command, size_t size) {
    const int maxAttempts = 5;

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        if (_serialPort->availableForWrite() >= static_cast<int>(size)) {
            _serialPort->write(command, size);
            _serialPort->flush();
            return true;
        }

        delay(10);
    }

    return false;
}

bool TB600B_H2S_2::validateChecksum(const byte *data, int length) {
    byte checksum = 0;

    for (int i = 1; i < length - 1; i++) {
        checksum += data[i];
    }

    checksum = ~checksum + 1;
    return checksum == data[length - 1];
}

int TB600B_H2S_2::calculateConcentration(byte high, byte low) {
    return ((int)high << 8) | low;
}

bool TB600B_H2S_2::queryLights() {
    const byte command[] = {
        0xFF, 0x01, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75
    };

    const int maxAttempts = 2;
    const unsigned long timeoutMs = 500;

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        while (_serialPort->available()) {
            _serialPort->read();
        }

        if (!trySendCommand(command, sizeof(command))) {
            delay(100);
            continue;
        }

        byte data[9];
        int index = 0;
        unsigned long startTime = millis();

        while (millis() - startTime < timeoutMs) {
            if (!_serialPort->available()) {
                delay(5);
                continue;
            }

            byte c = _serialPort->read();

            if (index == 0 && c != 0xFF) {
                continue;
            }

            data[index++] = c;

            if (index == 9) {
                if (data[0] == 0xFF &&
                    data[1] == 0x8A &&
                    validateChecksum(data, 9)) {
                    return true;
                }

                break;
            }
        }

        delay(100);
    }

    return false;
}

bool TB600B_H2S_2::checkPresent() {
    for (int attempt = 0; attempt < 2; attempt++) {
        while (_serialPort->available()) {
            _serialPort->read();
        }

        if (queryLights()) {
            return true;
        }

        while (_serialPort->available()) {
            _serialPort->read();
        }

        delay(250);
    }

    return false;
}

int TB600B_H2S_2::readSensor() {
    while (_serialPort->available() && _serialPort->peek() != 0xFF) {
        _serialPort->read();
    }

    if (_serialPort->available() < 9) {
        return -1;
    }

    byte data[9];

    for (int i = 0; i < 9; i++) {
        data[i] = _serialPort->read();
    }

    if (data[0] != 0xFF || data[1] != 0x86 || !validateChecksum(data, 9)) {
        return -1;
    }

    return calculateConcentration(data[6], data[7]);
}

int TB600B_H2S_2::readSensorWithTimeout(unsigned long timeoutMs) {
    unsigned long startTime = millis();

    while (millis() - startTime < timeoutMs) {
        int result = readSensor();

        if (result != -1) {
            return result;
        }

        delay(5);
    }

    return -1;
}

bool TB600B_H2S_2::switchToActiveUpload() {
    const byte command[] = {
        0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47
    };

    const int maxAttempts = 5;
    const int delayBetweenAttempts = 1000;

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        while (_serialPort->available()) {
            _serialPort->read();
        }

        if (!trySendCommand(command, sizeof(command))) {
            delay(delayBetweenAttempts);
            continue;
        }

        if (readSensorWithTimeout(1000) != -1) {
            return true;
        }

        delay(delayBetweenAttempts);
    }

    return false;
}

bool TB600B_H2S_2::switchToPassiveUpload() {
    const byte command[] = {
        0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46
    };

    const int maxAttempts = 2;
    const int delayBetweenAttempts = 500;

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        while (_serialPort->available()) {
            _serialPort->read();
        }

        if (!trySendCommand(command, sizeof(command))) {
            delay(delayBetweenAttempts);
            continue;
        }

        delay(200);

        if (queryLights()) {
            return true;
        }

        delay(delayBetweenAttempts);
    }

    return false;
}

bool TB600B_H2S_2::passiveDataCommand() {
    const byte command[] = {
        0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78
    };

    return trySendCommand(command, sizeof(command));
}

TB600B_H2S_2_ResponseType TB600B_H2S_2::getGasTemperature(
    int *gasConcentrationPpb,
    float *temperatureC,
    float *relativeHumidity
) {
    static unsigned long frameStartTime = 0;
    const unsigned long frameTimeoutMs = 250;
    const size_t frameLength = 13;

    // Drop a partial frame that has been sitting too long.
    if (_bufferIndex > 0 && millis() - frameStartTime > frameTimeoutMs) {
        _bufferIndex = 0;
    }

    while (_serialPort->available()) {
        byte c = _serialPort->read();

        // Wait for start byte.
        if (_bufferIndex == 0) {
            if (c != 0xFF) {
                continue;
            }

            frameStartTime = millis();
            _buffer[_bufferIndex++] = c;
            continue;
        }

        // Second byte must be 0x87 (passive data response).
        // If it is another 0xFF treat it as a new start byte.
        if (_bufferIndex == 1) {
            if (c != 0x87) {
                if (c == 0xFF) {
                    frameStartTime = millis();
                    _buffer[0] = 0xFF;
                    _bufferIndex = 1;
                } else {
                    _bufferIndex = 0;
                }

                continue;
            }
        }

        if (_bufferIndex < frameLength) {
            _buffer[_bufferIndex++] = c;
        } else {
            _bufferIndex = 0;
            return TB600B_H2S_2_INVALID_RESPONSE;
        }

        if (_bufferIndex == frameLength) {
            _bufferIndex = 0;

            if (_buffer[0] != 0xFF || _buffer[1] != 0x87) {
                return TB600B_H2S_2_INVALID_RESPONSE;
            }

            if (!validateChecksum(_buffer, frameLength)) {
                return TB600B_H2S_2_INVALID_RESPONSE;
            }

            int gas = calculateConcentration(_buffer[6], _buffer[7]);

            float parsedTemp =
                (float)((int16_t)((_buffer[8] << 8) | _buffer[9])) / 100.0f;

            float parsedRh =
                (float)((uint16_t)((_buffer[10] << 8) | _buffer[11])) / 100.0f;

            if (parsedTemp < -40.0f || parsedTemp > 85.0f) {
                return TB600B_H2S_2_INVALID_RESPONSE;
            }

            if (parsedRh < 0.0f || parsedRh > 100.0f) {
                return TB600B_H2S_2_INVALID_RESPONSE;
            }

            if (gasConcentrationPpb != nullptr) {
                *gasConcentrationPpb = gas;
            }

            if (temperatureC != nullptr) {
                *temperatureC = parsedTemp;
            }

            if (relativeHumidity != nullptr) {
                *relativeHumidity = parsedRh;
            }

            return TB600B_H2S_2_GAS_CONCENTRATION;
        }
    }

    return TB600B_H2S_2_NO_DATA;
}

bool TB600B_H2S_2::getSensorInfo(TB600B_H2S_2_SensorData *sensorData) {
    if (sensorData == nullptr) {
        return false;
    }

    const byte command[] = { 0xD7 };
    const int length = 9;
    const unsigned long timeoutMs = 1000;

    while (_serialPort->available()) {
        _serialPort->read();
    }

    if (!trySendCommand(command, sizeof(command))) {
        return false;
    }

    byte data[length];
    int index = 0;
    unsigned long startTime = millis();

    while (millis() - startTime < timeoutMs) {
        if (!_serialPort->available()) {
            delay(5);
            continue;
        }

        byte c = _serialPort->read();

        if (index == 0 && c != 0xFF) {
            continue;
        }

        data[index++] = c;

        if (index == length) {
            if (data[0] == 0xFF &&
                data[1] == 0xD7 &&
                validateChecksum(data, length)) {

                sensorData->command = data[1];
                sensorData->sensorType = data[2];
                sensorData->maxRange = ((int)data[3] << 8) | data[4];
                sensorData->units = data[5];
                sensorData->numberOfDecimals = data[6] >> 4;
                return true;
            }

            return false;
        }
    }

    return false;
}
