/*!
 *  @file TB600B_H2S_2.h
 *
 *  @brief Arduino library for the ECsense TB600B-H2S-2 hydrogen sulfide sensor.
 *
 *  @author Ross Edwards
 *
 *  @license MIT License
 *
 *  MIT License
 *  ------------
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including, without limitation, the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *  @section VERSION HISTORY
 *
 *  1.0 - Dec, 2023 - Initial release of the TB600B-H2S-2 library.
 *  This version introduces support for the ECsense TB600B-H2S-2 sensor,
 *  including UART communication, passive data requests, sensor presence checks,
 *  gas concentration, temperature, and relative humidity parsing.
 */

#ifndef TB600B_H2S_2_H
#define TB600B_H2S_2_H

#include <Arduino.h>

enum TB600B_H2S_2_ResponseType {
    TB600B_H2S_2_NO_DATA,
    TB600B_H2S_2_GAS_CONCENTRATION,
    TB600B_H2S_2_INVALID_RESPONSE
};

struct TB600B_H2S_2_SensorData {
    uint8_t command = 0;
    uint8_t sensorType = 0;
    uint16_t maxRange = 0;
    uint8_t units = 0;
    uint8_t numberOfDecimals = 0;
};

class TB600B_H2S_2 {
public:
    TB600B_H2S_2(HardwareSerial& serialPort, long baudRate = 9600);

    bool begin();

    bool checkPresent(uint8_t attempts = 2, unsigned long delayBetweenAttemptsMs = 250);

    void switchToActiveUpload();
    void switchToPassiveUpload();

    bool queryLights();

    void passiveDataCommand();

    TB600B_H2S_2_ResponseType getGasTemperature(
        int* gasConcentrationPpb,
        float* temperatureC,
        float* relativeHumidity
    );

    bool getSensorInfo(TB600B_H2S_2_SensorData* sensorData);

private:
    HardwareSerial* _serialPort;
    long _baudRate;

    static const size_t RESPONSE_BUFFER_SIZE = 13;
    byte _buffer[RESPONSE_BUFFER_SIZE];
    size_t _bufferIndex = 0;

    bool trySendCommand(const byte* command, size_t size);
    bool validateChecksum(const byte* data, int length);
    int calculateConcentration(byte high, byte low);
    void clearSerialInput();
};

#endif  // TB600B_H2S_2_H
