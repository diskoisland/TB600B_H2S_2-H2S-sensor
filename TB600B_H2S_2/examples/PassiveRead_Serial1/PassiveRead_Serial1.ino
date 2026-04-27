#include <Arduino.h>
#include <TB600B_H2S_2.h>
#include <millisDelay.h>

// Uses Serial for USB debug output and Serial1 for the TB600B-H2S-2 sensor.
TB600B_H2S_2 h2sSensor(Serial1, 9600);

millisDelay h2sReadTimer;
millisDelay h2sReconnectTimer;

const unsigned long H2S_READ_TIME = 5000;
const unsigned long H2S_RECONNECT_TIME = 30000;

bool h2sConnected = false;
bool h2sDataReceived = false;

int h2sPpb = -999;
int gasConcentrationPpb = -1;
float h2sTempC = -999.0;
float h2sRh = -999.0;

void checkH2SReconnect();
void processH2SData();

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("TB600B-H2S-2 passive read example on Serial1");

    h2sSensor.begin();

    if (h2sSensor.checkPresent()) {
        h2sConnected = true;
        h2sSensor.switchToPassiveUpload();

        Serial.println("H2S sensor connected.");

        TB600B_H2S_2_SensorData sensorData;
        if (h2sSensor.getSensorInfo(&sensorData)) {
            Serial.println("H2S sensor info:");
            Serial.print("Command: 0x");
            Serial.println(sensorData.command, HEX);
            Serial.print("Sensor type: ");
            Serial.println(sensorData.sensorType);
            Serial.print("Max range: ");
            Serial.println(sensorData.maxRange);
            Serial.print("Number of decimals: ");
            Serial.println(sensorData.numberOfDecimals);

            if (sensorData.units == 0x04) {
                Serial.println("Units: ppb");
            }
        } else {
            Serial.println("H2S present, but sensor info read failed.");
        }

        h2sReadTimer.start(H2S_READ_TIME);
    } else {
        h2sConnected = false;
        h2sPpb = -999;
        gasConcentrationPpb = -1;
        h2sTempC = -999.0;
        h2sRh = -999.0;

        Serial.println("H2S sensor not detected.");
    }

    h2sReconnectTimer.start(H2S_RECONNECT_TIME);
}

void loop() {
    if (h2sReconnectTimer.justFinished()) {
        h2sReconnectTimer.repeat();
        checkH2SReconnect();
    }

    processH2SData();
}

void checkH2SReconnect() {
    if (h2sConnected) {
        return;
    }

    if (h2sSensor.checkPresent()) {
        h2sPpb = -999;
        gasConcentrationPpb = -1;
        h2sTempC = -999.0;
        h2sRh = -999.0;
        h2sDataReceived = false;

        h2sSensor.switchToPassiveUpload();

        h2sConnected = true;
        h2sReadTimer.start(H2S_READ_TIME);

        Serial.println("H2S sensor reconnected.");
    }
}

void processH2SData() {
    static uint8_t h2sMissedResponses = 0;
    static bool waitingForH2SResponse = false;

    const uint8_t H2S_MAX_MISSED_RESPONSES = 3;

    if (!h2sConnected) {
        waitingForH2SResponse = false;
        h2sMissedResponses = 0;
        return;
    }

    if (waitingForH2SResponse) {
        TB600B_H2S_2_ResponseType response =
            h2sSensor.getGasTemperature(&gasConcentrationPpb, &h2sTempC, &h2sRh);

        if (response == TB600B_H2S_2_GAS_CONCENTRATION) {
            h2sPpb = gasConcentrationPpb;
            h2sDataReceived = true;
            waitingForH2SResponse = false;
            h2sMissedResponses = 0;

            Serial.print("H2S ppb: ");
            Serial.print(h2sPpb);
            Serial.print(", Temp C: ");
            Serial.print(h2sTempC, 2);
            Serial.print(", RH %: ");
            Serial.println(h2sRh, 2);
        } else if (response == TB600B_H2S_2_INVALID_RESPONSE) {
            waitingForH2SResponse = false;
            h2sMissedResponses++;

            Serial.print("Invalid H2S response. Missed count: ");
            Serial.println(h2sMissedResponses);
        }
    }

    if (h2sReadTimer.justFinished()) {
        h2sReadTimer.restart();

        if (waitingForH2SResponse) {
            h2sMissedResponses++;

            Serial.print("No H2S response. Missed count: ");
            Serial.println(h2sMissedResponses);
        }

        if (h2sMissedResponses >= H2S_MAX_MISSED_RESPONSES) {
            h2sConnected = false;
            h2sDataReceived = false;
            waitingForH2SResponse = false;

            h2sPpb = -999;
            gasConcentrationPpb = -1;
            h2sTempC = -999.0;
            h2sRh = -999.0;

            h2sMissedResponses = 0;

            Serial.println("H2S sensor lost. Marking disconnected.");
            return;
        }

        h2sSensor.passiveDataCommand();
        h2sDataReceived = false;
        waitingForH2SResponse = true;
    }
}
