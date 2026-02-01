#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>

#define S0 A1
#define S1 A0
#define S2 12
#define S3 11
#define sensorOut 10   // ⬅️ FIXED

inline void colorSensorSetup() {
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);

    pinMode(sensorOut, INPUT);
    Serial.println("Color sensor ready");
}

inline int getRedPW() {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    return pulseIn(sensorOut, LOW, 30000);
}

inline int getGreenPW() {
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    return pulseIn(sensorOut, LOW, 30000);
}

inline int getBluePW() {
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    return pulseIn(sensorOut, LOW, 30000);
}

inline void colorSensorRead(int &r, int &g, int &b) {
    r = getRedPW();
    g = getGreenPW();
    b = getBluePW();
}

#endif
