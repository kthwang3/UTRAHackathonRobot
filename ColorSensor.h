#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>

// ---------------- Pin Definitions ----------------
#define S0 A1
#define S1 A0
#define S2 12
#define S3 11
#define sensorOut 13

// ---------------- Public API ----------------
void colorSensorSetup();
void colorSensorRead(int &redPW, int &greenPW, int &bluePW);

// ---------------- Internal Helpers ----------------
int getRedPW();
int getGreenPW();
int getBluePW();

// ---------------- Implementations ----------------
inline void colorSensorSetup() {
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    // 20% frequency scaling
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);

    pinMode(sensorOut, INPUT);
}

inline void colorSensorRead(int &redPW, int &greenPW, int &bluePW) {
    redPW = getRedPW();
    delay(200);

    greenPW = getGreenPW();
    delay(200);

    bluePW = getBluePW();
    delay(200);
}

inline int getRedPW() {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    return pulseIn(sensorOut, LOW);
}

inline int getGreenPW() {
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    return pulseIn(sensorOut, LOW);
}

inline int getBluePW() {
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    return pulseIn(sensorOut, LOW);
}

#endif // COLOR_SENSOR_H
