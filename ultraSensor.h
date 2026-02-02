#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// =====================
// PIN DEFINITIONS
// =====================
#define TRIG_PIN 2
#define ECHO_PIN 13

// =====================
// PUBLIC API
// =====================
inline void ultrasonicSetup() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

inline int getDistanceCM() {
    // Trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read echo (timeout 30ms ≈ 5m range)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0) {
        return -1; // no echo / out of range
    }

    return duration * 0.034 / 2;
}

#endif // ULTRASONIC_H
