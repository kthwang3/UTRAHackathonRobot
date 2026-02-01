#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

// =====================
// IR SENSOR PINS
// =====================
#define IR_LEFT_PIN A3
#define IR_RIGHT_PIN A4

// =====================
// THRESHOLDS
// =====================
#define IR_BLACK_THRESHOLD 500
#define IR_WHITE_THRESHOLD 200

// =====================
// PUBLIC API
// =====================
inline void irSensorSetup() {
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);
}

inline int readIRLeft() {
    return analogRead(IR_LEFT_PIN);
}

inline int readIRRight() {
    return analogRead(IR_RIGHT_PIN);
}

inline bool irLeftOnBlack() {
    return analogRead(IR_LEFT_PIN) > IR_BLACK_THRESHOLD;
}

inline bool irRightOnBlack() {
    return analogRead(IR_RIGHT_PIN) > IR_BLACK_THRESHOLD;
}

inline bool irLeftOnWhite() {
    return analogRead(IR_LEFT_PIN) < IR_WHITE_THRESHOLD;
}

inline bool irRightOnWhite() {
    return analogRead(IR_RIGHT_PIN) < IR_WHITE_THRESHOLD;
}

inline bool bothOnLine() {
    return irLeftOnBlack() && irRightOnBlack();
}

inline bool bothOffLine() {
    return !irLeftOnBlack() && !irRightOnBlack();
}

inline int getLinePosition() {
    bool left = irLeftOnBlack();
    bool right = irRightOnBlack();

    if (left && right) return 0;
    if (left && !right) return -1;
    if (!left && right) return 1;
    return 0;
}

#endif // IR_SENSOR_H
