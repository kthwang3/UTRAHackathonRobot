#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <Servo.h>

// =====================
// SERVO OBJECT
// =====================
static Servo myServo;   // static prevents multiple-definition issues

// =====================
// PUBLIC API
// =====================
inline void initServo(int pin = 9) {
    myServo.attach(pin);
}

inline void sweepServo() {
    // Move from 0 to 180
    for (int pos = 0; pos <= 180; pos++) {
        myServo.write(pos);
        delay(15);
    }

    // Move from 180 to 0
    for (int pos = 180; pos >= 0; pos--) {
        myServo.write(pos);
        delay(15);
    }
}

inline void setServoAngle(int angle) {
    angle = constrain(angle, 0, 180);
    myServo.write(angle);
}

#endif // SERVO_DRIVER_H
