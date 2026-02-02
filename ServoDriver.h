#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <Servo.h>

// =====================
// SERVO OBJECTS
// =====================
static Servo armServo;
static Servo clawServo;

// =====================
// DEFAULT PINS
// =====================
#define ARM_SERVO_PIN 3
#define CLAW_SERVO_PIN A2

// =====================
// ARM POSITIONS
// =====================
#define ARM_UP 30
#define ARM_DOWN 150
#define ARM_MID 90

// =====================
// CLAW POSITIONS
// =====================
#define CLAW_OPEN 150
#define CLAW_CLOSED 60
#define CLAW_MID 100

// =====================
// PUBLIC API
// =====================
inline void initServo(int pin = ARM_SERVO_PIN) {
    armServo.attach(pin);
}

inline void initServos(int armPin = ARM_SERVO_PIN, int clawPin = CLAW_SERVO_PIN) {
    armServo.attach(armPin);
    clawServo.attach(clawPin);
}

inline void setServoAngle(int angle) {
    angle = constrain(angle, 0, 180);
    armServo.write(angle);
}

inline void setArmAngle(int angle) {
    angle = constrain(angle, 0, 180);
    armServo.write(angle);
}

inline void setClawAngle(int angle) {
    angle = constrain(angle, 0, 180);
    clawServo.write(angle);
}

inline void armUp() {
    armServo.write(ARM_UP);
}

inline void armDown() {
    armServo.write(ARM_DOWN);
}

inline void armMid() {
    armServo.write(ARM_MID);
}

inline void clawOpen() {
    clawServo.write(CLAW_OPEN);
}

inline void clawClose() {
    clawServo.write(CLAW_CLOSED);
}

inline void grabBox() {
    clawClose();
    delay(300);
    armUp();
}

inline void releaseBox() {
    armDown();
    delay(300);
    clawOpen();
}

inline void sweepServo() {
    for (int pos = 0; pos <= 180; pos++) {
        armServo.write(pos);
        delay(15);
    }
    for (int pos = 180; pos >= 0; pos--) {
        armServo.write(pos);
        delay(15);
    }
}

#endif // SERVO_DRIVER_H
