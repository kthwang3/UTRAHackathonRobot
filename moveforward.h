#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// =====================
// MOTOR PIN DEFINITIONS
// =====================

// Left motor
#define ENA 9
#define IN1 8
#define IN2 7

// Right motor
#define ENB 4
#define IN3 6
#define IN4 5

// =====================
// CONFIGURATION
// =====================
#define RIGHT_MOTOR_REVERSED true  // set true because this motor is physically reversed

// =====================
// PUBLIC API
// =====================
void motorSetup();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();

// =====================
// IMPLEMENTATION
// =====================

// Helper to set a motor with optional reversal
inline void setMotor(int inA, int inB, int pwmPin, int speed, bool reversed) {
    if (reversed) {
        digitalWrite(inA, LOW);
        digitalWrite(inB, HIGH);
    } else {
        digitalWrite(inA, HIGH);
        digitalWrite(inB, LOW);
    }
    analogWrite(pwmPin, speed);
}

inline void motorSetup() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    stopMotors();
}

inline void moveForward(int speed) {
    setMotor(IN1, IN2, ENA, speed, false);                 // left motor normal
    setMotor(IN3, IN4, ENB, speed, RIGHT_MOTOR_REVERSED);  // right motor reversed
}

inline void moveBackward(int speed) {
    setMotor(IN1, IN2, ENA, speed, true);                  // left motor backward
    setMotor(IN3, IN4, ENB, speed, !RIGHT_MOTOR_REVERSED); // right motor backward
}

inline void turnLeft(int speed) {
    setMotor(IN1, IN2, ENA, speed, true);                  // left motor backward
    setMotor(IN3, IN4, ENB, speed, RIGHT_MOTOR_REVERSED);  // right motor forward
}

inline void turnRight(int speed) {
    setMotor(IN1, IN2, ENA, speed, false);                // left motor forward
    setMotor(IN3, IN4, ENB, speed, !RIGHT_MOTOR_REVERSED); // right motor backward
}

inline void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

#endif // MOTOR_DRIVER_H
