#pragma once
#include <Arduino.h>

//////////////////////////////
// COLOR SENSOR (TCS3200)
//////////////////////////////

#define S0 A1
#define S1 A0
#define S2 12
#define S3 11
#define sensorOut 13

void colorSensorSetup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // 20% frequency scaling
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  pinMode(sensorOut, INPUT);
}

int readRedPW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  return pulseIn(sensorOut, LOW, 30000); // timeout helps avoid hangs
}

int readGreenPW() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  return pulseIn(sensorOut, LOW, 30000);
}

int readBluePW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  return pulseIn(sensorOut, LOW, 30000);
}

void readColor(int &r, int &g, int &b) {
  r = readRedPW();
  delay(5);
  g = readGreenPW();
  delay(5);
  b = readBluePW();
  delay(5);
}

unsigned long readSumPW() {
  int r, g, b;
  readColor(r, g, b);

  // If pulseIn times out it returns 0 — avoid zero wrecking logic
  if (r <= 0) r = 30000;
  if (g <= 0) g = 30000;
  if (b <= 0) b = 30000;

  unsigned long s = (unsigned long)r + (unsigned long)g + (unsigned long)b;

  // light smoothing
  static unsigned long sFilt = 0;
  if (sFilt == 0) sFilt = s;
  sFilt = (sFilt * 3 + s) / 4;
  return sFilt;
}

//////////////////////////////
// MOTOR CONTROL (L298N)
//////////////////////////////

#define ENA 9
#define IN1 8
#define IN2 7

#define ENB 4
#define IN3 6
#define IN4 5

int motorSpeed = 180;

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void driveForward(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void turnRightInPlace(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnLeftInPlace(int speed) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

//////////////////////////////
// LINE FOLLOW (FIXED THRESHOLD)
//////////////////////////////

// You said you want 150 for simplicity:
const unsigned long BLACK_SUM_THRESH = 150;

// Tunables for behavior
const int BASE_SPEED = 170;
const int TURN_SPEED = 160;
const int NUDGE_MS   = 60;   // how much we “peek” left/right

bool onBlack(unsigned long sumPW) {
  // In your setup: black tape -> sum goes UP
  return sumPW > BLACK_SUM_THRESH;
}

unsigned long sampleNudge(bool right) {
  if (right) turnRightInPlace(TURN_SPEED);
  else       turnLeftInPlace(TURN_SPEED);

  delay(NUDGE_MS);
  stopMotors();
  delay(15);

  return readSumPW();
}

void followLineStep() {
  unsigned long s = readSumPW();

  if (onBlack(s)) {
    // on the line: go straight
    driveForward(BASE_SPEED, BASE_SPEED);
    return;
  }

  // off line: peek right then left, choose the side with "more black" (bigger sum)
  unsigned long sRight = sampleNudge(true);

  // return to center
  turnLeftInPlace(TURN_SPEED);
  delay(NUDGE_MS);
  stopMotors();
  delay(15);

  unsigned long sLeft = sampleNudge(false);

  // return to center
  turnRightInPlace(TURN_SPEED);
  delay(NUDGE_MS);
  stopMotors();
  delay(15);

  if (sRight > sLeft) {
    // black is more to the right -> steer right
    driveForward(BASE_SPEED + 15, BASE_SPEED - 40);
  } else {
    // black is more to the left -> steer left
    driveForward(BASE_SPEED - 40, BASE_SPEED + 15);
  }
}

