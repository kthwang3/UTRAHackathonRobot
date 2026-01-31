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
  return pulseIn(sensorOut, LOW);
}

int readGreenPW() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  return pulseIn(sensorOut, LOW);
}

int readBluePW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  return pulseIn(sensorOut, LOW);
}

void readColor(int &r, int &g, int &b) {
  r = readRedPW();
  delay(50);
  g = readGreenPW();
  delay(50);
  b = readBluePW();
  delay(50);
}

//////////////////////////////
// MOTOR CONTROL (L298N)
//////////////////////////////

// Left motor
#define ENA 9
#define IN1 8
#define IN2 7

// Right motor
#define ENB 4
#define IN3 6
#define IN4 5

int motorSpeed = 180;

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnRightInPlace(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnLeftInPlace(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

//////////////////////////////
// TURN LOGIC
//////////////////////////////

enum TrackColor {
  TRACK_RED,
  TRACK_GREEN
};

TrackColor currentTrack = TRACK_RED;

// Convert pulse width to "strength"
// smaller PW = stronger color
long colorStrength(int pw) {
  if (pw <= 0) pw = 1;
  return 1000000L / pw;
}

long scoreForTarget(int r, int g, TrackColor target) {
  if (target == TRACK_RED) {
    return colorStrength(r) - colorStrength(g);
  } else {
    return colorStrength(g) - colorStrength(r);
  }
}

long scanDirection(bool right) {
  int r, g, b;
  long totalScore = 0;

  if (right) turnRightInPlace(motorSpeed);
  else       turnLeftInPlace(motorSpeed);

  delay(200);
  stopMotors();
  delay(80);

  for (int i = 0; i < 5; i++) {
    readColor(r, g, b);
    totalScore += scoreForTarget(r, g, currentTrack);
    delay(40);
  }

  return totalScore;
}

void decideTurn() {
  long rightScore = scanDirection(true);

  // return to center
  turnLeftInPlace(motorSpeed);
  delay(200);
  stopMotors();
  delay(100);

  long leftScore = scanDirection(false);

  // return to center
  turnRightInPlace(motorSpeed);
  delay(200);
  stopMotors();
  delay(100);

  if (rightScore > leftScore) {
    turnRightInPlace(motorSpeed);
    delay(300);
  } else {
    turnLeftInPlace(motorSpeed);
    delay(300);
  }

  stopMotors();
}

//////////////////////////////
// ARDUINO SETUP / LOOP
//////////////////////////////

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
  colorSensorSetup();
}

void loop() {
  // Example usage:
  moveForward(motorSpeed);
  delay(1500);

  stopMotors();
  delay(300);

  decideTurn();

  delay(2000);
}
