#include <ColorSensor.h>

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
// TRACK/SCORING LOGIC (red target)
//////////////////////////////

// Convert pulse width to "strength" (smaller PW = stronger color)
long colorStrength(int pw) {
  if (pw <= 0) pw = 1;
  return 1000000L / pw;
}

long scoreForRed(int r, int g) {
  return colorStrength(r) - colorStrength(g);
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
    totalScore += scoreForRed(r, g);
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
// RED-FOLLOW BEHAVIOR
//////////////////////////////

#define FORWARD_STEP_MS 120
#define SCORE_THRESHOLD 30L

// Ultrasonic sensor (right-side) - HC-SR04 style
#define US_TRIG_PIN 2
#define US_ECHO_PIN 3
#define OBSTACLE_DIST_CM 25  // if an object is closer than this, avoid
#define BACKUP_MS 300
#define AVOID_TURN_MS 350

void ultrasonicSetup() {
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);
}

long readDistanceCM() {
  // Send a 10us pulse to trigger
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  long duration = pulseIn(US_ECHO_PIN, HIGH, 30000); // timeout 30ms
  if (duration <= 0) return 300; // no echo -> far away
  // Speed of sound ~343 m/s => 29.1 microsec per cm for round trip
  long cm = duration / 29 / 2;
  return cm;
}

void moveBackward(int speed) {
  // reverse both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void avoidRightObstacle() {
  Serial.println("Right obstacle detected -> avoiding");
  // Back up a bit, then turn left to steer away from the right-side block
  moveBackward(motorSpeed);
  delay(BACKUP_MS);
  stopMotors();
  delay(80);
  turnLeftInPlace(motorSpeed);
  delay(AVOID_TURN_MS);
  stopMotors();
  delay(80);
}

void followTrack() {
  int r, g, b;
  readColor(r, g, b);
  long score = scoreForRed(r, g);

  Serial.print("R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.print(b);
  Serial.print(" score="); Serial.println(score);

  // First: check right-side ultrasonic for blocks and avoid if needed
  long dist = readDistanceCM();
  Serial.print(" rightDist="); Serial.println(dist);
  if (dist > 0 && dist < OBSTACLE_DIST_CM) {
    avoidRightObstacle();
    return;
  }

  if (score >= SCORE_THRESHOLD) {
    moveForward(motorSpeed);
    delay(FORWARD_STEP_MS);
    stopMotors();
  } else {
    stopMotors();
    delay(80);
    decideTurn();
  }
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
  ultrasonicSetup();
  Serial.begin(9600);
}

void loop() {
  // Keep following the red track continuously
  followTrack();
  delay(20);
}
