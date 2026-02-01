#include "Turn.h"

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopMotors();
  colorSensorSetup();
}

void loop() {
  followLineStep();
  delay(20);
}
