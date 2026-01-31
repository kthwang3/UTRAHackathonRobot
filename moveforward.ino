void moveForward(int speed) {
  // Left motor forward (FLIPPED)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right motor forward (unchanged)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  // Left motor backward (FLIPPED)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor backward (unchanged)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}
