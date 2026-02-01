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

// Speed values (0–255)
int motorSpeed = 255;

void setup() {
  // Set all motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop motors at start
  stopMotors();
}

void loop() {
  // Move forward
  moveForward(motorSpeed);
  delay(2000);

  // Stop
  stopMotors();
  delay(1000);
}
