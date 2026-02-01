#include "ColorSensor.h"
#include "moveforward.h"
#include "ServoDriver.h"
#include "ultraSensor.h"
#define BLACK_THRESHOLD 150


int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int motorSpeed = 255;

// Arm: UP = holding box at start; DOWN = gently drop when we see blue box.
#define SERVO_PIN 3
#define SERVO_CENTER  90   // straight ahead (mid-range)
#define SERVO_LEFT    45   // look left
#define SERVO_RIGHT  135   // look right
#define SERVO_ARM_UP   30   // arm up = holding box (tune to your mechanism)
#define SERVO_ARM_DOWN 150  // arm down = release/drop box (tune to your mechanism)

// Blue box drop: back up this long, then lower arm
const int BACKUP_SPEED = 150;
const int BACKUP_MS = 600;   // how long to back up before dropping (tune as needed)

void setup() {
    Serial.begin(9600);
    colorSensorSetup();
    ultrasonicSetup();
    motorSetup();
    initServo(SERVO_PIN);
    setServoAngle(SERVO_ARM_DOWN);   // start with arm down
    delay(500);
    setServoAngle(SERVO_ARM_UP);     // then move arm up (e.g. pick up / hold box)
}

void loop() {
    int redPW, greenPW, bluePW;

    colorSensorRead(redPW, greenPW, bluePW);

    int brightness = redPW + greenPW + bluePW;

    Serial.print("R=");
    Serial.print(redPW);
    Serial.print(" G=");
    Serial.print(greenPW);
    Serial.print(" B=");
    Serial.print(bluePW);
    Serial.print(" sum=");
    Serial.println(brightness);

    if (brightness > BLACK_THRESHOLD) {
        moveForward(150);
        Serial.println("BLACK -> forward");
    } else {
        stopMotors();
        Serial.println("NOT black -> stop");
    }

    delay(50);
}
