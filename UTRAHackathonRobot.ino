#include "ColorSensor.h"
#include "moveforward.h"
#include "ServoDriver.h"
#include "ultraSensor.h"

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

enum ColorState {
        RED = 1,
        GREEN = 2,
};

ColorState currentState = GREEN;

//Obstacle Avoidance
const int AVOID_SPEED = 180;
const int AVOID_TURN_MS = 400;
const int AVOID_MOVE_MS = 500;

// Line-follow steering (when off line, alternate turn to find it again)
const int STEER_SPEED = 180;
const int STEER_MS = 120;       // how long each search turn
const int SEARCH_TIMEOUT_MS = 5000;  // max time to search for line after avoid

// Returns true when black or red line is seen again
bool searchForLine() {
    unsigned long start = millis();
    static bool searchLeft = true;
    while (millis() - start < (unsigned long)SEARCH_TIMEOUT_MS) {
        colorSensorRead(redPW, greenPW, bluePW);
        bool onBlack = (redPW > BLACK_PW_THRESHOLD && greenPW > BLACK_PW_THRESHOLD);
        bool onRed   = (redPW > greenPW);
        if (onBlack || onRed) {
            stopMotors();
            Serial.println("Line found!");
            return true;
        }
        if (searchLeft) {
            turnLeft(STEER_SPEED);
        }else{
            turnRight(STEER_SPEED);
        }   
        delay(STEER_MS);
        searchLeft = !searchLeft;
    }
    stopMotors();
    Serial.println("Search timeout");
    return false;
}
void avoidObstacle() {
    stopMotors();
    Serial.println("Obstacle detected, avoiding...");
    delay(200);

    // Fixed path: turn R -> move -> turn R -> move -> turn L -> move (ultrasonic not on servo)
    turnRight(AVOID_SPEED);
    delay(AVOID_TURN_MS);
    stopMotors();
    delay(100);

    moveForward(AVOID_SPEED);
    delay(AVOID_MOVE_MS);
    stopMotors();
    delay(100);

    turnRight(AVOID_SPEED);
    delay(AVOID_TURN_MS);
    stopMotors();
    delay(100);

    moveForward(AVOID_SPEED);
    delay(AVOID_MOVE_MS);
    stopMotors();
    delay(100);

    turnLeft(AVOID_SPEED);
    delay(AVOID_TURN_MS);
    stopMotors();
    delay(100);

    moveForward(AVOID_SPEED);
    delay(AVOID_MOVE_MS);
    stopMotors();
    delay(100);
    searchForLine();
}

#define OBSTACLE_THRESHOLD_CM 15

// Black line: high PW = low reflectance (dark). Tune so sensor on black gives PW above this.
const int BLACK_PW_THRESHOLD = 1500;  // increase if black not detected, decrease if floor triggers it

// Blue box: lower PW = stronger color on TCS3200. Only drop once per run.
static bool boxDropped = false;
static bool searchLeft = true;

void loop() {

    int distance = getDistanceCM();
    if (distance >= 0 && distance < OBSTACLE_THRESHOLD_CM) {
        avoidObstacle();
        return;
    }

    colorSensorRead(redPW, greenPW, bluePW);

    // See blue box -> back up and gently drop the box (once)
    if (!boxDropped && bluePW < redPW && bluePW < greenPW) {
        stopMotors();
        Serial.println("Blue box - backing up and dropping...");
        moveBackward(BACKUP_SPEED);
        delay(BACKUP_MS);
        stopMotors();
        delay(200);
        setServoAngle(SERVO_ARM_DOWN);   // gently lower arm to drop box
        delay(500);   // give servo time to move
        boxDropped = true;
        return;
    }

    // Line follow: go forward on black OR on red; otherwise search for the line
    bool onBlack = (redPW > BLACK_PW_THRESHOLD && greenPW > BLACK_PW_THRESHOLD);
    bool onRed   = (redPW > greenPW);

    if (onBlack || onRed) {
        currentState = RED;
        moveForward(motorSpeed);
        if (onBlack) Serial.println("Black (moving)");
        else         Serial.println("Red (moving)");
    } else {
        currentState = GREEN;
        if (searchLeft) {
            turnLeft(STEER_SPEED);
        } else {
            turnRight(STEER_SPEED);
        }
        delay(STEER_MS);
        stopMotors();
        searchLeft = !searchLeft;
    }

    Serial.print("dist=");
    Serial.print(distance);
    Serial.print(" R=");
    Serial.print(redPW);
    Serial.print(" G=");
    Serial.print(greenPW);
    Serial.print(" B=");
    Serial.println(bluePW);
    delay(50);
}
