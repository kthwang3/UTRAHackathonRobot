#include "ColorSensor.h"
#include "moveforward.h"
#include "ServoDriver.h"
#include "ultraSensor.h"

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int motorSpeed = 255;

void setup() {
    Serial.begin(9600);
    colorSensorSetup();
    ultrasonicSetup();
    motorSetup();
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

void avoidObstacle(){
    stopMotors();
    Serial.println("Obstacle detected, avoiding...");
    delay(200);
    
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
}

#define OBSTACLE_THRESHOLD_CM 15


void loop() {

    int distance = getDistanceCM();
    if (distance >= 0 && distance < OBSTACLE_THRESHOLD_CM){
        avoidObstacle();
        return;
    }

    // read color sensor and go forward when red
    colorSensorRead(redPW, greenPW, bluePW);
    if (redPW < greenPW){
        currentState = RED;
        moveForward(motorSpeed);
        serial.println("Red (moving)");
    }else{
        currentState = GREEN;
        stopMotors();
        serial.println("Green (stopped)");
    }

    Serial.print("dist=");
    Serial.print(distance);
    Serial.print(" R=");
    Serial.print(redPW);
    Serial.print(" G=");
    Serial.print(greenPW);
    Serial.print(" B=");
    Serial.println(bluePW);
    delay(100);
}
