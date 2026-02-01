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

    if (greenPW < redPW){
        currentState = GREEN;
        stopMotors();
        serial.println("Green (stopped)");
    }else if(redPW < greenPW){
        currentState = RED;
        serial.println("Red (moving)");
    }

    if (currentState == RED){
        moveForward(motorSpeed);
    }else{
        stopMotors();
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
    /*
    colorSensorRead(redPW, greenPW, bluePW);

    if(greenPW < redPW){
        currentState = GREEN; 
        stopMotors();
        Serial.println(5);
    } else if(redPW < greenPW){
        Serial.println(4);
        currentState = RED;
    }
    if(currentState == RED){
        moveForward(motorSpeed);
    }
    Serial.println(currentState);
    */
    if (currentState == RED){
        moveForward(motorSpeed);
    }
    delay(100);
}
