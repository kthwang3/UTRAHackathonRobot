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
    motorSetup();
}

enum ColorState {
        RED = 1,
        GREEN = 2,
};

ColorState currentState = GREEN;

void loop() {
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
    Serial.println("Testing: Move Backward + Turn Right");
    moveBackward(motorSpeed);
    delay(2000);           // move backward for 2 seconds
    turnRight(motorSpeed);
    delay(3000);           // turn right for 3 seconds
    stopMotors();
    Serial.println("Stopped");
    delay(2000);           // wait 2 seconds

    Serial.println("Testing: Move Backward + Turn Left");
    moveBackward(motorSpeed);
    delay(2000);           // move backward for 2 seconds
    turnLeft(motorSpeed);
    delay(3000);           // turn left for 3 seconds
    stopMotors();
    Serial.println("Stopped");
    delay(2000);           // wait 2 seconds
}
