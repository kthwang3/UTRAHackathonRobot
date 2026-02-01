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
    moveForward(motorSetup);
    int distance = getDistanceCM();
    Serial.println(distance);
    delay(2000);
}
