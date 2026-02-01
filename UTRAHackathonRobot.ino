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

#define OBSTACLE_THRESHOLD_CM 15


void loop() {

    int distance = getDistanceCM();
    if distance >= 0 && distance < OBSTACLE_THRESHOLD_CM{
        stopMotors();
        Serial.println("Obstacle: ");
        Serial.println(distance);
        delay(200);
        return;
    }
    if (greenPW < redPW){
        greenState = GREEN;
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
    moveForward(motorSetup);
    int distance = getDistanceCM();
    Serial.println(distance);
    delay(2000);
}
