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

// Line-follow steering (when off line, alternate turn to find it again)
const int STEER_SPEED = 180;
const int STEER_MS = 120;       // how long each search turn
const int SEARCH_TIMEOUT_MS = 5000;  // max time to search for line after avoid

// Returns true when red (on line) is seen again
bool searchForLine() {
    unsigned long start = millis();
    static bool searchLeft = true;
    while (millis() - start < (unsigned long)SEARCH_TIMEOUT_MS) {
        if redPW < greenPW{
            stopMotors();
            Serial.println("Line found!");
            return true;
        }
        if searchLeft{
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
    searchForLine();
}

#define OBSTACLE_THRESHOLD_CM 15

//we alternate between left and right turns to search for the line
static bool searchLeft = true;


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
        Serial.println("Red (moving)");
    }else{
        currentState = GREEN;
        if (searchLeft){
            turnLeft(STEER_SPEED);
        }else{
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
