// UTRA Hackathon Biathlon Robot
// Full mission implementation with state machine

#include "ColorSensor.h"
#include "moveforward.h"
#include "ServoDriver.h"
#include "ultraSensor.h"
#include "IRSensor.h"

// =====================
// MISSION STATES
// =====================
enum MissionState {
    STATE_INIT,
    STATE_PICKUP_BOX,
    STATE_CARRY_TO_ZONE,
    STATE_DROP_BOX,
    STATE_DETECT_PATH,
    STATE_GREEN_PATH,
    STATE_RED_PATH,
    STATE_TARGET_SHOOTING,
    STATE_OBSTACLE_COURSE,
    STATE_RETURN_HOME,
    STATE_COMPLETE,
    STATE_ERROR
};

MissionState currentState = STATE_INIT;
MissionState previousState = STATE_INIT;

// =====================
// SUB-STATES FOR COMPLEX TASKS
// =====================
enum GreenPathSubState {
    GREEN_CLIMB_RAMP,
    GREEN_NAVIGATE_RINGS,
    GREEN_FIND_BLACK_CENTER,
    GREEN_PICKUP_BALL,
    GREEN_LAUNCH_BALL,
    GREEN_DESCEND_RAMP
};

enum RedPathSubState {
    RED_FOLLOW_PATH,
    RED_AVOID_OBSTACLE,
    RED_SHARP_TURN
};

GreenPathSubState greenSubState = GREEN_CLIMB_RAMP;
RedPathSubState redSubState = RED_FOLLOW_PATH;

// =====================
// SENSOR VALUES
// =====================
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
int distance = 0;

// =====================
// MOTOR SPEEDS
// =====================
const int SPEED_FAST = 255;
const int SPEED_NORMAL = 200;
const int SPEED_SLOW = 150;
const int SPEED_TURN = 180;
const int SPEED_RAMP = 255;

// =====================
// COLOR THRESHOLDS (tune these based on your environment)
// =====================
const int BLACK_PW_THRESHOLD = 1500;
const int WHITE_PW_MAX = 400;
const int RED_DOMINANCE = 100;
const int GREEN_DOMINANCE = 100;
const int BLUE_DOMINANCE = 100;

// =====================
// DISTANCE THRESHOLDS
// =====================
const int OBSTACLE_CLOSE = 10;
const int OBSTACLE_FAR = 20;
const int WALL_DISTANCE = 8;

// =====================
// TIMING
// =====================
const int TURN_90_MS = 500;
const int TURN_180_MS = 1000;
const int BACKUP_MS = 400;
const int SEARCH_TIMEOUT_MS = 3000;
const unsigned long MISSION_TIMEOUT_MS = 300000;

// =====================
// STATE FLAGS
// =====================
bool boxPickedUp = false;
bool boxDropped = false;
bool pathChosen = false;
bool ballPickedUp = false;
bool ballLaunched = false;
bool returnStarted = false;

// =====================
// TIMING TRACKING
// =====================
unsigned long missionStartTime = 0;
unsigned long stateStartTime = 0;
unsigned long lastActionTime = 0;

// =====================
// RECOVERY
// =====================
int recoveryAttempts = 0;
const int MAX_RECOVERY_ATTEMPTS = 3;

// =====================
// DEBUG
// =====================
bool debugMode = true;

void debugPrint(const char* msg) {
    if (debugMode) {
        Serial.print("[");
        Serial.print(millis() / 1000);
        Serial.print("s] ");
        Serial.println(msg);
    }
}

void debugState() {
    if (debugMode) {
        Serial.print("State: ");
        Serial.print(currentState);
        Serial.print(" | R:");
        Serial.print(redPW);
        Serial.print(" G:");
        Serial.print(greenPW);
        Serial.print(" B:");
        Serial.print(bluePW);
        Serial.print(" | Dist:");
        Serial.println(distance);
    }
}

// =====================
// COLOR DETECTION
// =====================
bool isBlack() {
    return (redPW > BLACK_PW_THRESHOLD && greenPW > BLACK_PW_THRESHOLD);
}

bool isWhite() {
    return (redPW < WHITE_PW_MAX && greenPW < WHITE_PW_MAX && bluePW < WHITE_PW_MAX);
}

bool isRed() {
    return (redPW < greenPW - RED_DOMINANCE && redPW < bluePW - RED_DOMINANCE);
}

bool isGreen() {
    return (greenPW < redPW - GREEN_DOMINANCE && greenPW < bluePW - GREEN_DOMINANCE);
}

bool isBlue() {
    return (bluePW < redPW - BLUE_DOMINANCE && bluePW < greenPW - BLUE_DOMINANCE);
}

void readSensors() {
    colorSensorRead(redPW, greenPW, bluePW);
    distance = getDistanceCM();
}

// =====================
// MOVEMENT HELPERS
// =====================
void turnLeftDegrees(int degrees, int speed) {
    int ms = (degrees * TURN_90_MS) / 90;
    turnLeft(speed);
    delay(ms);
    stopMotors();
}

void turnRightDegrees(int degrees, int speed) {
    int ms = (degrees * TURN_90_MS) / 90;
    turnRight(speed);
    delay(ms);
    stopMotors();
}

void backup(int ms, int speed) {
    moveBackward(speed);
    delay(ms);
    stopMotors();
}

void moveForwardTime(int ms, int speed) {
    moveForward(speed);
    delay(ms);
    stopMotors();
}

// =====================
// LINE FOLLOWING
// =====================
bool searchForLine(int timeoutMs) {
    unsigned long start = millis();
    bool searchLeft = true;

    while (millis() - start < (unsigned long)timeoutMs) {
        readSensors();

        if (isBlack() || isRed() || isGreen()) {
            stopMotors();
            return true;
        }

        if (searchLeft) {
            turnLeft(SPEED_TURN);
        } else {
            turnRight(SPEED_TURN);
        }
        delay(100);
        stopMotors();
        delay(50);
        searchLeft = !searchLeft;
    }

    stopMotors();
    return false;
}

void followLine() {
    int pos = getLinePosition();

    if (pos == 0) {
        moveForward(SPEED_NORMAL);
    } else if (pos < 0) {
        turnLeft(SPEED_TURN);
        delay(50);
    } else {
        turnRight(SPEED_TURN);
        delay(50);
    }
}

void followLineWithColor() {
    readSensors();

    if (isBlack()) {
        moveForward(SPEED_NORMAL);
    } else if (isRed()) {
        moveForward(SPEED_NORMAL);
    } else if (isGreen()) {
        moveForward(SPEED_NORMAL);
    } else {
        static bool searchDir = true;
        if (searchDir) {
            turnLeft(SPEED_TURN);
        } else {
            turnRight(SPEED_TURN);
        }
        delay(80);
        stopMotors();
        searchDir = !searchDir;
    }
}

// =====================
// OBSTACLE AVOIDANCE
// =====================
void avoidObstacle() {
    debugPrint("Avoiding obstacle");
    stopMotors();
    delay(100);

    backup(BACKUP_MS, SPEED_SLOW);
    delay(100);

    turnRightDegrees(90, SPEED_TURN);
    delay(100);

    moveForwardTime(500, SPEED_SLOW);
    delay(100);

    turnLeftDegrees(90, SPEED_TURN);
    delay(100);

    moveForwardTime(600, SPEED_SLOW);
    delay(100);

    turnLeftDegrees(90, SPEED_TURN);
    delay(100);

    moveForwardTime(500, SPEED_SLOW);
    delay(100);

    turnRightDegrees(90, SPEED_TURN);

    searchForLine(SEARCH_TIMEOUT_MS);
}

bool obstacleAhead() {
    return (distance > 0 && distance < OBSTACLE_CLOSE);
}

// =====================
// STATE HANDLERS
// =====================
void handleInit() {
    debugPrint("=== MISSION START ===");
    missionStartTime = millis();

    motorSetup();
    colorSensorSetup();
    ultrasonicSetup();
    irSensorSetup();
    initServos();

    clawOpen();
    delay(300);
    armDown();
    delay(500);

    currentState = STATE_PICKUP_BOX;
    stateStartTime = millis();
}

void handlePickupBox() {
    if (!boxPickedUp) {
        debugPrint("Picking up box");

        clawClose();
        delay(500);
        armUp();
        delay(500);

        boxPickedUp = true;
        currentState = STATE_CARRY_TO_ZONE;
        stateStartTime = millis();
    }
}

void handleCarryToZone() {
    readSensors();

    if (obstacleAhead()) {
        avoidObstacle();
        return;
    }

    if (isWhite() || isBlue()) {
        debugPrint("Zone detected");
        stopMotors();
        currentState = STATE_DROP_BOX;
        stateStartTime = millis();
        return;
    }

    followLineWithColor();

    if (millis() - stateStartTime > 30000) {
        debugPrint("Timeout carrying to zone");
        currentState = STATE_DROP_BOX;
    }
}

void handleDropBox() {
    if (!boxDropped) {
        debugPrint("Dropping box");
        stopMotors();

        backup(300, SPEED_SLOW);
        delay(200);

        armDown();
        delay(400);
        clawOpen();
        delay(300);
        armUp();
        delay(300);

        boxDropped = true;

        moveForwardTime(400, SPEED_SLOW);

        currentState = STATE_DETECT_PATH;
        stateStartTime = millis();
    }
}

void handleDetectPath() {
    readSensors();

    debugPrint("Detecting path split");

    moveForward(SPEED_SLOW);
    delay(100);

    readSensors();

    if (isRed()) {
        debugPrint("RED path detected - Obstacle Course");
        stopMotors();
        pathChosen = true;
        currentState = STATE_RED_PATH;
        redSubState = RED_FOLLOW_PATH;
        stateStartTime = millis();
        return;
    }

    if (isGreen()) {
        debugPrint("GREEN path detected - Target Shooting");
        stopMotors();
        pathChosen = true;
        currentState = STATE_GREEN_PATH;
        greenSubState = GREEN_CLIMB_RAMP;
        stateStartTime = millis();
        return;
    }

    if (millis() - stateStartTime > 10000) {
        debugPrint("Path detection timeout - defaulting to GREEN");
        stopMotors();
        currentState = STATE_GREEN_PATH;
        greenSubState = GREEN_CLIMB_RAMP;
        stateStartTime = millis();
    }
}

// =====================
// GREEN PATH: TARGET SHOOTING
// =====================
void handleGreenPath() {
    readSensors();

    switch (greenSubState) {
        case GREEN_CLIMB_RAMP:
            handleClimbRamp();
            break;

        case GREEN_NAVIGATE_RINGS:
            handleNavigateRings();
            break;

        case GREEN_FIND_BLACK_CENTER:
            handleFindBlackCenter();
            break;

        case GREEN_PICKUP_BALL:
            handlePickupBall();
            break;

        case GREEN_LAUNCH_BALL:
            handleLaunchBall();
            break;

        case GREEN_DESCEND_RAMP:
            handleDescendRamp();
            break;
    }
}

void handleClimbRamp() {
    debugPrint("Climbing ramp");

    if (obstacleAhead()) {
        avoidObstacle();
        return;
    }

    moveForward(SPEED_RAMP);

    static unsigned long rampStart = millis();
    if (millis() - rampStart > 5000) {
        debugPrint("Ramp climb complete");
        stopMotors();
        delay(200);
        greenSubState = GREEN_NAVIGATE_RINGS;
        rampStart = millis();
    }
}

void handleNavigateRings() {
    debugPrint("Navigating rings");

    readSensors();

    if (isBlue()) {
        debugPrint("On BLUE ring - moving inward");
        moveForward(SPEED_SLOW);
        delay(200);
    } else if (isRed()) {
        debugPrint("On RED ring - moving inward");
        moveForward(SPEED_SLOW);
        delay(200);
    } else if (isBlack()) {
        debugPrint("BLACK center found!");
        stopMotors();
        greenSubState = GREEN_FIND_BLACK_CENTER;
        return;
    } else {
        static bool spiralDir = true;
        if (spiralDir) {
            turnLeft(SPEED_TURN);
        } else {
            turnRight(SPEED_TURN);
        }
        delay(100);
        moveForward(SPEED_SLOW);
        delay(150);
        stopMotors();
        spiralDir = !spiralDir;
    }

    if (millis() - stateStartTime > 30000) {
        debugPrint("Ring navigation timeout");
        greenSubState = GREEN_PICKUP_BALL;
    }
}

void handleFindBlackCenter() {
    debugPrint("At black center");
    stopMotors();
    delay(500);
    greenSubState = GREEN_PICKUP_BALL;
}

void handlePickupBall() {
    if (!ballPickedUp) {
        debugPrint("Picking up ball");

        armDown();
        delay(400);
        clawClose();
        delay(400);
        armUp();
        delay(400);

        ballPickedUp = true;
        greenSubState = GREEN_LAUNCH_BALL;
    }
}

void handleLaunchBall() {
    if (!ballLaunched) {
        debugPrint("Launching ball");

        turnRightDegrees(45, SPEED_TURN);
        delay(200);

        armDown();
        delay(200);
        clawOpen();
        delay(200);
        armUp();
        delay(300);

        ballLaunched = true;
        greenSubState = GREEN_DESCEND_RAMP;
        stateStartTime = millis();
    }
}

void handleDescendRamp() {
    debugPrint("Descending ramp");

    turnRightDegrees(180, SPEED_TURN);
    delay(200);

    moveForward(SPEED_SLOW);
    delay(5000);
    stopMotors();

    currentState = STATE_RETURN_HOME;
    stateStartTime = millis();
}

// =====================
// RED PATH: OBSTACLE COURSE
// =====================
void handleRedPath() {
    readSensors();

    switch (redSubState) {
        case RED_FOLLOW_PATH:
            handleRedFollowPath();
            break;

        case RED_AVOID_OBSTACLE:
            handleRedAvoidObstacle();
            break;

        case RED_SHARP_TURN:
            handleRedSharpTurn();
            break;
    }
}

void handleRedFollowPath() {
    if (obstacleAhead()) {
        debugPrint("Obstacle on red path");
        redSubState = RED_AVOID_OBSTACLE;
        return;
    }

    if (isBlack()) {
        debugPrint("Black obstacle - avoiding");
        backup(300, SPEED_SLOW);
        redSubState = RED_AVOID_OBSTACLE;
        return;
    }

    if (isRed()) {
        moveForward(SPEED_NORMAL);
    } else {
        if (!searchForLine(2000)) {
            debugPrint("Lost red path");
            redSubState = RED_SHARP_TURN;
        }
    }

    if (millis() - stateStartTime > 60000) {
        debugPrint("Red path complete");
        currentState = STATE_RETURN_HOME;
        stateStartTime = millis();
    }
}

void handleRedAvoidObstacle() {
    debugPrint("Avoiding obstacle on red path");

    backup(BACKUP_MS, SPEED_SLOW);
    delay(100);

    readSensors();
    int leftDist, rightDist;

    turnLeftDegrees(45, SPEED_TURN);
    delay(100);
    leftDist = getDistanceCM();

    turnRightDegrees(90, SPEED_TURN);
    delay(100);
    rightDist = getDistanceCM();

    turnLeftDegrees(45, SPEED_TURN);

    if (leftDist > rightDist) {
        turnLeftDegrees(60, SPEED_TURN);
    } else {
        turnRightDegrees(60, SPEED_TURN);
    }

    moveForwardTime(500, SPEED_SLOW);

    if (searchForLine(SEARCH_TIMEOUT_MS)) {
        redSubState = RED_FOLLOW_PATH;
    } else {
        recoveryAttempts++;
        if (recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
            currentState = STATE_RETURN_HOME;
        }
    }
}

void handleRedSharpTurn() {
    debugPrint("Sharp turn detected");

    backup(200, SPEED_SLOW);

    for (int angle = 30; angle <= 150; angle += 30) {
        turnRightDegrees(30, SPEED_TURN);
        delay(100);
        readSensors();
        if (isRed()) {
            debugPrint("Found red path after turn");
            redSubState = RED_FOLLOW_PATH;
            return;
        }
    }

    turnLeftDegrees(180, SPEED_TURN);

    for (int angle = 30; angle <= 150; angle += 30) {
        turnLeftDegrees(30, SPEED_TURN);
        delay(100);
        readSensors();
        if (isRed()) {
            debugPrint("Found red path after turn");
            redSubState = RED_FOLLOW_PATH;
            return;
        }
    }

    debugPrint("Cannot find red path - returning home");
    currentState = STATE_RETURN_HOME;
}

// =====================
// RETURN HOME
// =====================
void handleReturnHome() {
    if (!returnStarted) {
        debugPrint("=== RETURNING HOME ===");
        returnStarted = true;

        turnRightDegrees(180, SPEED_TURN);
        delay(300);
    }

    readSensors();

    if (obstacleAhead()) {
        avoidObstacle();
        return;
    }

    if (isWhite()) {
        debugPrint("Start zone detected!");
        stopMotors();
        currentState = STATE_COMPLETE;
        return;
    }

    followLineWithColor();

    if (millis() - stateStartTime > 60000) {
        debugPrint("Return timeout - stopping");
        stopMotors();
        currentState = STATE_COMPLETE;
    }
}

void handleComplete() {
    stopMotors();

    unsigned long totalTime = millis() - missionStartTime;
    Serial.println("=== MISSION COMPLETE ===");
    Serial.print("Total time: ");
    Serial.print(totalTime / 1000);
    Serial.println(" seconds");

    armDown();
    clawOpen();

    while (true) {
        delay(1000);
    }
}

void handleError() {
    stopMotors();
    debugPrint("ERROR STATE - attempting recovery");

    recoveryAttempts++;

    if (recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
        backup(500, SPEED_SLOW);
        turnRightDegrees(90, SPEED_TURN);

        if (searchForLine(SEARCH_TIMEOUT_MS)) {
            currentState = previousState;
            debugPrint("Recovery successful");
        } else {
            debugPrint("Recovery failed - returning home");
            currentState = STATE_RETURN_HOME;
        }
    } else {
        debugPrint("Max recovery attempts - giving up");
        currentState = STATE_RETURN_HOME;
    }
}

// =====================
// MAIN SETUP AND LOOP
// =====================
void setup() {
    Serial.begin(9600);
    delay(1000);

    Serial.println("UTRA Biathlon Robot Starting...");
    Serial.println("Press any key to begin mission or wait 3 seconds");

    unsigned long waitStart = millis();
    while (millis() - waitStart < 3000) {
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            break;
        }
        delay(100);
    }

    currentState = STATE_INIT;
}

void loop() {
    if (millis() - missionStartTime > MISSION_TIMEOUT_MS && missionStartTime > 0) {
        debugPrint("MISSION TIMEOUT");
        currentState = STATE_COMPLETE;
    }

    previousState = currentState;

    switch (currentState) {
        case STATE_INIT:
            handleInit();
            break;

        case STATE_PICKUP_BOX:
            handlePickupBox();
            break;

        case STATE_CARRY_TO_ZONE:
            handleCarryToZone();
            break;

        case STATE_DROP_BOX:
            handleDropBox();
            break;

        case STATE_DETECT_PATH:
            handleDetectPath();
            break;

        case STATE_GREEN_PATH:
            handleGreenPath();
            break;

        case STATE_RED_PATH:
            handleRedPath();
            break;

        case STATE_RETURN_HOME:
            handleReturnHome();
            break;

        case STATE_COMPLETE:
            handleComplete();
            break;

        case STATE_ERROR:
            handleError();
            break;

        default:
            currentState = STATE_ERROR;
            break;
    }

    debugState();
    delay(50);
}
