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
    STATE_RETURN_HOME,
    STATE_COMPLETE,
    STATE_ERROR
};

MissionState currentState = STATE_INIT;
MissionState previousState = STATE_INIT;

// =====================
// SUB-STATES
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
// MOTOR SPEEDS - ACTIVE TUNING AREA
// =====================
const int SPEED_FORWARD = 180;      // Main forward speed
const int SPEED_SLOW = 120;         // Careful movement
const int SPEED_TURN = 150;         // Turning speed
const int SPEED_SEARCH = 100;       // Slow search speed
const int SPEED_RAMP = 200;         // Climbing ramp

// =====================
// COLOR THRESHOLDS - ACTIVE TUNING AREA
// Lower PW = stronger color detection on TCS3200
// =====================
const int BLACK_PW_MIN = 800;       // Black has HIGH PW (dark = less reflection)
const int WHITE_PW_MAX = 300;       // White has LOW PW (bright = more reflection)
const int COLOR_DIFF_THRESHOLD = 80; // How much one channel must dominate

// =====================
// TIMING - ACTIVE TUNING AREA
// =====================
const int TURN_90_MS = 450;         // Time for 90-degree turn (tune for your robot!)
const int SEARCH_STEP_MS = 150;     // Each search turn step
const int FORWARD_BURST_MS = 200;   // Forward movement burst
const int SEARCH_TIMEOUT_MS = 2000; // Give up searching
const unsigned long MISSION_TIMEOUT_MS = 300000; // 5 minutes

// =====================
// DISTANCE THRESHOLDS
// =====================
const int OBSTACLE_CLOSE = 12;

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
// LINE FOLLOWING STATE
// =====================
int searchDirection = 1;            // 1 = right, -1 = left
int searchSteps = 0;                // How many search steps taken
unsigned long lastLineTime = 0;     // When we last saw the line
bool onLine = false;                // Are we currently on a line?

// =====================
// TIMING
// =====================
unsigned long missionStartTime = 0;
unsigned long stateStartTime = 0;

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

void debugSensors() {
    if (debugMode) {
        Serial.print("R:");
        Serial.print(redPW);
        Serial.print(" G:");
        Serial.print(greenPW);
        Serial.print(" B:");
        Serial.print(bluePW);
        Serial.print(" D:");
        Serial.print(distance);
        Serial.print(" | ");

        // Print detected color
        if (isOnBlackLine()) Serial.print("BLACK ");
        if (isOnRedLine()) Serial.print("RED ");
        if (isOnGreenLine()) Serial.print("GREEN ");
        if (isOnBlueSurface()) Serial.print("BLUE ");
        if (isOnWhiteSurface()) Serial.print("WHITE ");
        Serial.println();
    }
}

// =====================
// COLOR DETECTION - Simplified and robust
// =====================
void readSensors() {
    colorSensorRead(redPW, greenPW, bluePW);
    distance = getDistanceCM();
}

// Black line = all channels have HIGH pulse width (dark surface reflects less)
bool isOnBlackLine() {
    return (redPW > BLACK_PW_MIN && greenPW > BLACK_PW_MIN && bluePW > BLACK_PW_MIN);
}

// White surface = all channels have LOW pulse width (bright surface reflects more)
bool isOnWhiteSurface() {
    return (redPW < WHITE_PW_MAX && greenPW < WHITE_PW_MAX && bluePW < WHITE_PW_MAX);
}

// Red line = red channel significantly lower than others (stronger red)
bool isOnRedLine() {
    return (redPW < greenPW - COLOR_DIFF_THRESHOLD &&
            redPW < bluePW - COLOR_DIFF_THRESHOLD &&
            redPW < 600);  // Also check it's reasonably strong
}

// Green line = green channel significantly lower than others
bool isOnGreenLine() {
    return (greenPW < redPW - COLOR_DIFF_THRESHOLD &&
            greenPW < bluePW - COLOR_DIFF_THRESHOLD &&
            greenPW < 600);
}

// Blue surface = blue channel significantly lower than others
bool isOnBlueSurface() {
    return (bluePW < redPW - COLOR_DIFF_THRESHOLD &&
            bluePW < greenPW - COLOR_DIFF_THRESHOLD &&
            bluePW < 600);
}

// Are we on ANY followable line?
bool isOnAnyLine() {
    return isOnBlackLine() || isOnRedLine() || isOnGreenLine();
}

// =====================
// MOVEMENT HELPERS
// =====================
void goForward(int speed) {
    moveForward(speed);
}

void goBackward(int speed, int ms) {
    moveBackward(speed);
    delay(ms);
    stopMotors();
}

void turnLeftTime(int speed, int ms) {
    turnLeft(speed);
    delay(ms);
    stopMotors();
}

void turnRightTime(int speed, int ms) {
    turnRight(speed);
    delay(ms);
    stopMotors();
}

void turn180() {
    turnRightTime(SPEED_TURN, TURN_90_MS * 2);
}

// =====================
// SMART LINE FOLLOWING
// This is the core fix - moves forward by default, searches intelligently
// =====================
void smartLineFollow() {
    readSensors();

    // Check for obstacles first
    if (distance > 0 && distance < OBSTACLE_CLOSE) {
        debugPrint("Obstacle!");
        avoidObstacle();
        return;
    }

    // Are we on a line?
    if (isOnAnyLine()) {
        // YES - we found/are on a line
        onLine = true;
        lastLineTime = millis();
        searchSteps = 0;

        // Just go forward!
        goForward(SPEED_FORWARD);

        if (isOnBlackLine()) {
            // debugPrint("Following BLACK");
        } else if (isOnRedLine()) {
            // debugPrint("Following RED");
        } else if (isOnGreenLine()) {
            // debugPrint("Following GREEN");
        }
    } else {
        // NOT on a line - need to search
        unsigned long timeSinceLine = millis() - lastLineTime;

        if (timeSinceLine < 300) {
            // Just lost the line - keep going forward briefly
            // The line might just be a small gap
            goForward(SPEED_SLOW);
        } else if (timeSinceLine < SEARCH_TIMEOUT_MS) {
            // Lost for a bit - do a search pattern
            stopMotors();
            delay(50);

            // Alternate left-right search, increasing arc
            searchSteps++;
            int turnTime = SEARCH_STEP_MS * ((searchSteps / 2) + 1);
            turnTime = min(turnTime, 500); // Cap at 500ms

            if (searchDirection > 0) {
                turnRight(SPEED_SEARCH);
            } else {
                turnLeft(SPEED_SEARCH);
            }
            delay(turnTime);
            stopMotors();

            // Check if we found the line
            readSensors();
            if (isOnAnyLine()) {
                onLine = true;
                lastLineTime = millis();
                searchSteps = 0;
                return;
            }

            // Flip search direction
            searchDirection = -searchDirection;

        } else {
            // Lost for too long - move forward and try again
            debugPrint("Search timeout - moving forward");
            goForward(SPEED_SLOW);
            delay(FORWARD_BURST_MS);
            stopMotors();
            lastLineTime = millis(); // Reset timer
            searchSteps = 0;
        }
    }
}

// =====================
// OBSTACLE AVOIDANCE
// =====================
void avoidObstacle() {
    debugPrint("Avoiding obstacle");
    stopMotors();
    delay(100);

    // Back up
    goBackward(SPEED_SLOW, 400);
    delay(100);

    // Turn right
    turnRightTime(SPEED_TURN, TURN_90_MS);
    delay(100);

    // Go around
    goForward(SPEED_SLOW);
    delay(500);
    stopMotors();

    // Turn back toward original direction
    turnLeftTime(SPEED_TURN, TURN_90_MS / 2);

    // Continue forward
    goForward(SPEED_SLOW);
    delay(600);
    stopMotors();

    // Turn to rejoin path
    turnLeftTime(SPEED_TURN, TURN_90_MS / 2);

    lastLineTime = millis();
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

    // Start with claw open and arm down
    clawOpen();
    delay(300);
    armDown();
    delay(500);

    lastLineTime = millis();
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
        lastLineTime = millis();
    }
}

void handleCarryToZone() {
    readSensors();
    debugSensors();

    // Check if we reached the drop zone (white or blue)
    if (isOnWhiteSurface() || isOnBlueSurface()) {
        debugPrint("Drop zone detected!");
        stopMotors();
        delay(200);
        currentState = STATE_DROP_BOX;
        stateStartTime = millis();
        return;
    }

    // Follow the line to the zone
    smartLineFollow();

    // Timeout failsafe
    if (millis() - stateStartTime > 45000) {
        debugPrint("Carry timeout - dropping anyway");
        stopMotors();
        currentState = STATE_DROP_BOX;
    }
}

void handleDropBox() {
    if (!boxDropped) {
        debugPrint("Dropping box");
        stopMotors();

        // Back up slightly
        goBackward(SPEED_SLOW, 200);
        delay(200);

        // Release
        armDown();
        delay(400);
        clawOpen();
        delay(300);
        armUp();
        delay(300);

        boxDropped = true;

        // Move forward past the drop zone
        goForward(SPEED_SLOW);
        delay(500);
        stopMotors();

        currentState = STATE_DETECT_PATH;
        stateStartTime = millis();
        lastLineTime = millis();
    }
}

void handleDetectPath() {
    readSensors();
    debugSensors();

    // Move forward slowly while looking for colored paths
    goForward(SPEED_SLOW);
    delay(100);
    stopMotors();

    readSensors();

    if (isOnRedLine()) {
        debugPrint("RED path - Obstacle Course");
        pathChosen = true;
        currentState = STATE_RED_PATH;
        redSubState = RED_FOLLOW_PATH;
        stateStartTime = millis();
        lastLineTime = millis();
        return;
    }

    if (isOnGreenLine()) {
        debugPrint("GREEN path - Target Shooting");
        pathChosen = true;
        currentState = STATE_GREEN_PATH;
        greenSubState = GREEN_CLIMB_RAMP;
        stateStartTime = millis();
        lastLineTime = millis();
        return;
    }

    // Keep searching
    if (millis() - stateStartTime > 15000) {
        debugPrint("Path timeout - defaulting to GREEN path");
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
    debugSensors();

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
    if (obstacleAhead()) {
        avoidObstacle();
        return;
    }

    // Climb at full speed
    goForward(SPEED_RAMP);

    // Check if we've climbed enough (time-based for now)
    static unsigned long climbStart = 0;
    if (climbStart == 0) climbStart = millis();

    if (millis() - climbStart > 4000) {
        debugPrint("Ramp climb complete");
        stopMotors();
        delay(300);
        greenSubState = GREEN_NAVIGATE_RINGS;
        climbStart = 0;
    }
}

void handleNavigateRings() {
    readSensors();

    // Navigate through concentric rings toward black center
    if (isOnBlackLine()) {
        debugPrint("BLACK center found!");
        stopMotors();
        greenSubState = GREEN_FIND_BLACK_CENTER;
        return;
    }

    if (isOnBlueSurface()) {
        debugPrint("On BLUE ring");
        goForward(SPEED_SLOW);
        delay(300);
    } else if (isOnRedLine()) {
        debugPrint("On RED ring");
        goForward(SPEED_SLOW);
        delay(300);
    } else {
        // Spiral inward pattern
        static int spiralStep = 0;
        spiralStep++;

        if (spiralStep % 2 == 0) {
            turnLeftTime(SPEED_SEARCH, 100);
        }
        goForward(SPEED_SLOW);
        delay(200);
        stopMotors();
    }

    if (millis() - stateStartTime > 25000) {
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

        // Turn toward blue zone
        turnRightTime(SPEED_TURN, TURN_90_MS / 2);
        delay(200);

        // Launch!
        armDown();
        delay(200);
        clawOpen();
        delay(300);
        armUp();

        ballLaunched = true;
        greenSubState = GREEN_DESCEND_RAMP;
        stateStartTime = millis();
    }
}

void handleDescendRamp() {
    debugPrint("Descending ramp");

    turn180();
    delay(200);

    goForward(SPEED_SLOW);
    delay(4000);
    stopMotors();

    currentState = STATE_RETURN_HOME;
    stateStartTime = millis();
    lastLineTime = millis();
}

// =====================
// RED PATH: OBSTACLE COURSE
// =====================
void handleRedPath() {
    readSensors();
    debugSensors();

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

    // Check for black obstacles (not black lines - need to distinguish)
    // For now, just avoid if ultrasonic sees something

    if (isOnRedLine()) {
        goForward(SPEED_FORWARD);
        lastLineTime = millis();
    } else {
        // Lost red line - search
        unsigned long timeLost = millis() - lastLineTime;

        if (timeLost < 500) {
            goForward(SPEED_SLOW);
        } else if (timeLost < 3000) {
            redSubState = RED_SHARP_TURN;
        } else {
            debugPrint("Red path lost - continuing");
            currentState = STATE_RETURN_HOME;
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

    avoidObstacle();

    // Try to find red line again
    if (isOnRedLine()) {
        redSubState = RED_FOLLOW_PATH;
    } else {
        redSubState = RED_SHARP_TURN;
    }
}

void handleRedSharpTurn() {
    debugPrint("Searching for red path");

    stopMotors();
    delay(100);

    // Scan right
    for (int i = 0; i < 6; i++) {
        turnRightTime(SPEED_SEARCH, 100);
        readSensors();
        if (isOnRedLine()) {
            debugPrint("Found red path");
            redSubState = RED_FOLLOW_PATH;
            lastLineTime = millis();
            return;
        }
    }

    // Return to center and scan left
    turnLeftTime(SPEED_SEARCH, 600);

    for (int i = 0; i < 6; i++) {
        turnLeftTime(SPEED_SEARCH, 100);
        readSensors();
        if (isOnRedLine()) {
            debugPrint("Found red path");
            redSubState = RED_FOLLOW_PATH;
            lastLineTime = millis();
            return;
        }
    }

    // Can't find it - move forward and try again
    turnRightTime(SPEED_SEARCH, 300); // Return to roughly center
    goForward(SPEED_SLOW);
    delay(400);
    stopMotors();

    lastLineTime = millis();
    redSubState = RED_FOLLOW_PATH;
}

// =====================
// RETURN HOME
// =====================
void handleReturnHome() {
    if (!returnStarted) {
        debugPrint("=== RETURNING HOME ===");
        returnStarted = true;
        turn180();
        delay(300);
        lastLineTime = millis();
    }

    readSensors();

    // Check if we're back at start (white zone)
    if (isOnWhiteSurface()) {
        debugPrint("Start zone found!");
        stopMotors();
        currentState = STATE_COMPLETE;
        return;
    }

    smartLineFollow();

    if (millis() - stateStartTime > 60000) {
        debugPrint("Return timeout");
        stopMotors();
        currentState = STATE_COMPLETE;
    }
}

void handleComplete() {
    stopMotors();

    unsigned long totalTime = millis() - missionStartTime;
    Serial.println("=== MISSION COMPLETE ===");
    Serial.print("Time: ");
    Serial.print(totalTime / 1000);
    Serial.println("s");

    armDown();
    clawOpen();

    while (true) {
        delay(1000);
    }
}

void handleError() {
    stopMotors();
    debugPrint("ERROR - recovering");

    recoveryAttempts++;

    if (recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
        goBackward(SPEED_SLOW, 300);
        turnRightTime(SPEED_TURN, TURN_90_MS);
        lastLineTime = millis();
        currentState = previousState;
    } else {
        currentState = STATE_RETURN_HOME;
    }
}

// =====================
// SETUP AND LOOP
// =====================
void setup() {
    Serial.begin(9600);
    delay(1000);

    Serial.println("UTRA Biathlon Robot");
    Serial.println("Starting in 2 seconds...");
    delay(2000);

    currentState = STATE_INIT;
}

void loop() {
    // Mission timeout
    if (missionStartTime > 0 && millis() - missionStartTime > MISSION_TIMEOUT_MS) {
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

    delay(30);
}
