// Component Test Sketch
// Test each component individually before running the full mission

#include "../ColorSensor.h"
#include "../ultraSensor.h"
#include "../IRSensor.h"
#include "../moveforward.h"
#include "../ServoDriver.h"

enum TestMode {
    TEST_IDLE,
    TEST_MOTORS_FWD,
    TEST_MOTORS_BWD,
    TEST_MOTORS_LEFT,
    TEST_MOTORS_RIGHT,
    TEST_LINE_FOLLOW,
    TEST_OBSTACLE_AVOID,
    TEST_COLOR_TRACK_RED,
    TEST_COLOR_TRACK_GREEN
};

TestMode currentTest = TEST_IDLE;
int redPW, greenPW, bluePW;

void setup() {
    Serial.begin(9600);
    delay(1000);

    colorSensorSetup();
    ultrasonicSetup();
    irSensorSetup();
    motorSetup();
    initServos();

    armUp();
    clawOpen();

    Serial.println("=== COMPONENT TEST MODE ===");
    Serial.println("Commands:");
    Serial.println("  0 - Stop all / Idle");
    Serial.println("  1 - Motors forward");
    Serial.println("  2 - Motors backward");
    Serial.println("  3 - Turn left");
    Serial.println("  4 - Turn right");
    Serial.println("  5 - Line following (IR sensors)");
    Serial.println("  6 - Obstacle avoidance test");
    Serial.println("  7 - Track RED color");
    Serial.println("  8 - Track GREEN color");
    Serial.println("  g - Grab (close claw, arm up)");
    Serial.println("  r - Release (arm down, open claw)");
    Serial.println("");
}

void testLineFollow() {
    int pos = getLinePosition();

    if (pos == 0) {
        moveForward(180);
    } else if (pos < 0) {
        turnLeft(150);
    } else {
        turnRight(150);
    }
}

void testObstacleAvoid() {
    int dist = getDistanceCM();

    if (dist > 0 && dist < 15) {
        Serial.println("Obstacle detected!");
        stopMotors();
        delay(200);

        moveBackward(150);
        delay(400);
        stopMotors();

        turnRight(180);
        delay(500);
        stopMotors();
    } else {
        moveForward(150);
    }
}

void testColorTrack(bool trackRed) {
    colorSensorRead(redPW, greenPW, bluePW);

    bool onTarget;
    if (trackRed) {
        onTarget = (redPW < greenPW - 100 && redPW < bluePW - 100);
    } else {
        onTarget = (greenPW < redPW - 100 && greenPW < bluePW - 100);
    }

    if (onTarget) {
        moveForward(150);
    } else {
        static bool dir = true;
        if (dir) {
            turnLeft(120);
        } else {
            turnRight(120);
        }
        delay(80);
        stopMotors();
        dir = !dir;
    }
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        while (Serial.available()) Serial.read();

        stopMotors();

        switch (cmd) {
            case '0':
                currentTest = TEST_IDLE;
                Serial.println("IDLE - Motors stopped");
                break;

            case '1':
                currentTest = TEST_MOTORS_FWD;
                Serial.println("Motors FORWARD");
                break;

            case '2':
                currentTest = TEST_MOTORS_BWD;
                Serial.println("Motors BACKWARD");
                break;

            case '3':
                currentTest = TEST_MOTORS_LEFT;
                Serial.println("Turn LEFT");
                break;

            case '4':
                currentTest = TEST_MOTORS_RIGHT;
                Serial.println("Turn RIGHT");
                break;

            case '5':
                currentTest = TEST_LINE_FOLLOW;
                Serial.println("LINE FOLLOWING mode");
                break;

            case '6':
                currentTest = TEST_OBSTACLE_AVOID;
                Serial.println("OBSTACLE AVOIDANCE mode");
                break;

            case '7':
                currentTest = TEST_COLOR_TRACK_RED;
                Serial.println("Track RED color");
                break;

            case '8':
                currentTest = TEST_COLOR_TRACK_GREEN;
                Serial.println("Track GREEN color");
                break;

            case 'g':
            case 'G':
                Serial.println("GRAB");
                clawClose();
                delay(400);
                armUp();
                break;

            case 'r':
            case 'R':
                Serial.println("RELEASE");
                armDown();
                delay(400);
                clawOpen();
                break;
        }
    }

    switch (currentTest) {
        case TEST_IDLE:
            break;

        case TEST_MOTORS_FWD:
            moveForward(180);
            break;

        case TEST_MOTORS_BWD:
            moveBackward(180);
            break;

        case TEST_MOTORS_LEFT:
            turnLeft(150);
            break;

        case TEST_MOTORS_RIGHT:
            turnRight(150);
            break;

        case TEST_LINE_FOLLOW:
            testLineFollow();
            break;

        case TEST_OBSTACLE_AVOID:
            testObstacleAvoid();
            break;

        case TEST_COLOR_TRACK_RED:
            testColorTrack(true);
            break;

        case TEST_COLOR_TRACK_GREEN:
            testColorTrack(false);
            break;
    }

    // Print sensor status periodically
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        colorSensorRead(redPW, greenPW, bluePW);
        int dist = getDistanceCM();

        Serial.print("R:");
        Serial.print(redPW);
        Serial.print(" G:");
        Serial.print(greenPW);
        Serial.print(" B:");
        Serial.print(bluePW);
        Serial.print(" | Dist:");
        Serial.print(dist);
        Serial.print(" | IR L:");
        Serial.print(readIRLeft());
        Serial.print(" R:");
        Serial.println(readIRRight());

        lastPrint = millis();
    }

    delay(50);
}
