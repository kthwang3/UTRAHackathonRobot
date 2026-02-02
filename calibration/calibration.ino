// Sensor Calibration Utility
// Use this to find the correct threshold values for your environment

#include "../ColorSensor.h"
#include "../ultraSensor.h"
#include "../IRSensor.h"
#include "../moveforward.h"
#include "../ServoDriver.h"

int redPW, greenPW, bluePW;

void setup() {
    Serial.begin(9600);
    delay(1000);

    colorSensorSetup();
    ultrasonicSetup();
    irSensorSetup();
    motorSetup();
    initServos();

    Serial.println("=== SENSOR CALIBRATION ===");
    Serial.println("Commands:");
    Serial.println("  c - Read color sensor");
    Serial.println("  u - Read ultrasonic");
    Serial.println("  i - Read IR sensors");
    Serial.println("  a - Read all sensors");
    Serial.println("  s - Servo test (arm up/down)");
    Serial.println("  m - Motor test");
    Serial.println("  l - Continuous color logging (press any key to stop)");
    Serial.println("  h - Help");
    Serial.println("");
}

void printColorAnalysis() {
    colorSensorRead(redPW, greenPW, bluePW);

    Serial.println("--- COLOR SENSOR ---");
    Serial.print("Raw PW - R: ");
    Serial.print(redPW);
    Serial.print(" | G: ");
    Serial.print(greenPW);
    Serial.print(" | B: ");
    Serial.println(bluePW);

    // Analyze what color this likely is
    Serial.print("Detected: ");

    if (redPW > 1500 && greenPW > 1500) {
        Serial.println("BLACK (high PW = dark surface)");
    } else if (redPW < 400 && greenPW < 400 && bluePW < 400) {
        Serial.println("WHITE (low PW = bright surface)");
    } else if (redPW < greenPW - 100 && redPW < bluePW - 100) {
        Serial.println("RED (red channel strongest)");
    } else if (greenPW < redPW - 100 && greenPW < bluePW - 100) {
        Serial.println("GREEN (green channel strongest)");
    } else if (bluePW < redPW - 100 && bluePW < greenPW - 100) {
        Serial.println("BLUE (blue channel strongest)");
    } else {
        Serial.println("UNKNOWN / MIXED");
    }

    Serial.print("Suggested thresholds - BLACK_PW: ");
    Serial.print(max(redPW, greenPW) + 200);
    Serial.print(" | WHITE_PW_MAX: ");
    Serial.println(min(min(redPW, greenPW), bluePW) - 50);
    Serial.println("");
}

void printUltrasonic() {
    int dist = getDistanceCM();
    Serial.println("--- ULTRASONIC ---");
    Serial.print("Distance: ");
    if (dist < 0) {
        Serial.println("OUT OF RANGE");
    } else {
        Serial.print(dist);
        Serial.println(" cm");
    }
    Serial.println("");
}

void printIR() {
    Serial.println("--- IR SENSORS ---");
    Serial.print("Left: ");
    Serial.print(readIRLeft());
    Serial.print(" (");
    Serial.print(irLeftOnBlack() ? "BLACK" : "white");
    Serial.println(")");

    Serial.print("Right: ");
    Serial.print(readIRRight());
    Serial.print(" (");
    Serial.print(irRightOnBlack() ? "BLACK" : "white");
    Serial.println(")");

    Serial.print("Line position: ");
    Serial.println(getLinePosition());
    Serial.println("");
}

void testServos() {
    Serial.println("--- SERVO TEST ---");

    Serial.println("Arm UP...");
    armUp();
    delay(1000);

    Serial.println("Arm MID...");
    armMid();
    delay(1000);

    Serial.println("Arm DOWN...");
    armDown();
    delay(1000);

    Serial.println("Claw OPEN...");
    clawOpen();
    delay(1000);

    Serial.println("Claw CLOSE...");
    clawClose();
    delay(1000);

    Serial.println("Claw OPEN...");
    clawOpen();
    delay(500);

    Serial.println("Servo test complete");
    Serial.println("");
}

void testMotors() {
    Serial.println("--- MOTOR TEST ---");

    Serial.println("Forward...");
    moveForward(150);
    delay(500);
    stopMotors();
    delay(300);

    Serial.println("Backward...");
    moveBackward(150);
    delay(500);
    stopMotors();
    delay(300);

    Serial.println("Turn Left...");
    turnLeft(150);
    delay(500);
    stopMotors();
    delay(300);

    Serial.println("Turn Right...");
    turnRight(150);
    delay(500);
    stopMotors();
    delay(300);

    Serial.println("Motor test complete");
    Serial.println("");
}

void continuousColorLog() {
    Serial.println("--- CONTINUOUS COLOR LOG ---");
    Serial.println("Move sensor over different surfaces. Press any key to stop.");
    Serial.println("R\tG\tB\tDetected");

    while (!Serial.available()) {
        colorSensorRead(redPW, greenPW, bluePW);

        Serial.print(redPW);
        Serial.print("\t");
        Serial.print(greenPW);
        Serial.print("\t");
        Serial.print(bluePW);
        Serial.print("\t");

        if (redPW > 1500 && greenPW > 1500) {
            Serial.println("BLACK");
        } else if (redPW < 400 && greenPW < 400 && bluePW < 400) {
            Serial.println("WHITE");
        } else if (redPW < greenPW - 100 && redPW < bluePW - 100) {
            Serial.println("RED");
        } else if (greenPW < redPW - 100 && greenPW < bluePW - 100) {
            Serial.println("GREEN");
        } else if (bluePW < redPW - 100 && bluePW < greenPW - 100) {
            Serial.println("BLUE");
        } else {
            Serial.println("-");
        }

        delay(200);
    }

    while (Serial.available()) Serial.read();
    Serial.println("Logging stopped.");
    Serial.println("");
}

void printHelp() {
    Serial.println("=== CALIBRATION HELP ===");
    Serial.println("");
    Serial.println("RECOMMENDED CALIBRATION PROCESS:");
    Serial.println("1. Use 'l' (continuous log) while moving over surfaces");
    Serial.println("2. Note the R/G/B values for each color:");
    Serial.println("   - Black tape/line");
    Serial.println("   - White zone");
    Serial.println("   - Red path");
    Serial.println("   - Green path");
    Serial.println("   - Blue circle/zone");
    Serial.println("3. Update thresholds in UTRAHackathonRobot.ino:");
    Serial.println("   - BLACK_PW_THRESHOLD: Should be below black readings");
    Serial.println("   - WHITE_PW_MAX: Should be above white readings");
    Serial.println("   - RED/GREEN/BLUE_DOMINANCE: Difference needed to detect");
    Serial.println("");
    Serial.println("PIN ASSIGNMENTS:");
    Serial.println("  Color Sensor: S0=A1, S1=A0, S2=12, S3=11, OUT=10");
    Serial.println("  Ultrasonic: TRIG=2, ECHO=13");
    Serial.println("  IR Sensors: LEFT=A3, RIGHT=A4");
    Serial.println("  Motors: ENA=9, IN1=8, IN2=7, ENB=4, IN3=6, IN4=5");
    Serial.println("  Arm Servo: 3");
    Serial.println("  Claw Servo: A2");
    Serial.println("");
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        while (Serial.available()) Serial.read();

        switch (cmd) {
            case 'c':
            case 'C':
                printColorAnalysis();
                break;

            case 'u':
            case 'U':
                printUltrasonic();
                break;

            case 'i':
            case 'I':
                printIR();
                break;

            case 'a':
            case 'A':
                printColorAnalysis();
                printUltrasonic();
                printIR();
                break;

            case 's':
            case 'S':
                testServos();
                break;

            case 'm':
            case 'M':
                testMotors();
                break;

            case 'l':
            case 'L':
                continuousColorLog();
                break;

            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;

            default:
                Serial.println("Unknown command. Press 'h' for help.");
                break;
        }
    }

    delay(50);
}
