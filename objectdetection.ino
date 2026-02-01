#define TRIG_PIN 3
#define ECHO_PIN 2


// Left motor
#define ENA 9
#define IN1 8
#define IN2 7

// Right motor
#define ENB 4
#define IN3 6
#define IN4 5

enum RobotState {
    DRIVE_FORWARD,
    TURN_1,
    ARC_FORWARD,
    TURN_2
};

RobotState state = DRIVE_FORWARD;
unsigned long stateStartTime = 0;


const int DRIVE_SPEED = 150;  // normal forward speed
const int TURN_SPEED  = 120;  // turning speed

const int TURN_TIME = 600;    // ms for ~90° turn 
const int ARC_TIME  = 1200;   // ms to drive around box 

void setup() {
    Serial.begin(9600);

    // Ultrasonic sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Motor pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void drive(int leftSpeed, int rightSpeed) {
    // ----- LEFT MOTOR -----
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -leftSpeed);
    }

    // ----- RIGHT MOTOR -----
    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, -rightSpeed);
    }
}

long getDistanceCM() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout
    if (duration == 0) return -1;  // no object detected

    return duration * 0.034 / 2;   // convert to cm
}

bool objectDetected() {
    long distance = getDistanceCM();

    if (distance == -1) return false;

    Serial.print("Distance: ");
    Serial.println(distance);

    return distance < 20; // object detected if closer than 20 cm
}

// ================= MAIN LOOP =================
void loop() {
    switch (state) {

        case DRIVE_FORWARD:
            drive(DRIVE_SPEED, DRIVE_SPEED);

            if (objectDetected()) {
                drive(0, 0); // stop briefly before turning
                state = TURN_1;
                stateStartTime = millis();
            }
            break;

        case TURN_1:
            drive(TURN_SPEED, -TURN_SPEED); // turn left in place

            if (millis() - stateStartTime > TURN_TIME) {
                state = ARC_FORWARD;
                stateStartTime = millis();
            }
            break;

        case ARC_FORWARD:
            // smooth curve: left wheel faster than right
            drive(160, 100);

            if (millis() - stateStartTime > ARC_TIME) {
                state = TURN_2;
                stateStartTime = millis();
            }
            break;

        case TURN_2:
            drive(TURN_SPEED, -TURN_SPEED); // turn left again

            if (millis() - stateStartTime > TURN_TIME) {
                state = DRIVE_FORWARD; // resume straight driving
            }
            break;
    }
}