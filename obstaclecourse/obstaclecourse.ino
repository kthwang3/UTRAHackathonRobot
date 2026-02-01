
// Left motor
#define ENA 9
#define IN1 8
#define IN2 7

// Right motor
#define ENB 4
#define IN3 6
#define IN4 5

/***********************
  ULTRASONIC PINS (PLACEHOLDERS)
  Sensor pins: VCC TRIG ECHO OUT GND
  - Use TRIG + ECHO for distance
  - Ignore OUT
************************/
#define US_TRIG 3
#define US_ECHO 2

/***********************
  TCS230 COLOR SENSOR PINS (PLACEHOLDERS)
  Typical pins: S0 S1 S2 S3 OUT VCC GND (sometimes OE)
************************/
#define TCS_S0   A1
#define TCS_S1   A0  // can be any digital-capable pin; A0 works as digital on Uno
#define TCS_S2   12
#define TCS_S3   11
#define TCS_OUT  13  

/***********************
  TUNING
************************/
const int BASE_SPEED   = 255;  // forward speed (0–255)
const int STEER_BOOST  = 70;   // steering amount
const float ON_RED     = 0.45; // redRatio >= ON_RED => confidently on tape
const float LOST_RED   = 0.40; // redRatio <= LOST_RED => lost tape => search

const int OBSTACLE_CM  = 15;   // obstacle distance threshold in cm

/***********************
  SETUP
************************/
void setup() {
  // Motors
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  // Ultrasonic
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  // TCS230
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);

  // TCS230 scaling: S0=HIGH, S1=LOW => 20% (usually stable)
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  Serial.begin(115200);
  Serial.println("Robot: Ultrasonic + TCS230 red-line follow");
}

/***********************
  LOOP
************************/
void loop() {
  // 1) Obstacle override
  float d = getDistanceCM();
  if (d > 0 && d < OBSTACLE_CM) {
    avoidObstacle();
    return; // skip line-follow this cycle
  }

  // 2) Line-follow using TCS230 (red tape)
  uint16_t R, G, B;
  readRGB(R, G, B);

  // (Optional debug)
  // Serial.print("d="); Serial.print(d);
  // Serial.print(" R="); Serial.print(R);
  // Serial.print(" G="); Serial.print(G);
  // Serial.print(" B="); Serial.print(B);
  // Serial.print(" rr="); Serial.println(redRatio(R,G,B), 3);

  followRedTape(R, G, B);

  delay(10);
}

/***********************
  MOTOR HELPERS
************************/
void setLeft(bool forwardDir, int spd) {
  digitalWrite(IN1, forwardDir ? HIGH : LOW);
  digitalWrite(IN2, forwardDir ? LOW : HIGH);
  analogWrite(ENA, constrain(spd, 0, 255));
}

void setRight(bool forwardDir, int spd) {
  digitalWrite(IN3, forwardDir ? HIGH : LOW);
  digitalWrite(IN4, forwardDir ? LOW : HIGH);
  analogWrite(ENB, constrain(spd, 0, 255));
}

// + = forward, - = backward
void setMotors(int leftSpd, int rightSpd) {
  if (leftSpd >= 0) setLeft(true, leftSpd);
  else setLeft(false, -leftSpd);

  if (rightSpd >= 0) setRight(true, rightSpd);
  else setRight(false, -rightSpd);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

/***********************
  ULTRASONIC
************************/
float getDistanceCM() {
  // Trigger pulse
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  // Read echo time
  unsigned long duration = pulseIn(US_ECHO, HIGH, 25000); // 25ms timeout ~4m
  if (duration == 0) return -1; // no reading

  // Convert to cm
  return (duration * 0.0343f) / 2.0f;
}

void avoidObstacle() {
  // Simple: stop -> back up -> turn -> go
  stopMotors();
  delay(150);

  setMotors(-130, -130); // back up
  delay(350);

  setMotors(150, -150);  // pivot right
  delay(300);

  stopMotors();
  delay(100);
}

/***********************
  TCS230 READING
************************/
// Filter select:
// S2 S3
// 0  0  -> Red
// 1  1  -> Green
// 0  1  -> Blue
// 1  0  -> Clear (not used)
void setFilterRed()   { digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, LOW); }
void setFilterGreen() { digitalWrite(TCS_S2, HIGH); digitalWrite(TCS_S3, HIGH); }
void setFilterBlue()  { digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, HIGH); }

uint16_t readColorIntensity() {
  // Measure LOW pulse width; invert to get an "intensity-like" number
  unsigned long t = pulseIn(TCS_OUT, LOW, 30000);
  if (t == 0) return 0;
  return (uint16_t)constrain(1000000UL / t, 0UL, 65535UL);
}

void readRGB(uint16_t &R, uint16_t &G, uint16_t &B) {
  setFilterRed();
  delay(3);
  R = readColorIntensity();

  setFilterGreen();
  delay(3);
  G = readColorIntensity();

  setFilterBlue();
  delay(3);
  B = readColorIntensity();
}

float redRatio(uint16_t R, uint16_t G, uint16_t B) {
  float sum = (float)R + (float)G + (float)B;
  if (sum < 1.0f) sum = 1.0f;
  return (float)R / sum;
}

/***********************
  RED TAPE FOLLOWER
************************/
bool followRedTape(uint16_t R, uint16_t G, uint16_t B) {
  static bool searchLeft = true;

  float rr = redRatio(R, G, B);

  if (rr >= ON_RED) {
    // On tape: straight
    setMotors(BASE_SPEED, BASE_SPEED);
    return true;
  }

  if (rr <= LOST_RED) {
    // Lost tape: search by curving left/right
    if (searchLeft) {
      setMotors(BASE_SPEED - STEER_BOOST, BASE_SPEED + STEER_BOOST);
    } else {
      setMotors(BASE_SPEED + STEER_BOOST, BASE_SPEED - STEER_BOOST);
    }
    searchLeft = !searchLeft;
    return false;
  }

  // In-between: keep going to reduce jitter
  setMotors(BASE_SPEED, BASE_SPEED);
  return false;
}
*/