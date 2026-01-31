#include <Servo.h>

Servo myServo;   // create servo object

void setup() {
  myServo.attach(9);  // attach servo to pin 9
}

void loop() {
  // Move from 0 to 180
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(15);   // wait for servo to reach position
  }

  // Move from 180 to 0
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(15);
  }
}