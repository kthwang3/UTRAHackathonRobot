#include "ColorSensor.h"

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

void setup() {
    Serial.begin(9600);
    colorSensorSetup();
}

void loop() {
    colorSensorRead(redPW, greenPW, bluePW);

    Serial.print("Red PW = ");
    Serial.print(redPW);
    Serial.print(" - Green PW = ");
    Serial.print(greenPW);
    Serial.print(" - Blue PW = ");
    Serial.println(bluePW);
}