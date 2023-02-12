#include "AS5601.h"
#include <USBComposite.h>

// TODO: debounce buttons

// SDA1: PB7
// SCL1: PB6

/*
 8   7   6   5
 A  SCL SDA  B 
 +------------+
 |  AS5601    |
 |o           |
 +------------+
5V 3V3 PUSH GND
 1   2   3   4
*/

// SDA2: PB11
// SCL2: PB10

AS5601 Sensor;

void setup() {
    Serial.begin();
}

void loop() {
    if (Sensor.magnetDetected()) {
        Serial.println(Sensor.getAngle());
    }
}
