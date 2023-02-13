#include "AS5601.h"
#include "debounce.h"
#include <USBComposite.h>

//#define DEBUG

unsigned const b1 = PB11;
unsigned const b2 = PB10;
unsigned const b3 = PB1;
unsigned const b4 = PB0;
Debounce button1(b1, LOW);
Debounce button2(b2, LOW);
Debounce button3(b3, LOW);
Debounce button4(b4, LOW);
Debounce* buttons[] = { &button1, &button2, &button3, &button4 };

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


AS5601 Sensor;
USBCompositeSerial CompositeSerial;
USBHID HID;
HIDMouse Mouse(HID); 
uint32_t prev = 0xFFFFFFFF;
const uint32_t LED = PC13;

const uint32_t mask = 0xFFF;
const uint32_t signBit = 0x800;

const int32_t hysteresis = 1;

void setup() {
    pinMode(b1, INPUT_PULLUP);
    pinMode(b2, INPUT_PULLUP);
    pinMode(b3, INPUT_PULLUP);
    pinMode(b4, INPUT_PULLUP);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, 1);
    //Serial.begin();
    HID.setTXInterval(2);
    HID.begin(CompositeSerial);
    while (!USBComposite);
    Sensor.writeRaw8(0x07,0); // 0b11011); // Fast Filter 110, Slow Filter 11
#ifdef DEBUG    
    delay(2000);
    while (!CompositeSerial);
    CompositeSerial.println(Sensor.readRaw8(0x07),HEX);
#endif    
}

unsigned count = 0;
unsigned exactPrev = 0xFFFFFFFF;
unsigned lastZeroCross = 0;

void loop() {    
    if (Sensor.magnetDetected()) {
        digitalWrite(LED, 0);
        uint32_t value = Sensor.getAngleFiltered();
#ifdef DEBUG        
        if (value != exactPrev) {
//          CompositeSerial.println(value);
          if ((exactPrev & signBit) && !(value & signBit)) {
            uint32_t dt = millis()-lastZeroCross;
            if (dt != 0) {
              CompositeSerial.print("rpm:");
              CompositeSerial.println(60000/dt);
            }
            lastZeroCross=millis();
          }          
          exactPrev = value;
        }
#endif        

        if (prev == 0xFFFFFFFF) {
          prev = value;
        }
        else {
          int32_t delta = (value - prev) & mask;
          if (delta & signBit)
            delta |= ~mask;
#ifdef DEBUG
          if (delta<-10) {
            CompositeSerial.print(">");
          }
#endif            
          if (delta > hysteresis || delta < -hysteresis) {
            if (delta < -128) {
              do {
                Mouse.move(-128, 0);
                delta += 128;
              } while ( delta < -128 );
            }
            else if (delta > 127) {
              do {
                Mouse.move(127, 0);
                delta -= 127;
              } while( delta > 127 );
            }
            if (delta != 0)
              Mouse.move(delta,0);
            prev = value;
          }
        }
    }
    else {
        digitalWrite(LED, 1);
    }
    uint8_t mask = 1;
    for (unsigned i = 0 ; i < sizeof buttons / sizeof *buttons ; i++, mask <<= 1) {
      switch(buttons[i]->getEvent()) {
        case DEBOUNCE_PRESSED:
          Mouse.press(mask);
          break;
        case DEBOUNCE_RELEASED:
          Mouse.release(mask);
          break;
        default:
          break;
      }
    }
}
