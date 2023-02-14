#include "AS5601.h"
#include "debounce.h"
#include <USBComposite.h>

//#define DEBUG

#define RESCALE 1200

#if !defined(RESCALE)
# define SCALE(x) ((uint32_t)(x))
#else
# define SCALE(x) ((uint32_t)(x)*(RESCALE-1)/4095)
#endif

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
//HIDMouse Mouse(HID); 
uint32_t prev = 0xFFFFFFFF;
const uint32_t LED = PC13;

const uint32_t mask = 0xFFF;
const uint32_t signBit = 0x800;

const int32_t hysteresis = 1;

// mouse allowing 12 bit movement
#define HID_FAST_MOUSE_REPORT_DESCRIPTOR(...) \
    0x05, 0x01,            /*  USAGE_PAGE (Generic Desktop)  // 54 */ \
    0x09, 0x02,           /*  USAGE (Mouse) */ \
    0xa1, 0x01,           /*  COLLECTION (Application) */ \
    0x85, MACRO_GET_ARGUMENT_1_WITH_DEFAULT(HID_MOUSE_REPORT_ID, ## __VA_ARGS__),  /*    REPORT_ID */ \
    0x09, 0x01,           /*    USAGE (Pointer) */ \
    0xa1, 0x00,           /*    COLLECTION (Physical) */ \
    0x05, 0x09,           /*      USAGE_PAGE (Button) */ \
    0x19, 0x01,           /*      USAGE_MINIMUM (Button 1) */ \
    0x29, 0x08,           /*      USAGE_MAXIMUM (Button 8) */ \
    0x15, 0x00,           /*      LOGICAL_MINIMUM (0) */ \
    0x25, 0x01,           /*      LOGICAL_MAXIMUM (1) */ \
    0x95, 0x08,           /*      REPORT_COUNT (8) */ \
    0x75, 0x01,           /*      REPORT_SIZE (1) */ \
    0x81, 0x02,           /*      INPUT (Data,Var,Abs) */ \
    0x05, 0x01,           /*      USAGE_PAGE (Generic Desktop) */ \
    0x09, 0x30,           /*      USAGE (X) */ \
    0x09, 0x31,           /*      USAGE (Y) */ \
    0x16, 0x00, 0xF8,     /*      LOGICAL_MINIMUM (-2048) */ \
    0x26, 0xFF, 0x07,     /*      LOGICAL_MAXIMUM (2047) */ \
    0x75, 12 ,           /*       REPORT_SIZE (12) */ \
    0x95, 0x02,           /*      REPORT_COUNT (2) */ \
    0x81, 0x06,           /*      INPUT (Data,Var,Rel) */ \
    0xc0,                 /*    END_COLLECTION */ \
    MACRO_ARGUMENT_2_TO_END(__VA_ARGS__)  \
    0xc0                  /*  END_COLLECTION */ 

struct FastMouseReport_t {
    uint8_t reportID;
    uint32_t buttons:8;
    int32_t dx:12;
    int32_t dy:12;
} __packed;

uint8_t fastMouseDescriptor[] = {HID_FAST_MOUSE_REPORT_DESCRIPTOR(HID_MOUSE_REPORT_ID)};

HIDReportDescriptor fastMouse = {
  fastMouseDescriptor, sizeof(fastMouseDescriptor)
};

class HIDFastMouse : public HIDReporter {
protected:
    FastMouseReport_t reportBuffer;
    uint8_t buttons;
public:
  HIDFastMouse(USBHID& HID, uint8_t reportID=HID_MOUSE_REPORT_ID) : HIDReporter(HID, &fastMouse, (uint8_t*)&reportBuffer, sizeof(reportBuffer), reportID) {
      buttons = 0;
      reportBuffer.buttons = 0;
      reportBuffer.dx = 0;
      reportBuffer.dy = 0;
    }
  void begin(void);
  void end(void);
  void click(uint8_t b = MOUSE_LEFT);
  void move(int16_t x, int16_t y) {
    reportBuffer.dx = x;
    reportBuffer.dy = y;
    sendReport();
  }
  void press(uint8_t b = MOUSE_LEFT) {
    reportBuffer.buttons |= b;
    move(0,0);
  }
  void release(uint8_t b = MOUSE_RIGHT) {
    reportBuffer.buttons &= ~b;
    move(0,0);
  }
};

HIDFastMouse Mouse(HID);


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
    //Sensor.writeRaw8(0x07,0); // 0b11011); // Fast Filter 110, Slow Filter 11
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
        uint32_t value = SCALE(Sensor.getRawAngleFiltered());

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
#ifdef RESCALE
          int32_t delta = (value - prev + RESCALE) % RESCALE;
          if (delta >= RESCALE/2)
            delta -= RESCALE; 
#else
          int32_t delta = (value - prev) & mask;
          if (delta & signBit)
            delta |= ~mask;
#endif            
          if (delta > hysteresis || delta < -hysteresis) {
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
