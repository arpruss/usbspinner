#define STELLA_PRODUCT_ID 0xBEEF
#define STELLA_MFG_ID 0x04D8

const uint8_t gray[] = { 0b00, 0b01, 0b11, 0b10 };

#define TEN_BIT

#ifndef TEN_BIT
#define AXIS_MULT 1
const uint8_t stellaReportDescriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 1,                       /*    REPORT_ID */ // not present in official Stelladaptor
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1) /* byte 0 unused */
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x95, 0x02,                    //     REPORT_COUNT (2) /* byte 1 & 2 */
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //     END_COLLECTION
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x04,                    //   USAGE_MAXIMUM (Button 4)
    0x55, 0x00,                    //   UNIT_EXPONENT (0)
    0x65, 0x00,                    //   UNIT (None)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x04,                    //   REPORT_COUNT (4) /* byte 3 data bits */
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x04,                    //   REPORT_COUNT (4) /* byte 3 padding bits */
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0xc0                           // END_COLLECTION
};
#else
#define AXIS_MULT 4
uint8 stellaReportDescriptor[] = {
  0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
  0x15, 0x00,                    // LOGICAL_MINIMUM (0)
  0x09, 0x04,                    // USAGE (Joystick)
  0xa1, 0x01,                    // COLLECTION (Application)
  0x85, 1,                       /*    REPORT_ID */ // not present in official Stelladaptor
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x03,              //   LOGICAL_MAXIMUM (1023) // 255 for official Stelladaptor
  0x75, 0x0A,                    //   REPORT_SIZE (10) // 8 for official Stelladaptor
  //0x95, 0x01,                    //   REPORT_COUNT (1) /* byte 0 unused */ // official Stelladaptor
  //0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)  // official Stelladaptor
  0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
  0x09, 0x01,                    //   USAGE (Pointer)
  0xa1, 0x00,                    //   COLLECTION (Physical)
  0x09, 0x30,                    //     USAGE (X)
  0x09, 0x31,                    //     USAGE (Y)
  0x95, 0x02,                    //     REPORT_COUNT (2)
  0x81, 0x02,                    //     INPUT (Data,Var,Abs)
  0xc0,                          //     END_COLLECTION
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
  0x05, 0x09,                    //   USAGE_PAGE (Button)
  0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
  0x29, 0x04,                    //   USAGE_MAXIMUM (Button 4)
  0x55, 0x00,                    //   UNIT_EXPONENT (0)
  0x65, 0x00,                    //   UNIT (None)
  0x75, 0x01,                    //   REPORT_SIZE (1)
  0x95, 0x04,                    //   REPORT_COUNT (2)
  0x81, 0x02,                    //   INPUT (Data,Var,Abs)
  0xc0                           // END_COLLECTION
};
#endif

HIDReportDescriptor stellaDescriptor = {
  stellaReportDescriptor,
  sizeof(stellaReportDescriptor)
};

#ifdef TEN_BIT
typedef struct {
  uint8_t reportID;
  unsigned x: 10;
  unsigned y: 10;
  uint8_t buttons: 4;
} __packed StellaReport_t;
#else
typedef struct {
  uint8_t _reportID;
  uint8_t unused;
  uint8_t x;
  uint8_t y;
  uint8_t buttons: 4;
  uint8_t padding: 4;
} __packed StellaReport_t;
#endif

class HIDStella : public HIDReporter {
  public:
    StellaReport_t report;
    HIDStella(USBHID& HID, uint8_t reportID = HID_JOYSTICK_REPORT_ID)
      : HIDReporter(HID, &stellaDescriptor, (uint8_t*) & report, sizeof(report), HID_JOYSTICK_REPORT_ID) {
    }
};

HIDStella DrivingController(HID);



void stella_driving_setup() {
    USBComposite.setVendorId(0x04d8);
    USBComposite.setProductId(0xbeef);
    USBComposite.setManufacturerString("Grand Idea Studio");
    USBComposite.setProductString("Stelladaptor 2600-to-USB Interface");

    HID.clear();
    HID.setTXInterval(2);
    DrivingController.registerProfile();
    HID.begin();
    
    while (!USBComposite);
}

void stella_driving_loop() {
    static int startAngle = -1;
    bool force = false;
    StellaReport_t oldReport = DrivingController.report;
    
    if (Sensor.magnetDetected()) {
        digitalWrite(LED, 0);
        uint32_t value = Sensor.getRawAngle();
        if (startAngle == -1) {
          startAngle = value;
          force = true;
        }
        else {
          force = false;
        }
        value = ( (value - startAngle + 4096) % 4096 );
        uint8_t grayCode = gray[ (value / (4096/16)) % 4 ];
        uint8_t up = grayCode & 2;
        uint8_t down = grayCode & 1;
        if (up && down) {
          DrivingController.report.y = 0xC0*AXIS_MULT;
        }
        else if (up) {
          DrivingController.report.y = 0;
        }
        else if (down) {
          DrivingController.report.y = 0x100 * AXIS_MULT - 1;
        }
        else {
          DrivingController.report.y = 0x80 * AXIS_MULT;
        }
        DrivingController.report.x = 0x80 * AXIS_MULT;
    }

      for (unsigned i = 0 ; i < 4 ; i++) {
        switch(buttons[i]->getEvent()) {
          case DEBOUNCE_PRESSED:
            DrivingController.report.buttons |= (1<<i);
            break;
          case DEBOUNCE_RELEASED:
            DrivingController.report.buttons &= ~(1<<i);
            break;
        }
      }

      if (force || memcmp(&DrivingController.report,&oldReport,sizeof(DrivingController.report))) {
        DrivingController.sendReport();
      }
}

