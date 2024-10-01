/*

TinyIO

Input/output for DCC-EX

*/

#include <SPI.h>
#include <NRFLite.h>
#include <NmraDcc.h>
#include <EEPROM.h>


// Uncomment to ignore all serial comms
//#define NO_SERIAL

// Base address (0..255), the offset for the first input after a Factory Reset. The consecutive
// pins are numbered +1. This offset is added to 'EXIO_PIN_START' in the EXTIO controller to calculate
// the vpin number. Note: 'EXIO_PIN_START' + 'EXIO_DIGITAL_PIN_COUNT' is the highest vpin number!
// Note 2: if using DCC outputs all addresses from 0 to 255 will be valid.
#define STD_VPIN_BASE 0

// Standard pin function
#define STD_PIN_FUNCTION PINCONFIG_DIGITAL_INPUT

// Tiny pins
#define PIN_DCC 2
#define PIN_LED 3
#define PIN_RADIO_CE 0
#define PIN_RADIO_CSN 1

// Test pins (UNO)
/*
#define PIN_DCC 2
#define PIN_LED 13
#define PIN_RADIO_CE 9
#define PIN_RADIO_CSN 10
*/

// Mapping IO internal pin to hardware pin
#define PIN_IO1 7
#define PIN_IO2 8
#define PIN_IO3 9
#define PIN_IO4 10
#define PIN_IO5 PINCONFIG_NOT_USED
#define PIN_IO6 PINCONFIG_NOT_USED
#define PIN_IO7 PINCONFIG_NOT_USED
#define PIN_IO8 PINCONFIG_NOT_USED

// Ignored on Tiny's
#define PIN_IO9 PINCONFIG_NOT_USED
#define PIN_IO10 PINCONFIG_NOT_USED
#define PIN_IO11 PINCONFIG_NOT_USED
#define PIN_IO12 PINCONFIG_NOT_USED
#define PIN_IO13 PINCONFIG_NOT_USED
#define PIN_IO14 PINCONFIG_NOT_USED
#define PIN_IO15 PINCONFIG_NOT_USED
#define PIN_IO16 PINCONFIG_NOT_USED

// Nothing to configure below this

// Version
#define TS_VER "0.9.0"

// Arduino
#if defined(__AVR_ATmega328P__)
#define MAXPINS 16

#define PIN_LED 13
#define PIN_RADIO_CE 9
#define PIN_RADIO_CSN 10

#define Sbegin(a) (Serial.begin(a))
#define Sprintln(a) (Serial.println(a))
#define Sprint(a) (Serial.print(a))
#endif

// ATTINY
#if defined(__AVR_ATtiny84__) || (__AVR_ATtiny85__) || (SERIAL)
#define _BE_TINY_
#define MAXPINS 8

#define Sbegin(a)
#define Sprintln(a)
#define Sprint(a)
#endif

#if defined(NO_SERIAL)
#define Sbegin(a)
#define Sprintln(a)
#define Sprint(a)
#endif

// Pin configuration
#define PINCONFIG_NOT_USED 0
#define PINCONFIG_DIGITAL_INPUT 1
#define PINCONFIG_DIGITAL_OUTPUT 2
#define PINCONFIG_PWM_LED_OUTPUT 3
#define PINCONFIG_PWM_SERVO_OUTPUT 4
#define PINCONFIG_DCC_OUTPUT 5

// Radio packet types
#define PACKET_EMPTY 0
#define PACKET_INIT 1
#define PACKET_VPIN_CONFIG 2
#define PACKET_UPDATE_DIGITAL 3
#define PACKET_UPDATE_ANALOG 4
#define PACKET_JUST_SAY_HI 99

// What to show in the serial monitor besides basic info
#define SERIAL_SHOW_KEEPALIVE 0b00000001
#define SERIAL_SHOW_DIGITAL_INPUT 0b00000010
#define SERIAL_SHOW_DIGITAL_OUTPUT 0b00000100
#define SERIAL_SHOW_PWM_OUTPUT 0b00001000
#define SERIAL_SHOW_DCC_OUTPUT 0b00010000

// Radio settings
#define RADIO_CHAN 105
#define RADIO_EXTIO_ID 0

// Keep alive messages
#define KEEPALIVE_COUNT 500000

// CV indexes and default values
#define CV_DCC_ADDRESS_DEFAULT 9
#define CV_RADIO_ID 10
#define CV_RADIO_ID_DEFAULT 0
#define CV_PIN_ADDRESS_BASE 40
#define CV_PIN_ADDRESS_FIRST STD_VPIN_BASE
#define CV_SERIAL_OUTPUT 99
#define CV_SERIAL_OUTPUT_DEFAULT 0b11111111
#define CV_DECODER_MASTER_RESET 120


struct CVPair {
  uint16_t CV;
  uint8_t Value;
};

struct RadioPacket {
  uint8_t fromRadioId;
  uint8_t message;
  uint8_t data[4];
};

struct Pin {
  uint8_t pin;       // Hardware pin
  uint8_t function;  // Pin function
  uint8_t address;   // Virtual pin / dcc address

  uint16_t state;  // Value (high/low/pwm/analog)

  // PWM update vars
  uint8_t duration;   // Duration in 100ms parts
  uint16_t endValue;  // State value after sequence

  // For PWM
  uint16_t step;       // Value to add each 32ms
  uint16_t tempState;  // Current value

  uint32_t lastUpdate;  // Time (ms) of last update

#if not defined(_BE_TINY_)
  uint32_t startTime;  // Time (ms) the pwm sequence started
#endif
};

CVPair FactoryDefaultCVs[] = {
  // These two CVs define the Long Accessory Address
  { CV_ACCESSORY_DECODER_ADDRESS_LSB, CV_DCC_ADDRESS_DEFAULT & 0xFF },
  { CV_ACCESSORY_DECODER_ADDRESS_MSB, (CV_DCC_ADDRESS_DEFAULT >> 8) & 0x07 },

  { CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0 },
  { CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0 },

  // Accesory Decoder Short Address
  { CV_29_CONFIG, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE | CV29_F0_LOCATION },

  { CV_RADIO_ID, CV_RADIO_ID_DEFAULT },
  { CV_SERIAL_OUTPUT, CV_SERIAL_OUTPUT_DEFAULT },

  // Mapping from PIN_IO1 - PIN_IO16 to vpin address (or DCC address)
  // Note: vpin address is an OFFSET from the starting address of the EXTIO
  { CV_PIN_ADDRESS_BASE, CV_PIN_ADDRESS_FIRST },
  { CV_PIN_ADDRESS_BASE + 1, CV_PIN_ADDRESS_FIRST + 1 },
  { CV_PIN_ADDRESS_BASE + 2, CV_PIN_ADDRESS_FIRST + 2 },
  { CV_PIN_ADDRESS_BASE + 3, CV_PIN_ADDRESS_FIRST + 3 },
  { CV_PIN_ADDRESS_BASE + 4, CV_PIN_ADDRESS_FIRST + 4 },
  { CV_PIN_ADDRESS_BASE + 5, CV_PIN_ADDRESS_FIRST + 5 },
  { CV_PIN_ADDRESS_BASE + 6, CV_PIN_ADDRESS_FIRST + 6 },
  { CV_PIN_ADDRESS_BASE + 7, CV_PIN_ADDRESS_FIRST + 7 },

#if not defined(_BE_TINY_)
  { CV_PIN_ADDRESS_BASE + 8, CV_PIN_ADDRESS_FIRST + 8 },
  { CV_PIN_ADDRESS_BASE + 9, CV_PIN_ADDRESS_FIRST + 9 },
  { CV_PIN_ADDRESS_BASE + 10, CV_PIN_ADDRESS_FIRST + 10 },
  { CV_PIN_ADDRESS_BASE + 11, CV_PIN_ADDRESS_FIRST + 11 },
  { CV_PIN_ADDRESS_BASE + 12, CV_PIN_ADDRESS_FIRST + 12 },
  { CV_PIN_ADDRESS_BASE + 13, CV_PIN_ADDRESS_FIRST + 13 },
  { CV_PIN_ADDRESS_BASE + 14, CV_PIN_ADDRESS_FIRST + 14 },
  { CV_PIN_ADDRESS_BASE + 15, CV_PIN_ADDRESS_FIRST + 15 },
#endif

  { CV_DECODER_MASTER_RESET, 0 },

};

Pin _pin[] = {
  { PIN_IO1, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO2, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO3, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO4, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO5, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO6, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO7, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO8, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },

#if not defined(_BE_TINY_)
  { PIN_IO9, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO10, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO11, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO12, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO13, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO14, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO15, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
  { PIN_IO16, PINCONFIG_NOT_USED, 0, 0, 0, 0, 0, 0, 0 },
#endif
};

uint8_t _pwmIndex;
uint8_t _sendIndex = 0;
uint8_t _serialOutput;
uint8_t _hiCount;
uint8_t _DccOnlyMode;
uint8_t _radioReady;

uint16_t _offsetVPin;

uint32_t _lastSendCount;
uint32_t _endBlink = 0;

NRFLite _radio;
NmraDcc Dcc;
RadioPacket _radioData;

void setup() {
  resetCVFactoryDefault();

  pinMode(PIN_LED, OUTPUT);
  blink(5000);

  Sbegin(115200);
  Sprintln("--- TinySensor ---");
  Sprint("Version: ");
  Sprintln(TS_VER);

  initDCC();
  setPinModes();
  initRadio();
  printSummary(0);
}

void loop() {
  // Do all DCC stuff in library
  Dcc.process();

  // Send data
  send();

  if (_endBlink && millis() > _endBlink) {
    digitalWrite(PIN_LED, LOW);
    _endBlink = 0;
  }
}

void initDCC() {
  // Init DCC object
  Dcc.pin(0, PIN_DCC, 1);
  Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);

  // First run? Set all CV's to default in EEPROM
  if (Dcc.getCV(CV_DECODER_MASTER_RESET) == 255 || Dcc.getCV(CV_DECODER_MASTER_RESET) == CV_DECODER_MASTER_RESET) {
    resetCVFactoryDefault();
  }

  // Some settings can be put directly into the radioData
  _radioData.fromRadioId = Dcc.getCV(CV_RADIO_ID);

// What kind of info to print to the serial monitor
#if not defined(_BE_TINY_)
  _serialOutput = Dcc.getCV(CV_SERIAL_OUTPUT);
#endif

  // Get the pin config
  for (uint8_t i = 0; i < MAXPINS; i++) {
    _pin[i].address = Dcc.getCV(CV_PIN_ADDRESS_BASE + i);
    _pin[i].function = PINCONFIG_DIGITAL_INPUT;
  }
}

void blink(uint16_t duration) {
  digitalWrite(PIN_LED, HIGH);
  _endBlink = millis() + duration;
}

void setPinModes() {
  for (uint8_t i = 0; i < MAXPINS; i++) {
    if (_pin[i].pin != PINCONFIG_NOT_USED) {
      pinMode(_pin[i].pin, INPUT_PULLUP);
    }
  }
}

// Function only used on devices that support serial monitor, outputs a
// summary of all relevant settings
void printSummary(uint8_t onlyIO) {
#if not defined(_BE_TINY_)
  if (!onlyIO) {
    Sprint("I: DCC address: ");
    Sprintln(Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) + (Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) << 8));

    if (_DccOnlyMode) {
      Sprintln("I: Using only DCC outputs, radio will be ignored");
    } else {
      Sprint("I: EXTIO receiver ID: ");
      Sprintln(RADIO_EXTIO_ID);

      Sprint("I: Using radio channel: ");
      Sprintln(RADIO_CHAN);

      if (!_radioReady) {
        Sprintln("E: Radio NOT ready, will retry");
      }
    }
  }

  Sprintln("IO config:");
  for (uint8_t i = 0; i < MAXPINS; i++) {
    if (_pin[i].pin != PINCONFIG_NOT_USED) {
      Sprint(i + 1);
      Sprint(": pin: ");
      Sprint(_pin[i].pin);

      switch (_pin[i].function) {
        case PINCONFIG_DIGITAL_INPUT:
          if (_radioReady)
            Sprint(", vpin: ");
          else
            Sprint(", vpin offset: ");
          Sprint(_pin[i].address + _offsetVPin);
          Sprint(", ");
          Sprintln("digital wireless IN");
          break;
        case PINCONFIG_DIGITAL_OUTPUT:
          if (_radioReady)
            Sprint(", vpin: ");
          else
            Sprint(", vpin offset: ");
          Sprint(_pin[i].address + _offsetVPin);
          Sprint(", ");
          Sprintln("digital wireless OUT");
          break;
        case PINCONFIG_PWM_LED_OUTPUT:
          if (_radioReady)
            Sprint(", vpin: ");
          else
            Sprint(", vpin offset: ");
          Sprint(_pin[i].address + _offsetVPin);
          Sprint(", ");
          Sprintln("analog wireless LED OUT");
          break;
        case PINCONFIG_PWM_SERVO_OUTPUT:
          if (_radioReady)
            Sprint(", vpin: ");
          else
            Sprint(", vpin offset: ");
          Sprint(_pin[i].address + _offsetVPin);
          Sprint(", ");
          Sprintln("analog wireless SERVO OUT");
          break;
        case PINCONFIG_DCC_OUTPUT:
          Sprint(", DCC address: ");
          Sprint(_pin[i].address);
          Sprint(", ");
          Sprintln("digital DCC OUT");
          break;
        default:
          Sprintln(" UNKNOWN");
          break;
      }
    }
  }
#endif
}


// Init radio and get the basic info from the EXTIO controller
uint8_t initRadio() {

  Sprintln("I: Init radio:");

  _radioReady = 0;
  _offsetVPin = 0;

  // Set radioId to 255 if a previous radioId was not found in EEPROM
  if (!_radioData.fromRadioId) _radioData.fromRadioId = 255;

  if (!_radio.init(_radioData.fromRadioId, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, RADIO_CHAN)) {
    Sprintln("E: Failed to init radio for address request");
    return;
  }

  _radioData.message = PACKET_INIT;

  Sprint("S: Requesting radio id and vpin offset...");

  if (!_radio.send(RADIO_EXTIO_ID, &_radioData, sizeof(_radioData))) {
    Sprintln("failed (offline)");
    return 0;
  }

  // Wait for the EXTIO controller to respond (or timeout)
  uint16_t cnt = 0;

  while (!_radio.hasData()) {
    cnt++;
    if (cnt == 10000) {
      Sprintln("failed (timeout)");
      _radioData.fromRadioId = 0;
      return;
    }
  }

  Sprintln("done");

  // Read response, use radioId from response or
  _radio.readData(&_radioData);

  if (_radioData.message == PACKET_INIT && _radioData.data[0] != 255) {
    while (!Dcc.isSetCVReady())
      ;
    Dcc.setCV(CV_RADIO_ID, _radioData.data[0]);

    _radioData.fromRadioId = _radioData.data[0];
    _offsetVPin = _radioData.data[1] + (_radioData.data[2] << 8);
  } else {
    Sprintln("E: No radio id available...");

    _radioData.fromRadioId = 0;

    return;
  }

  Sprint("I: Using radio id: ");
  Sprintln(_radioData.fromRadioId);
  Sprint("I: Using vpin offset: ");
  Sprintln(_offsetVPin);

  if (!_radio.init(_radioData.fromRadioId, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, RADIO_CHAN)) {
    Sprintln("E: Failed to init _radio...");
    return;
  }

  Sprint("S: Sending vpin configuration...");

  for (uint8_t i = 0; i < MAXPINS; i++) {
    if (_pin[i].pin != PINCONFIG_NOT_USED) {  //} || func[i] == PINCONFIG_DCC_OUTPUT) {
      _radioData.message = PACKET_VPIN_CONFIG;
      _radioData.data[0] = i;
      _radioData.data[1] = _pin[i].address;
      _radioData.data[2] = _pin[i].function;

      if (!_radio.send(RADIO_EXTIO_ID, &_radioData, sizeof(_radioData))) {
        Sprintln("failed");
        return;
      }
    }
    Sprint(".");
  }

  Sprintln("done");

  _radioReady = 1;

  Sprintln("I: Radio init done!");
}

// Check the INPUT pins and send data if needed
void send() {
  uint16_t newState;
  uint16_t state = 0;

  // Check 1 pin each "loop"
  _sendIndex++;
  _sendIndex %= MAXPINS;

  // Default = nothing happened
  _radioData.message = PACKET_EMPTY;

  // This pin a digital input? Check if the value changed and
  // if so -> send to EXTIO controller
  if (_pin[_sendIndex].function == PINCONFIG_DIGITAL_INPUT) {
    _radioData.message = PACKET_UPDATE_DIGITAL;
    newState = !digitalRead(_pin[_sendIndex].pin);

#if not defined(_BE_TINY_)
    if (newState != _pin[_sendIndex].state && _serialOutput & SERIAL_SHOW_DIGITAL_INPUT) {
      Sprint("S: (DIGITAL) pin ");
      Sprint(_pin[_sendIndex].pin);
      Sprint(", vpin ");
      Sprint(_pin[_sendIndex].address + _offsetVPin);
      Sprint(", state ");
      serialHighOrLow(newState);
      Sprint("...");
    }
#endif
  }

  // If the message type changed from "EMPTY" do the actual sending
  // Set the stored state to the state that was send (so it will retry in
  // the event of send error)
  if (_radioData.message && newState != _pin[_sendIndex].state) {
    blink(250);

    _radioData.data[0] = _pin[_sendIndex].address;
    _radioData.data[1] = newState;

    if (_radio.send(RADIO_EXTIO_ID, &_radioData, sizeof(_radioData))) {
      _pin[_sendIndex].state = newState;

#if not defined(_BE_TINY_)
      Sprintln("done");
    } else {
      Sprintln("failed");
#endif
    }
  }
}

// Print "HIGH" or "LOW" for 1 / 0
void serialHighOrLow(uint8_t n) {
  if (n)
    Serial.print("HIGH");
  else
    Serial.print("LOW");
}


// Reset all CV's to factory default
void resetCVFactoryDefault() {
  uint8_t n = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
  Sprintln("I: Factory reset!");

  for (uint8_t i = 0; i < n; i++) {
    while (!Dcc.isSetCVReady())
      ;
    Dcc.setCV(FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  }

  setPinModes();

  _radioReady = 0;
}

void notifyCVChange(uint16_t CV, uint8_t Value) {
  blink(1000);
}

// Called from DCC library @ factory reset
void notifyCVResetFactoryDefault() {
  resetCVFactoryDefault();
}

// Called from DCC library when a turnout is set
void notifyDccAccTurnoutOutput(uint16_t address, uint8_t direction, uint8_t outputPower) {
  // Ignore pulselength packet
  if (!outputPower) return;

  blink(250);

  // Check if the address matches our pins
  for (uint8_t i = 0; i < MAXPINS; i++) {
    if (_pin[i].function == PINCONFIG_DCC_OUTPUT && _pin[i].address == address) {
      digitalWrite(_pin[i].pin, direction);

#if not defined(_BE_TINY_)
      if (_serialOutput & SERIAL_SHOW_DCC_OUTPUT) {
        Sprint("I: (DCC) pin ");
        Sprint(_pin[i].pin);
        Sprint(", DCC address ");
        Sprint(_pin[i].address);
        Sprint(", state ");
        serialHighOrLow(direction);
        Sprintln();
      }
#endif
    }
  }
}
