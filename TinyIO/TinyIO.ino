/*

TinyIO

Input/output for DCC-EX



Radio    Arduino
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND

*/

#include <SPI.h>
#include <NRFLite.h>
#include <NmraDcc.h>
#include <EEPROM.h>


// If using own hardwareconfiguration use section below to configure TIO

// Uncomment to ignore all serial comms
//#define NO_SERIAL

// TODO: Change for TINY!!
#define PIN_DCC 2
#define PIN_RADIO_CE 9
#define PIN_RADIO_CSN 10

// Mapping IO internal pin to hardware pin
#define PIN_IO1 4
#define PIN_IO2 3
#define PIN_IO3 7
#define PIN_IO4 5
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
#define TIO_VER "0.9"

// Arduino
#if defined(__AVR_ATmega328P__)
#define MAXPINS 16

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
#define PINCONFIG_ANALOG_INPUT 3
#define PINCONFIG_PWM_LED_OUTPUT 4
#define PINCONFIG_PWM_SERVO_OUTPUT 5
#define PINCONFIG_DCC_OUTPUT 6

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
#define SERIAL_SHOW_ANALOG_INPUT 0b00000100
#define SERIAL_SHOW_DIGITAL_OUTPUT 0b00001000
#define SERIAL_SHOW_PWM_OUTPUT 0b00010000
#define SERIAL_SHOW_DCC_OUTPUT 0b00100000

// Servo timings (SG90)
#define SERVO_PWM_MIN_DURATION 500
#define SERVO_PWM_MAX_DURATION 2400


// Radio settings
#define RADIO_CHAN 105
#define RADIO_EXIO_ID 0

// Keep alive messages
#define KEEPALIVE_COUNT 100000
//#define KEEPALIVE_COUNT 1000000

// CV indexes and default values
#define CV_DCC_ADDRESS_DEFAULT 9
#define CV_LED 2
#define CV_LED_DEFAULT 0
#define CV_RADIO_ID 10
#define CV_RADIO_ID_DEFAULT 0
#define CV_PIN_ADDRESS_BASE 40
#define CV_PIN_ADDRESS_FIRST 0
#define CV_PIN_FUNCTION_BASE 60
#define CV_PIN_FUNCTION_DEFAULT PINCONFIG_DIGITAL_INPUT
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

  uint16_t step;       // Fraction value to add each 100ms
  uint16_t tempState;  // Current fraction

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
  // Note: vpin address is an OFFSET from the starting address of the EXIO
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

  // Defines what function PIN_IO1 - PIN_IO16 should have
  { CV_PIN_FUNCTION_BASE, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 1, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 2, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 3, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 4, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 5, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 6, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 7, CV_PIN_FUNCTION_DEFAULT },

#if not defined(_BE_TINY_)
  { CV_PIN_FUNCTION_BASE + 8, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 9, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 10, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 11, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 12, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 13, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 14, CV_PIN_FUNCTION_DEFAULT },
  { CV_PIN_FUNCTION_BASE + 15, CV_PIN_FUNCTION_DEFAULT },
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

uint8_t _pwmIndex = 0;
uint8_t _sendIndex = 0;
uint16_t _offsetVPin;
uint8_t _DccOnlyMode;
uint8_t _radioReady;
uint32_t _lastSendCount = 0;
uint8_t _serialOutput;
uint8_t _hiCount = 0;

NRFLite _radio;
NmraDcc Dcc;
RadioPacket _radioData;

void setup() {
  // resetCVFactoryDefault();

  Sbegin(115200);
  Sprintln("--- TinyIO ---");
  Sprint("Version: ");
  Sprintln(TIO_VER);

  initDCC();
  setPinModes();
  initRadio();
  printSummary(0);
}

void loop() {
  // Do all DCC stuff in library
  Dcc.process();

  // Send & receive data
  if (!_DccOnlyMode && _radioReady) {
    send();
    receive();
    pwm();
  }

  sayHi();
}


void pwm() {
  uint16_t passed;
  uint8_t multiplier;

  _pwmIndex++;

  if (_pwmIndex == MAXPINS) _pwmIndex = 0;
  if (!_pin[_pwmIndex].duration) return;

  passed = millis() - _pin[_pwmIndex].lastUpdate;

  multiplier = passed >> 5;

  while (multiplier) {
    _pin[_pwmIndex].tempState += _pin[_pwmIndex].step;

    if (_pin[_pwmIndex].duration)
      _pin[_pwmIndex].duration--;

    multiplier--;

    if (!multiplier) {
      _pin[_pwmIndex].lastUpdate = millis();

      if (_pin[_pwmIndex].function == PINCONFIG_PWM_LED_OUTPUT) {
        // This is a LED PWM, output direct analog value
        _pin[_pwmIndex].state = _pin[_pwmIndex].tempState >> 8;
        analogWrite(_pin[_pwmIndex].pin, _pin[_pwmIndex].state);
      } else {
        // This is a SERVO PWM, output a high pulse
        _pin[_pwmIndex].state = _pin[_pwmIndex].tempState >> 3;
        digitalWrite(_pin[_pwmIndex].pin, HIGH);
        delayMicroseconds(_pin[_pwmIndex].state);
        digitalWrite(_pin[_pwmIndex].pin, LOW);
      }

      if (!_pin[_pwmIndex].duration) {
        _pin[_pwmIndex].state = _pin[_pwmIndex].endValue;

#if not defined(_BE_TINY_)
        if (_serialOutput & SERIAL_SHOW_PWM_OUTPUT) {
          if (_pin[_pwmIndex].function == PINCONFIG_PWM_LED_OUTPUT)
            Sprint("I: (LED) pin ");
          else
            Sprint("I: (SERVO) pin ");
          Sprint(_pin[_pwmIndex].pin);
          Sprint(", vpin ");
          Sprint(_pin[_pwmIndex].address + _offsetVPin);
          Sprint(", value update to ");
          Sprint(_pin[_pwmIndex].state);
          Sprint(" done in ");
          Sprint(millis() - _pin[_pwmIndex].startTime);
          Sprintln("ms");
        }
#endif
      }
    }
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
    _pin[i].function = Dcc.getCV(CV_PIN_FUNCTION_BASE + i);
  }
}

void setPinModes() {
  // TEST PURPOSE
  _pin[0].function = PINCONFIG_PWM_SERVO_OUTPUT;
  _pin[3].function = PINCONFIG_PWM_LED_OUTPUT;

  _DccOnlyMode = 1;

  for (uint8_t i = 0; i < MAXPINS; i++) {
    if (_pin[i].pin != PINCONFIG_NOT_USED) {
      pinMode(_pin[i].pin, OUTPUT);

      if (_pin[i].function == PINCONFIG_DIGITAL_INPUT)
        pinMode(_pin[i].pin, INPUT_PULLUP);

      if (_pin[i].function != PINCONFIG_DCC_OUTPUT) {
        _DccOnlyMode = 0;
      }
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
      Sprint("I: EXIO receiver ID: ");
      Sprintln(RADIO_EXIO_ID);

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
          pinMode(_pin[i].pin, INPUT_PULLUP);
          if (_radioReady)
            Sprint(", vpin: ");
          else
            Sprint(", vpin offset: ");
          Sprint(_pin[i].address + _offsetVPin);
          Sprint(", ");
          Sprintln("digital wireless IN");
          break;
        case PINCONFIG_DIGITAL_OUTPUT:
          pinMode(_pin[i].pin, OUTPUT);
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

// Init radio and get the basic info from the EXIO controller
uint8_t initRadio() {
  // Only DCC?
  if (_DccOnlyMode) return;

  Sprintln("I: Init radio:");

  _radioReady = 0;
  _offsetVPin = 0;

  // Set radioId to 255 if a previous radioId was not found in EEPROM
  if (!_radioData.fromRadioId) _radioData.fromRadioId = 255;

  if (!_radio.init(_radioData.fromRadioId, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE1MBPS, RADIO_CHAN)) {
    Sprintln("E: Failed to init radio for address request");
    return;
  }

  _radioData.message = PACKET_INIT;

  Sprint("S: Requesting radio id and vpin offset...");

  if (!_radio.send(RADIO_EXIO_ID, &_radioData, sizeof(_radioData))) {
    Sprintln("failed (offline)");
    return 0;
  }

  // Wait for the EXIO controller to respond (or timeout)
  uint16_t cnt = 0;

  while (!_radio.hasData()) {
    cnt++;
    if (cnt == 5000) {
      Sprintln("failed (timeout)");
      _radioData.fromRadioId = 0;
      return;
    }
  }

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

  if (!_radio.init(_radioData.fromRadioId, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE1MBPS, RADIO_CHAN)) {
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

      if (!_radio.send(RADIO_EXIO_ID, &_radioData, sizeof(_radioData))) {
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
  uint8_t newState;

  // Check 1 pin each "loop"
  _sendIndex++;
  _sendIndex %= MAXPINS;

  if (!_pin[_sendIndex].pin)
    return;

  // Default = nothing happened
  _radioData.message = PACKET_EMPTY;

  // This pin an analog input? Check if the value changed and
  // if so -> send to EXIO controller
  if (_pin[_sendIndex].function == PINCONFIG_ANALOG_INPUT) {
    _radioData.message = PACKET_UPDATE_ANALOG;
    newState = analogRead(_pin[_sendIndex].pin);

#if not defined(_BE_TINY_)
    if (newState != _pin[_sendIndex].state && _serialOutput & SERIAL_SHOW_ANALOG_INPUT) {
      Sprint("S: (ANALOG) pin ");
      Sprint(_pin[_sendIndex].pin);
      Sprint(", vpin ");
      Sprint(_pin[_sendIndex].address + _offsetVPin);
      Sprint(", value ");
      Sprint(_pin[_sendIndex].state);
      Sprint(" > ");
      Sprint(newState);
      Sprint("...");
    }
#endif
  }

  // This pin a digital input? Check if the value changed and
  // if so -> send to EXIO controller
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
    _radioData.data[0] = _pin[_sendIndex].address;
    _radioData.data[1] = newState;

    if (_radio.send(RADIO_EXIO_ID, &_radioData, sizeof(_radioData))) {
      _pin[_sendIndex].state = newState;

#if not defined(_BE_TINY_)
      Sprintln("done");
    } else {
      Sprintln("failed");
#endif
    }
  }
}

// Process packets received from the EXIO controller
void receive() {
  uint8_t ppin;
  uint8_t ipin;
  int16_t delta;

  if (!_radio.hasData()) return;

  _radio.readData(&_radioData);

  ipin = _radioData.data[0];
  ppin = _pin[ipin].pin;

  switch (_radioData.message) {
#if not defined(_BE_TINY_)
    case PACKET_JUST_SAY_HI:
      if (_serialOutput & SERIAL_SHOW_KEEPALIVE) {
        Sprint("R: EXIO says hi! (");
        Sprint(_radioData.data[0]);
        Sprintln(")");
      }
      break;
#endif

    case PACKET_INIT:
      Sprintln("R: EXIO asked for re-init");
      initRadio();
      break;

    case PACKET_UPDATE_DIGITAL:
      digitalWrite(ppin, _radioData.data[1]);

#if not defined(_BE_TINY_)
      if (_serialOutput & SERIAL_SHOW_DIGITAL_OUTPUT) {
        Sprint("R: (DIGITAL) pin ");
        Sprint(ppin);
        Sprint(", vpin ");
        Sprint(_pin[ipin].address + _offsetVPin);
        Sprint(", state ");
        serialHighOrLow(_radioData.data[1]);
        Sprintln();
      }
#endif

      break;

    case PACKET_UPDATE_ANALOG:
      _pin[ipin].endValue = _radioData.data[1] + (_radioData.data[3] << 8);
      // Duration in 32ms parts!
      _pin[ipin].duration = _radioData.data[2];

      delta = (_pin[ipin].endValue - _pin[ipin].state) << 3;
      _pin[ipin].tempState = _pin[ipin].state << 3;

      if (_pin[ipin].function == PINCONFIG_PWM_LED_OUTPUT) {
        delta = delta << 3;
        _pin[ipin].tempState = _pin[ipin].tempState << 3;
      }

      _pin[ipin].step = delta / _pin[ipin].duration;
      _pin[ipin].lastUpdate = millis();

#if not defined(_BE_TINY_)
      if (_serialOutput & SERIAL_SHOW_PWM_OUTPUT) {
        // Only keep track of time on serial enabled devices
        _pin[ipin].startTime = _pin[ipin].lastUpdate;

        if (_pin[_pwmIndex].function == PINCONFIG_PWM_LED_OUTPUT)
          Sprint("R: (LED) pin ");
        else
          Sprint("R: (SERVO) pin ");

        Sprint(ppin);
        Sprint(", vpin ");
        Sprint(_pin[ipin].address + _offsetVPin);
        Sprint(", value update ");
        Sprint(_pin[ipin].state);
        Sprint(" > ");
        Sprint(_pin[ipin].endValue);
        Sprint(" in ");
        Sprint(_pin[ipin].duration * 32);
        Sprintln("ms...");
      }
#endif
      break;
  }
}


// Print "HIGH" or "LOW" for 1 / 0
void serialHighOrLow(uint8_t n) {
  if (n)
    Serial.print("HIGH");
  else
    Serial.print("LOW");
}

// Sort of keepalive, if fails it will try to re-init
void sayHi() {
  if (_lastSendCount++ < KEEPALIVE_COUNT) return;

  _lastSendCount = 0;

  if (!_radioReady) {
    initRadio();

    if (_radioReady) printSummary(1);

    return;
  }

  _radioData.message = PACKET_JUST_SAY_HI;
  _radioData.data[0] = _hiCount++;

#if not defined(_BE_TINY_)
  if (_serialOutput & SERIAL_SHOW_KEEPALIVE) {
    Sprint("S: Saying hi to EXIO (");
    Sprint(_radioData.data[0]);
    Sprint(")...");
  }
#endif

  if (!_radio.send(RADIO_EXIO_ID, &_radioData, sizeof(_radioData)) && !_radio.send(RADIO_EXIO_ID, &_radioData, sizeof(_radioData))) {
    _radioReady = 0;

#if not defined(_BE_TINY_)
    if (_serialOutput & SERIAL_SHOW_KEEPALIVE)
      Sprintln("failed, set Radio to NOT ready");
  } else {
    if (_serialOutput & SERIAL_SHOW_KEEPALIVE)
      Sprintln("done");
#endif
  }
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

// Called from DCC library @ factory reset
void notifyCVResetFactoryDefault() {
  resetCVFactoryDefault();
}

// Called from DCC library when a turnout is set
void notifyDccAccTurnoutOutput(uint16_t address, uint8_t direction, uint8_t outputPower) {
  // Ignore pulselength packet
  if (!outputPower) return;

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

