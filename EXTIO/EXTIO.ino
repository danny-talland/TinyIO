#include <Wire.h>
#include <SPI.h>
#include <NRFLite.h>
#include <EEPROM.h>

// Config I2C
#define EXIO_ADDRESS 0x65          // Extender address
#define EXIO_PIN_START 400         // First VPIN
#define EXIO_DIGITAL_PIN_COUNT 50  // # of pins (digital input only)
#define EXIO_ANALOG_PIN_COUNT 0
#define EXIO_PIN_TABLE_SIZE 50  // which vpin @ which radio (single vpin can have multiple in/out)

#define VERSION_MAJOR 0  // Still a beta thing
#define VERSION_MINOR 9
#define VERSION_PATCH 0

// Config radio
#define RADIO_ID 0        // Radio address (0..125)
#define RADIO_CHAN 105    // Radio channel (0..125)
#define RADIO_PIN_CE 9    // Radio CE pin
#define RADIO_PIN_CSN 10  // Radui CSN pin

// Config general
#define LED_PIN 13

// Nothing to configure below this
#define Sbegin(a) (Serial.begin(a))
#define Sprintln(a) (Serial.println(a))
#define Sprint(a) (Serial.print(a))
#define SprintHEX(a) (Serial.print(a, HEX))

#if defined(NO_SERIAL)
#define Sbegin(a)
#define Sprintln(a)
#define Sprint(a)
#endif

// EXIO op codes (not all are used for now)
#define EXIOINIT 0xE0   // Initialise the setup procedure
#define EXIORDY 0xE1    // Setup procedure complete
#define EXIODPUP 0xE2   // Send digital pin pullup configuration
#define EXIOVER 0xE3    // Get version
#define EXIORDAN 0xE4   // Read an analogue input
#define EXIOWRD 0xE5    // Send a digital write
#define EXIORDD 0xE6    // Read a digital input
#define EXIOENAN 0xE7   // Enable an analogue input
#define EXIOINITA 0xE8  // Request/receive analogue pin mappings
#define EXIOPINS 0xE9   // Request/receive buffer counts
#define EXIOWRAN 0xEA   // Send an analogue write (PWM)
#define EXIOERR 0xEF    // Error sent/received

#define EEPROM_RADIO_COUNT 25
#define EEPROM_RADIO_DEFAULT 10

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

// Servo PWM length (SG90)
#define SERVO_PWM_MIN_DURATION 500
#define SERVO_PWM_MAX_DURATION 2400

struct RadioPacket  // Any packet up to 32 bytes can be sent.
{
  uint8_t fromRadioId;

  uint8_t message;
  uint8_t data[4];
};

struct VPinConfig {
  uint8_t remoteIndex;
  uint8_t offset;
  uint8_t function;
  uint8_t radio;
  uint8_t state;
};


// Global vars
uint8_t outOP;
uint8_t _digitalPinStatus[EXIO_DIGITAL_PIN_COUNT];
uint8_t _digitalPinStatusBytesCount;
uint8_t *_digitalPinStatusBytes;

uint8_t _analogPinStatusBytesCount;
uint8_t *_analogPinMap;
uint8_t *_analogPinStatusBytes;

uint8_t _radiosOnline[EEPROM_RADIO_COUNT + 1];
NRFLite radio;
RadioPacket _radioData;
VPinConfig vPinConfig[EXIO_PIN_TABLE_SIZE];
uint32_t time;


void setup() {
  Sbegin(115200);
  Sprintln("--- EXTIO ---");
  Sprint("Version: ");
  Sprint(VERSION_MAJOR);
  Sprint(".");
  Sprint(VERSION_MINOR);
  Sprint(".");
  Sprintln(VERSION_PATCH);


  setRadioIds();
  initVPinConfig();
  initPinStatus();

  Wire.begin(EXIO_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  if (!radio.init(RADIO_ID, RADIO_PIN_CE, RADIO_PIN_CSN, NRFLite::BITRATE1MBPS, RADIO_CHAN)) {
    Sprintln("FATAL: Cannot communicate with radio...");
    death();
  }

  time = millis();
}

uint8_t step = 0;


void loop() {
  processRadioData();

  if (time + 15000 < millis()) {
    if (step) {
      sendPWMOutput(0, 40, 20);
      sendPWMOutput(3, 255, 50);
    } else {

      sendPWMOutput(3, 0, 30);
      /*
      sendPWMOutput(0, 40, 0);
      delay(500);
      sendPWMOutput(0, 130, 3);
      delay(300);
      sendPWMOutput(0, 94, 1);
      delay(100);
      sendPWMOutput(0, 130, 2);
      delay(200);
      sendPWMOutput(0, 103, 1);
      delay(100);
      sendPWMOutput(0, 130, 3);
      delay(300);
      */
    }

    time = millis();
    step = !step;
  }
}

void processRadioData() {
  while (radio.hasData()) {
    radio.readData(&_radioData);

    switch (_radioData.message) {
      case PACKET_INIT:
        if (_radioData.fromRadioId == 255) {
          Sprint("R: Radio init: ask for new ID and vpin offset ");
        } else {
          Sprint("R: Radio init: wants to use ID ");
          Sprint(_radioData.fromRadioId);
          Sprint(" and asks for vpin offset ");

          clearRadioVPinConfig(_radioData.fromRadioId);
        }

        _radioData.data[0] = getRadioId(_radioData.fromRadioId);
        _radioData.data[1] = EXIO_PIN_START;
        _radioData.data[2] = EXIO_PIN_START >> 8;

        Sprint("(id:");
        Sprint(_radioData.data[0]);
        Sprint(", vpin offset:");
        Sprint(EXIO_PIN_START);
        Sprint(")");


        if (radio.send(_radioData.fromRadioId, &_radioData, sizeof(_radioData))) {
          _radiosOnline[_radioData.data[0]] = 1;
          Sprintln("...done");
        } else Sprintln("...failed");

        break;

      case PACKET_VPIN_CONFIG:
        Sprint("R: Radio ");
        Sprint(_radioData.fromRadioId);
        Sprint(" config: services vpin ");
        Sprint(_radioData.data[0] + EXIO_PIN_START);

        if (_radioData.data[2] == PINCONFIG_DIGITAL_INPUT) Sprint(" as digital IN...");
        if (_radioData.data[2] == PINCONFIG_DIGITAL_OUTPUT) Sprint(" as digital OUT...");
        if (_radioData.data[2] == PINCONFIG_PWM_LED_OUTPUT) Sprint(" as PWM LED OUT...");
        if (_radioData.data[2] == PINCONFIG_PWM_SERVO_OUTPUT) Sprint(" as PWM SERVO OUT...");
        if (_radioData.data[2] == PINCONFIG_ANALOG_INPUT) Sprint(" as analog IN...");

        if (newVPinConfig(_radioData.data[0], _radioData.data[1], _radioData.data[2], _radioData.fromRadioId))
          Sprintln("saved");
        else
          Sprintln("not saved (config table full)");

        break;

      case PACKET_UPDATE_DIGITAL:
        Sprint("R: radio ");
        Sprint(_radioData.fromRadioId);
        Sprint(", vpin ");
        Sprint(_radioData.data[0] + EXIO_PIN_START);
        Sprint(" state update ");
        Sprint(_digitalPinStatus[_radioData.data[0]]);
        Sprint(" > ");
        Sprintln(_radioData.data[1]);

        _digitalPinStatus[_radioData.data[0]] = _radioData.data[1];

        break;

      case PACKET_JUST_SAY_HI:
        Sprint("R: Radio ");
        Sprint(_radioData.fromRadioId);
        Sprint(" says hi, responding (");
        Sprint(_radioData.data[0]);
        Sprint(")");

        if (!_radiosOnline[_radioData.fromRadioId]) {
          Sprint(" with re-init requested");
          _radioData.message = PACKET_INIT;
        }

        if (radio.send(_radioData.fromRadioId, &_radioData, sizeof(_radioData)))
          Sprintln("...done");
        else
          Sprintln("...failed");

        break;

      default:
        Sprint("R: Unknown data, ignoring! (id:");
        Sprint(_radioData.fromRadioId);
        Sprint(", m:");
        Sprint(_radioData.message);
        Sprint(", 0:");
        Sprint(_radioData.data[0]);
        Sprint(", 1:");
        Sprint(_radioData.data[1]);
        Sprint(", 2:");
        Sprint(_radioData.data[2]);
        Sprintln(")");
    }
  }
}

void sendDigitalOutput(uint8_t offset, uint8_t state) {
  uint8_t found = 0;

  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    if (vPinConfig[i].function != PINCONFIG_NOT_USED && vPinConfig[i].offset == offset) {
      if (vPinConfig[i].function == PINCONFIG_DIGITAL_OUTPUT) {
        found = 1;

        Sprint("S: radio ");
        Sprint(vPinConfig[i].radio);
        Sprint(", vpin ");
        Sprint(offset + EXIO_PIN_START);
        Sprint(", state ");
        serialHighOrLow(state);

        _radioData.fromRadioId = vPinConfig[i].radio;
        _radioData.message = PACKET_UPDATE_DIGITAL;
        _radioData.data[0] = vPinConfig[i].remoteIndex;
        _radioData.data[1] = state;

        if (radio.send(vPinConfig[i].radio, &_radioData, sizeof(_radioData)))
          Sprintln("...done");
        else
          Sprintln("...failed");
      } else {
        Sprint("E: radio ");
        Sprint(vPinConfig[i].radio);
        Sprint(", vpin ");
        Sprint(offset + EXIO_PIN_START);
        Sprint("...tried to set as digital OUT configured as ");
        serialPinConfiguration(vPinConfig[i].function);
        Sprintln(", ignored");
      }
    }
  }

  if (!found) {
    Sprint("E: vpin ");
    Sprint(offset + EXIO_PIN_START);
    Sprintln(" was not found to be online, command ignored...");
  }
}



void sendPWMOutput(uint8_t offset, uint8_t value, uint8_t duration) {
  uint8_t found = 0;
  float usDelayPrecise;
  uint16_t usDelay;

  // Duration on the TinyIO side must fit in a byte (32ms * duration byte)
  if (duration > 80) {
    Sprintln("I: Duration corrected to 8000ms (max)...");
    duration = 80;
  }

  // Instant movement always takes at least 32 ms (prevent div by 0)
  if (!duration) duration = 1;

  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    if (vPinConfig[i].function != PINCONFIG_NOT_USED && vPinConfig[i].offset == offset) {
      if (vPinConfig[i].function == PINCONFIG_PWM_LED_OUTPUT || vPinConfig[i].function == PINCONFIG_PWM_SERVO_OUTPUT) {
        found = 1;

        Sprint("S: radio ");
        Sprint(vPinConfig[i].radio);
        Sprint(", vpin ");
        Sprint(offset + EXIO_PIN_START);

        if (vPinConfig[i].function == PINCONFIG_PWM_LED_OUTPUT) {
          _radioData.fromRadioId = vPinConfig[i].radio;
          _radioData.message = PACKET_UPDATE_ANALOG;
          _radioData.data[0] = vPinConfig[i].remoteIndex;
          _radioData.data[1] = value;
          _radioData.data[2] = (duration * 100) / 32;
          _radioData.data[3] = 0;

          Sprint(", PWM LED value ");
          Sprint(value);
        } else {

          if (value > 180) value = 180;

          usDelayPrecise = ((SERVO_PWM_MAX_DURATION - SERVO_PWM_MIN_DURATION) / (float)180) * value;
          usDelay = (uint16_t)usDelayPrecise;

          _radioData.fromRadioId = vPinConfig[i].radio;
          _radioData.message = PACKET_UPDATE_ANALOG;
          _radioData.data[0] = vPinConfig[i].remoteIndex;
          _radioData.data[1] = usDelay & 0b11111111;
          _radioData.data[2] = (duration * 100) / 32;
          _radioData.data[3] = usDelay >> 8;

          Sprint(", PWM SERVO angle ");
          Sprint(value);
          Sprint("deg (pulse width: ");
          Sprint(SERVO_PWM_MIN_DURATION + usDelay);
          Sprint("us)");
        }

        Sprint(", duration ");
        Sprint(duration * 100);
        Sprint("ms");

        if (radio.send(vPinConfig[i].radio, &_radioData, sizeof(_radioData)))
          Sprintln("...done");
        else
          Sprintln("...failed");
      } else {
        Sprint("E: radio ");
        Sprint(vPinConfig[i].radio);
        Sprint(", vpin ");
        Sprint(offset + EXIO_PIN_START);
        Sprint("...tried to set as PWM OUT configured as ");
        serialPinConfiguration(vPinConfig[i].function);
        Sprintln(", ignored");
      }
    }
  }

  if (!found) {
    Sprint("E: vpin ");
    Sprint(offset + EXIO_PIN_START);
    Sprintln(" was not found to be online, command ignored...");
  }
}

void serialHighOrLow(uint8_t n) {
  if (n)
    Sprint("HIGH");
  else
    Sprint("LOW");
}

void serialPinConfiguration(uint8_t n) {
  switch (n) {
    case PINCONFIG_DIGITAL_INPUT: Sprint("digital IN"); break;
    case PINCONFIG_DIGITAL_OUTPUT: Sprint("digital OUT"); break;
    case PINCONFIG_ANALOG_INPUT: Sprint("analog IN"); break;
    case PINCONFIG_PWM_LED_OUTPUT: Sprint("PWM LED OUT"); break;
    case PINCONFIG_PWM_SERVO_OUTPUT: Sprint("PWM SERVO OUT"); break;
    default: Sprint("unknown"); break;
  }
}

void setRadioIds() {
  uint8_t n;

  for (uint8_t i = 1; i <= EEPROM_RADIO_COUNT; i++) {
    n = EEPROM.read(i);

    if (n > EEPROM_RADIO_DEFAULT)
      n = 0;

    if (n > 0)
      n--;

    EEPROM.write(i, n);

    _radiosOnline[i] = 0;
  }
}

uint8_t getRadioId(uint8_t current) {
  uint8_t n;

  for (uint8_t i = 1; i <= EEPROM_RADIO_COUNT; i++) {
    n = EEPROM.read(i);

    Sprint(i);
    Sprint(":");
    Sprint(n);
    Sprint(" - ");
  }
  Sprintln();

  if (current < EEPROM_RADIO_COUNT && EEPROM.read(current) > 0) {
    EEPROM.write(current, EEPROM_RADIO_DEFAULT);

    return current;
  }

  for (uint8_t i = 1; i <= EEPROM_RADIO_COUNT; i++) {
    n = EEPROM.read(i);

    if (n == 0) {
      EEPROM.write(i, EEPROM_RADIO_DEFAULT);

      return i;
    }
  }

  return 255;
}

void initVPinConfig() {
  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    vPinConfig[i].function = PINCONFIG_NOT_USED;
  }
}

uint8_t newVPinConfig(uint8_t remoteIndex, uint8_t offset, uint8_t function, uint8_t radio) {
  uint8_t i = 0;

  while (1) {
    if (vPinConfig[i].function == PINCONFIG_NOT_USED) break;

    if (i == EXIO_PIN_TABLE_SIZE) return 0;

    i++;
  }

  vPinConfig[i].remoteIndex = remoteIndex;
  vPinConfig[i].offset = offset;

  vPinConfig[i].function = function;
  vPinConfig[i].radio = radio;
  vPinConfig[i].state = 0;

  return 1;
}

void clearRadioVPinConfig(uint8_t radio) {
  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    if (vPinConfig[i].radio == radio) {
      vPinConfig[i].function = PINCONFIG_NOT_USED;
    }
  }
}

/*
uint8_t getVPinStatus(uint8_t offset) {
  uint8_t state = 0;

  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    if (vPinConfig[i].function == PINCONFIG_DIGITAL_INPUT && vPinConfig[i].offset == offset && vPinConfig[i].state > 0)
      state = 1;
  }

  return state;
}
*/


void receiveData(int numBytes) {
  if (numBytes == 0) {
    return;
  }

  uint8_t buffer[numBytes];

  for (uint8_t i = 0; i < numBytes; i++) {
    buffer[i] = Wire.read();

    if (buffer[0] != EXIORDD && buffer[0] != EXIORDAN) {
      Sprint(buffer[i]);
      Sprint(" (0x");
      SprintHEX(buffer[i]);
      Sprint(") ");
      Sprint(" - ");
    }
  }

  switch (buffer[0]) {
    case EXIOINIT:
      Sprintln("EXIOINIT (init device)");

      if (buffer[1] == EXIO_DIGITAL_PIN_COUNT && ((buffer[3] << 8) + buffer[2]) == EXIO_PIN_START) {
        outOP = EXIOPINS;
      } else {
        outOP = 0;
      }
      break;

    case EXIOINITA:
      Sprintln("EXIOINITA (request analog pin mappings)");
      outOP = EXIOINITA;
      break;

    case EXIODPUP:
      Sprintln("EXIODPUP (pin pullup configuration)");
      outOP = EXIORDY;
      break;

    case EXIOVER:
      Sprintln("EXIOVER (version number)");
      outOP = EXIOVER;
      break;

    case EXIOWRD:
      // Write digital pin status
      Sprintln("Writing digital pin");
      sendDigitalOutput(buffer[1], buffer[2]);
      outOP = EXIORDY;
      break;

    case EXIORDD:
      // Read digital pin status
      getDigitalPinStatus();
      outOP = EXIORDD;
      break;

    case EXIORDAN:
      getAnalogPinStatus();
      outOP = EXIORDAN;
      break;

    case EXIOWRAN:
      // Value is 16 bit but Tiny PWM only supports 255, truncate to a byte.
      uint16_t value = (buffer[3] << 8) + buffer[2];
      if (value > 255) value = 255;

      // Profile is ignored for now
      // uint8_t profile = buffer[4];

      // Duration seems to be divided by 100 by commandstation
      // Truncate to a byte.
      uint16_t duration = (buffer[6] << 8) + buffer[5];
      if (duration > 255) duration = 255;

      sendPWMOutput(buffer[1], value, duration);

      outOP = EXIORDY;

      break;


    default:
      Sprint("UNK: ");
      SprintHEX(buffer[0]);
      Sprintln();
      outOP = 0;
      break;
  }
}

void sendData() {
  switch (outOP) {
    case EXIOPINS:
      Wire.write(EXIOPINS);
      Wire.write(EXIO_DIGITAL_PIN_COUNT);
      Wire.write(EXIO_ANALOG_PIN_COUNT);
      break;

    case EXIOINITA:
      //_analogPinMap[0] = 0;
      //_analogPinMap[1] = 2;
      Wire.write(_analogPinMap, EXIO_ANALOG_PIN_COUNT);
      break;

    case EXIORDY:
      Wire.write(EXIORDY);
      break;

    case EXIOVER:
      Wire.write(VERSION_MAJOR);
      Wire.write(VERSION_MINOR);
      Wire.write(VERSION_PATCH);

      Sprintln("EXIO setup complete....");
      break;

    case EXIORDD:
      Wire.write(_digitalPinStatusBytes, _digitalPinStatusBytesCount);
      //Wire.write(pinStatus, PIN_COUNT);
      break;

    case EXIORDAN:
      Wire.write(_analogPinStatusBytes, _analogPinStatusBytesCount);
      break;
  }
}

void initPinStatus() {
  for (uint8_t i = 0; i < EXIO_DIGITAL_PIN_COUNT; i++) {
    _digitalPinStatus[i] = LOW;
  }

  _digitalPinStatusBytesCount = (EXIO_DIGITAL_PIN_COUNT + 7) / 8;
  _digitalPinStatusBytes = (byte *)malloc(_digitalPinStatusBytesCount);

  _analogPinStatusBytesCount = EXIO_ANALOG_PIN_COUNT * 2;

  _analogPinMap = (byte *)malloc(EXIO_ANALOG_PIN_COUNT);
  _analogPinStatusBytes = (byte *)malloc(_analogPinStatusBytesCount);
}

void getDigitalPinStatus() {
  byte byt = 0;
  byte msk = 0b00000001;

  for (uint8_t i = 0; i < _digitalPinStatusBytesCount; i++) {
    _digitalPinStatusBytes[i] = 0;
  }

  for (uint8_t i = 0; i < EXIO_DIGITAL_PIN_COUNT; i++) {
    if (_digitalPinStatus[i]) {
      _digitalPinStatusBytes[byt] ^= msk;
    }

    msk = msk << 1;

    if (!msk) {
      byt++;
      msk = 0b00000001;
    }
  }
  /*
  for(uint8_t i = 0; i < _digitalPinStatusBytesCount; i++) {
    Sprint(_digitalPinStatusBytes[i]);
    Sprint(" ");
  }
  Sprintln();
  */
}

void getAnalogPinStatus() {
  // nothing yet
}

void printPins() {
  for (uint8_t i = 0; i < EXIO_DIGITAL_PIN_COUNT; i++) {
    Sprint(EXIO_PIN_START + i);
    Sprint(" ");
  }

  Sprintln();

  for (uint8_t i = 0; i < EXIO_DIGITAL_PIN_COUNT; i++) {
    Sprint(" ");
    Sprint(_digitalPinStatus[i]);
    Sprint("  ");
  }

  Sprintln();
}

void death() {
  Sprintln("Death....");

  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}
