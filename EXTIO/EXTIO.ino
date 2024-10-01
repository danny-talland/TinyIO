#include <Wire.h>
#include <SPI.h>
#include <NRFLite.h>
#include <EEPROM.h>

// Config I2C
#define EXIO_ADDRESS 0x65          // Extender address
#define EXIO_PIN_START 400         // First VPIN
#define EXIO_DIGITAL_PIN_COUNT 50  // # of pins (digital input)
#define EXIO_ANALOG_PIN_COUNT 0    // # of pins (analog input)
#define EXIO_PIN_TABLE_SIZE 50     // which vpin @ which radio (single vpin can have multiple in/out)

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
#define Sprintln(a) (Serial.println(F(a)))
#define Sprintlnn(a) (Serial.println(a))
#define Sprint(a) (Serial.print(F(a)))
#define Sprintn(a) (Serial.print(a))
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
  uint8_t index;
  uint8_t offset;
  uint8_t function;
  uint8_t radio;
};

// Global vars
uint8_t outOP;
uint8_t _digitalPinStatus[EXIO_DIGITAL_PIN_COUNT];
uint8_t _digitalPinStatusBytesCount;
uint8_t *_digitalPinStatusBytes;

uint8_t _radiosOnline[EEPROM_RADIO_COUNT + 1];
NRFLite radio;
RadioPacket _radioData;
VPinConfig vPinConfig[EXIO_PIN_TABLE_SIZE];
uint32_t time;


void setup() {
  Sbegin(115200);
  Sprintln("--- EXTIO ---");
  Sprintn("Version: ");
  Sprintn(VERSION_MAJOR);
  Sprint(".");
  Sprintn(VERSION_MINOR);
  Sprint(".");
  Sprintlnn(VERSION_PATCH);

  setRadioIds();
  initVPinConfig();
  initPinStatus();

  Wire.begin(EXIO_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  if (!radio.init(RADIO_ID, RADIO_PIN_CE, RADIO_PIN_CSN, NRFLite::BITRATE250KBPS, RADIO_CHAN)) {
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
      //   sendPWMOutput(0, 40, 20);
      // sendPWMOutput(3, 255, 50);
    } else {

      //sendPWMOutput(3, 0, 30);
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
  uint8_t analogPinMapIndex;

  while (radio.hasData()) {
    radio.readData(&_radioData);

    switch (_radioData.message) {
      case PACKET_INIT:
        if (_radioData.fromRadioId == 255) {
          Sprint("R: Radio init: ask for new ID and vpin offset ");
        } else {
          Sprint("R: Radio init: wants to use ID ");
          Sprintn(_radioData.fromRadioId);
          Sprint(" and asks for vpin offset ");

          clearRadioVPinConfig(_radioData.fromRadioId);
        }

        _radioData.data[0] = getRadioId(_radioData.fromRadioId);
        _radioData.data[1] = lowByte(EXIO_PIN_START);
        _radioData.data[2] = highByte(EXIO_PIN_START);

        Sprint("(id:");
        Sprintn(_radioData.data[0]);
        Sprint(", vpin offset:");
        Sprintn(EXIO_PIN_START);
        Sprint(")");

        if (radio.send(_radioData.fromRadioId, &_radioData, sizeof(_radioData))) {
          _radiosOnline[_radioData.data[0]] = 1;
          Sprintln("...done");
        } else
          Sprintln("...failed");

        break;

      case PACKET_VPIN_CONFIG:
        Sprint("R: Radio ");
        Sprintn(_radioData.fromRadioId);
        Sprint(" config: services vpin ");
        Sprintn(_radioData.data[0] + EXIO_PIN_START);

        if (_radioData.data[2] == PINCONFIG_DIGITAL_INPUT) Sprint(" as digital IN...");
        if (_radioData.data[2] == PINCONFIG_DIGITAL_OUTPUT) Sprint(" as digital OUT...");
        if (_radioData.data[2] == PINCONFIG_PWM_LED_OUTPUT) Sprint(" as PWM LED OUT...");
        if (_radioData.data[2] == PINCONFIG_PWM_SERVO_OUTPUT) Sprint(" as PWM SERVO OUT...");

        if (newVPinConfig(_radioData.data[0], _radioData.data[1], _radioData.data[2], _radioData.fromRadioId))
          Sprintln("saved");
        else {
          Sprintln("not saved (pin config table full)");
        }

        break;

      case PACKET_UPDATE_DIGITAL:
        // Received a digital pin update, store in the digital pin byte array
        Sprint("R: (DIGITAL) radio ");
        Sprintn(_radioData.fromRadioId);
        Sprint(", vpin ");
        Sprintn(_radioData.data[0] + EXIO_PIN_START);
        Sprint(" state update ");
        Sprintn(_digitalPinStatus[_radioData.data[0]]);
        Sprint(" > ");
        Sprintlnn(_radioData.data[1]);

        _digitalPinStatus[_radioData.data[0]] = _radioData.data[1];

        break;

      case PACKET_JUST_SAY_HI:
        Sprint("R: Radio ");
        Sprintn(_radioData.fromRadioId);
        Sprint(" says hi, responding (");
        Sprintn(_radioData.data[0]);
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
        Sprint("R: Unknown data, ignoring! (radio:");
        Sprintn(_radioData.fromRadioId);
        Sprint(", message:");
        Sprintn(_radioData.message);
        Sprint(", 0:");
        Sprintn(_radioData.data[0]);
        Sprint(", 1:");
        Sprintn(_radioData.data[1]);
        Sprint(", 2:");
        Sprintn(_radioData.data[2]);
        Sprint(", 3:");
        Sprintn(_radioData.data[3]);
        Sprintln(")");
    }
  }
}

void sendDigitalOutput(uint8_t offset, uint8_t state) {
  uint8_t found = 0;

  String msg;

  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    if (vPinConfig[i].function != PINCONFIG_NOT_USED && vPinConfig[i].offset == offset) {
      if (vPinConfig[i].function == PINCONFIG_DIGITAL_OUTPUT) {
        found = 1;

        Sprint("S: radio ");
        Sprintn(vPinConfig[i].radio);
        Sprint(", vpin ");
        Sprintn(offset + EXIO_PIN_START);
        Sprint(", state ");
        serialHighOrLow(state);

        _radioData.fromRadioId = vPinConfig[i].radio;
        _radioData.message = PACKET_UPDATE_DIGITAL;
        _radioData.data[0] = vPinConfig[i].index;
        _radioData.data[1] = state;

        if (radio.send(vPinConfig[i].radio, &_radioData, sizeof(_radioData)))
          Sprintln("...done");
        else
          Sprintln("...failed");
      } else {
        Sprint("E: radio ");
        Sprintn(vPinConfig[i].radio);
        Sprint(", vpin ");
        Sprintn(offset + EXIO_PIN_START);
        Sprint("...tried to set as digital OUT configured as ");
        serialPinConfiguration(vPinConfig[i].function);
        Sprintln(", ignored");
      }
    }
  }

  if (!found) {
    Sprint("E: vpin ");
    Sprintn(offset + EXIO_PIN_START);
    Sprintln(" was not found to be online, command ignored...");
  }
}



void sendPWMOutput(uint8_t offset, uint8_t value, uint8_t duration) {
  uint8_t found = 0;
  float usDelayPrecise;
  uint16_t usDelay;

  RadioPacket radioData;

  String msg;
  String msgCorrected = "";

  // Duration on the TinyIO side must fit in a byte (32ms * duration byte)
  if (duration > 80) {
    msgCorrected = " (adjusted from " + String(duration * 100) + "ms to max duration)";
    duration = 80;
  }

  // Instant movement always takes at least 32 ms (prevent div by 0)
  if (!duration) duration = 1;

  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    if (vPinConfig[i].function != PINCONFIG_NOT_USED && vPinConfig[i].offset == offset) {
      if (vPinConfig[i].function == PINCONFIG_PWM_LED_OUTPUT || vPinConfig[i].function == PINCONFIG_PWM_SERVO_OUTPUT) {
        found = 1;

        msg = "S: radio " + String(vPinConfig[i].radio) + ", vpin " + String(offset + EXIO_PIN_START);

        if (vPinConfig[i].function == PINCONFIG_PWM_LED_OUTPUT) {
          radioData.fromRadioId = vPinConfig[i].radio;
          radioData.message = PACKET_UPDATE_ANALOG;
          radioData.data[0] = vPinConfig[i].index;
          radioData.data[1] = value;
          radioData.data[2] = (duration * 100) / 32;
          radioData.data[3] = 0;

          msg += ", PWM LED value " + String(value);
        } else {
          if (value > 180) value = 180;

          usDelayPrecise = ((SERVO_PWM_MAX_DURATION - SERVO_PWM_MIN_DURATION) / (float)180) * value;
          usDelay = (uint16_t)usDelayPrecise;

          radioData.fromRadioId = vPinConfig[i].radio;
          radioData.message = PACKET_UPDATE_ANALOG;
          radioData.data[0] = vPinConfig[i].index;
          radioData.data[1] = lowByte(usDelay);
          radioData.data[2] = (duration * 100) / 32;
          radioData.data[3] = highByte(usDelay);

          msg += ", PWM SERVO angle " + String(value) + "deg (pulse width: " + String(SERVO_PWM_MIN_DURATION + usDelay) + "us)";
        }

        msg += ", duration " + String(duration * 100) + "ms" + msgCorrected + "...";

        if (radio.send(vPinConfig[i].radio, &radioData, sizeof(radioData)))

          msg += "done";

        else
          msg += "failed";

        Serial.println(msg);
      } else {
        Sprint("E: radio ");
        Sprintn(vPinConfig[i].radio);
        Sprint(", vpin ");
        Sprintn(offset + EXIO_PIN_START);
        Sprint("...tried to set as PWM OUT configured as ");
        serialPinConfiguration(vPinConfig[i].function);
        Sprintln(", ignored");
      }
    }
  }

  if (!found) {
    Sprint("E: vpin ");
    Sprintn(offset + EXIO_PIN_START);
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
    /*
    Sprint(i);
    Sprint(":");
    Sprint(n);
    Sprint(" - ");*/
  }
  // Sprintln();

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
    if (i == EXIO_PIN_TABLE_SIZE) return 0;

    if (vPinConfig[i].function == PINCONFIG_NOT_USED) break;
    i++;
  }

  vPinConfig[i].index = remoteIndex;
  vPinConfig[i].offset = offset;
  vPinConfig[i].function = function;
  vPinConfig[i].radio = radio;

  return 1;
}

void clearRadioVPinConfig(uint8_t radio) {
  for (uint8_t i = 0; i < EXIO_PIN_TABLE_SIZE; i++) {
    if (vPinConfig[i].radio == radio) {
      vPinConfig[i].function = PINCONFIG_NOT_USED;
    }
  }
}

void receiveData(int numBytes) {
  if (numBytes == 0) {
    return;
  }

  uint8_t buffer[numBytes];

  buffer[0] = Wire.read();

  for (uint8_t i = 1; i < numBytes; i++) {
    buffer[i] = Wire.read();

    /*   if (buffer[0] = EXIORDAN) {
      if (i == 1) {
        SprintHEX(buffer[0]);
        Sprint(" - ");
      }

      Sprint(buffer[i]);
      Sprint(" (0x");
      SprintHEX(buffer[i]);
      Sprint(") ");
      Sprint(" - ");
    //}
    */
  }

  //// if (buffer[0] != EXIORDD /* && buffer[0] != EXIORDAN*/) {
  // Sprintln("EOD");
  //  }

  switch (buffer[0]) {
    case EXIOINIT:
      Sprintln("2: EXIOINIT, init device...");

      if (buffer[1] == EXIO_DIGITAL_PIN_COUNT && ((buffer[3] << 8) + buffer[2]) == EXIO_PIN_START) {
        outOP = EXIOPINS;
      } else {
        outOP = 0;
      }
      break;

    case EXIOINITA:
      Sprintln("2: EXIOINITA, request analog pin mappings...");
      outOP = EXIOINITA;
      break;

    case EXIODPUP:
      Sprintln("2: EXIODPUP, request pin pullup configuration...");
      outOP = EXIORDY;
      break;

    case EXIOVER:
      Sprintln("2: EXIOVER, request version number...");
      outOP = EXIOVER;
      break;

    case EXIOWRD:
      // Write digital pin status
      Sprintln("Writing digital pin");
      sendDigitalOutput(buffer[1], buffer[2]);
      outOP = EXIORDY;
      break;

    case EXIOENAN:
      // Enable analog input
      outOP = EXIOERR;
      Sprintln("HEREHEREHERE!");
      for (uint8_t i = 0; i < EXIO_ANALOG_PIN_COUNT; i++) {
        Sprint("I:");
        Sprintn(i);
        Sprint(" P:");
      }
      break;

    case EXIORDD:
      // Read digital pin status
      getDigitalPinStatus();
      outOP = EXIORDD;
      break;

    case EXIORDAN:
      //  Sprint("EXIORDAN - ");

      for (uint8_t i = 1; i < numBytes; i++) {

        Sprintn(buffer[i]);
        Sprint(" (0x");
        SprintHEX(buffer[i]);
        Sprint(") ");
        Sprint(" - ");
      }
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
      Sprintln("");
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
      break;

    case EXIORDY:
      Wire.write(EXIORDY);
      break;

    case EXIOVER:
      Wire.write(VERSION_MAJOR);
      Wire.write(VERSION_MINOR);
      Wire.write(VERSION_PATCH);

      Sprintln("2: EXIO setup complete....");
      break;

    case EXIORDD:
      Wire.write(_digitalPinStatusBytes, _digitalPinStatusBytesCount);
      //Wire.write(pinStatus, PIN_COUNT);
      break;
  }
}

void initPinStatus() {
  for (uint8_t i = 0; i < EXIO_DIGITAL_PIN_COUNT; i++) {
    _digitalPinStatus[i] = LOW;
  }

  _digitalPinStatusBytesCount = (EXIO_DIGITAL_PIN_COUNT + 7) / 8;
  _digitalPinStatusBytes = (byte *)malloc(_digitalPinStatusBytesCount);


  // TEST
  // _analogPinMap[0] = 1;
  // _analogPinMap[1] = 2;
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
    Sprintn(EXIO_PIN_START + i);
    Sprint(" ");
  }

  Sprintln("");

  for (uint8_t i = 0; i < EXIO_DIGITAL_PIN_COUNT; i++) {
    Sprint(" ");
    Sprintn(_digitalPinStatus[i]);
    Sprint("  ");
  }

  Sprintln("");
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
