#define SERBAUDRATE 115200

#define SCDMEASUREMENTINTERVAL 2 // interval is in seconds, range is 2-1800, same amt of time added as startup delay
#define SCDALTITUDEOFFSET 1409 // measurement altitude in meters

// GPIO pin# defs
#define P_BLANK 18
#define P_BUTTON 7 // See getButtonState() before editing-- it references the port bit instead of this pin number
#define P_LOAD 14 // See tubeLatch() before editing (same as prev. comment)
#define P_SCDREADY 10

// tubeRefresh actually cycles through display multiplexing REFRESHCYCLENUM times before returning.
// this is to offset any flickering caused by things like reading button/sensors
// (by just spending *more time* on the display)
#define REFRESHCYCLENUM 50
#define BTNDEBOUNCEDUR 100
#define BTNLONGDEBOUNCEDUR 500

#define NUMMODES 4

#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SCD30.h>

const char helpMessage[] PROGMEM = "----- HELP -----\nping: gets a 'pong' reply.\nm0: sets the current data-display mode (0: CO2, 1:Celsius, 2: Fahrenheit, 3: Humidity)\np: pauses the sensor-read-display cycle (for editing display manually).\nu: unpauses the sensor-read-display cycle.\nclr: clears the display buffer.\ndb1,01101101: sets/lights-up the given bits/segments at the given index/digit in the display buffer (ex. 1: starting digit, 01101101: segment config. to display # 3).\ndv8,1,1234: display numerical value at display index. 8: starting index (8 is last digit). 1: justification (0: left, 1: right), 1234: the value to be right-justified & displayed.\nb: reads the cur. state of the button pin.\nscdRead: requests an scd-30 data read\nscdReady: checks if the scd-30 currently has unread data\nscdSensor: outputs the cur. scd-30 sensor readings\nscdCalAuto: reads whether the scd-30 is in self-calibration mode\nscdCalAuto,1: sets the scd-30 self-calibration setting (1: on, 2: off)\nscdAltOffset: reads the cur. scd-30 alt offset\nscdAltOffset,1409: sets the cur. scd-30 alt offset (set '1409' to your alt in Ms abv sea lvl)\nscdTempOffset: reads the cur. scd-30 pos. temp offset (C)\nscdTempOffset,1013: sets the cur. scd-30 pos. temp offset (C) (100's of a deg, ex. 1013 = 10.13 deg C)\nscdCalValue: reads the cur. scd-30 cal. reference ppm value (typ. 400-outside co2 conc.)\nscdForceRecalReference,402: sets the cur. scd-30 cal. to read the current conc. as the given value (ex. 400 while scd is outside)\n---------------";

Adafruit_SCD30  scd30;

// Note to reader: I prefer globals for any state-related values (or arrays) when programming small embedded stuff.

// Serial's already slow & I don't wanna do command-parsing on c-strings
String serialMessage = "";

// the tube has 9 grids-- the first grid is the preceding negative-sign & dot, so I ignore it.
// AKA I only really touch grids 1-8.
// The tube can only display on one grid at a time & must be multiplexed.
// Thus, 3 bytes for the tube buffer (9 grids + 8 segments = 17 pins stretched over the 20 bits of my shift reg chip)
// I abstracted that to a "display buffer" that can contain every segment, and tubeRefresh() fills OutBuffer with it as-needed.
byte tubeOutBuffer[] = {0, 0, 0};
// The display boots with this test "image", just one segment on each digit
byte displayBuffer[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// The grids are wired out of order on the output shift-reg, so I use this to lookup which is which.
byte tubeGridTranslationLayer[] = {4, 3, 5, 2, 6, 1, 7, 0, 8};

// utility vars for my binary-decimal conversion needed for the display
// this is only meant to handle 5 digits (as chars) with the final element being a slot for the null terminator
// (this is because I can't disable snprintf's null terminator)
char _digitArray[6];
char _digitArrayLast = 0;

// 7 segment number font: input a number as the index, get the binary state each segment should be in.
byte displayBufferNumberLookup[] = {
  0b01110111,
  0b00100100,
  0b01011101,
  0b01101101,
  0b00101110,
  0b01101011,
  0b01111011,
  0b00100101,
  0b01111111,
  0b01101111
};

// The SCD30 lib gets all sensor readings as floats,
// so I intake them as floats & process them to ints for display.
float co2ValueRaw = 0;
long co2Value = 0;

float tempValueRaw = 0;
int tempValue_Celsius = 0;
int tempValue_Fahrenheit = 0;

float humidityValueRaw = 0;
int humidityValue = 0;

// Btn debouncing
bool btnState_last = true;
unsigned long long btnState_last_started = 0;

// Display "mode": which sensor data it's currently piping to the display.
// Set to -1 to "pause" this, allowing manual disp control over serial. (with command "p")
int modeNum = 0; //0: CO2, 1:Celsius, 2: Fahrenheit, 3: Humidity
int modeNum_last = 0; // Hacky way to preserve mode across display-pauses (which are triggered by serial command for debugging)

// State variables for settings UI
int settingsPageNum = -1;
bool settingsRecalTriggered = false;

// Rather than produce a bunch of values like "isPressed" and looking for rising vs. falling edges...
// I made a pseudo-event that JUST happens on debounced-button-release.
// Only one thing should "handle" a button event, so the theory is :
// - this is set true when it's detected
// - the listener checks for it every cycle & un-sets it after consumption
// (I didn't do this structure for the serial messages because I'm lazy)
bool btnHasReleaseEvent = false;
// Used for detecting long-presses (only used in settings menu)
bool btnHasLongReleaseEvent = false;

void setup() {
  initTube();
  initSerial();
  initButton();
  initSCD();

  if (digitalRead(P_BUTTON) == LOW) settingsLoop();

  delay(1000);
  Serial.println(F("CO2 Monitor Online."));

  // turning off TX/RX LEDS
  DDRD &= ~(1 << 5);
  PORTD &= ~(1 << 5);

  DDRB &= ~(1 << 0);
  PORTB &= ~(1 << 0);
}

void settingsLoop() {
  displayClearBuffer();
  tubeRefresh();
  while (true) {
    getSerialMessage();
    handleSerialMessage();

    getButtonState();
    handleSettingsButtonPress();

    tubeRefresh();
  }
}

void loop() {
  getSerialMessage();
  handleSerialMessage();

  getButtonState();
  handleButtonPress();

  scdRefresh();

  if (modeNum >= 0) {
    displayClearBuffer();

    if (modeNum == 0) {
      getCO2Data();
      displayCO2Data();

    } else if (modeNum <= 2) {
      getTempData();
      displayTempData();

    } else if (modeNum == 3) {
      getHumidityData();
      displayHumidityData();
    }
  }

  tubeRefresh();
}

void scdRefresh() {
  if (scdIsReady()) {
    tubeBlank();
    scd30.read();
    tubeUnblank();
  }
}

bool scdIsReady() {
  return digitalRead(P_SCDREADY) == HIGH;
}

void getButtonState() {
  bool btnState = (PINE >> 6) & 1;

  if (btnState == btnState_last) return;
  // after the above line, we can ASSUME it's changed state.

  if (btnState == true) {
    if ((millis() - btnState_last_started) >= BTNLONGDEBOUNCEDUR) {
      btnHasLongReleaseEvent = true;
    }
    else if ((millis() - btnState_last_started) >= BTNDEBOUNCEDUR) {
      btnHasReleaseEvent = true;
    }
  }

  btnState_last = btnState;
  btnState_last_started = millis();
}

void handleSettingsButtonPress() {
  if (btnHasLongReleaseEvent) {
    tubeBlank();
    switch (settingsPageNum) {
      case 0:
        scd30.selfCalibrationEnabled(!scd30.selfCalibrationEnabled()); // Toggle self-calibration
        break;
      case 1:
        scd30.forceRecalibrationWithReference((int)430);
        delay(10);
        Serial.println(scd30.getForcedCalibrationReference());
        settingsRecalTriggered = true;
        break;
      default:
        break;
    }

    displaySettings();
    btnHasLongReleaseEvent = false;
    tubeUnblank();
  }

  else if (btnHasReleaseEvent) {
    tubeBlank();
    settingsRecalTriggered = false;

    settingsPageNum++;
    if (settingsPageNum > 2) {
      settingsPageNum = 0;
    }

    displaySettings();
    btnHasReleaseEvent = false;
    tubeUnblank();
  }
}

void handleButtonPress() {
  if (!btnHasReleaseEvent) return;
  btnHasLongReleaseEvent = false;
  btnHasReleaseEvent = false;


  if (modeNum < 0) {
    modeNum = 0;
    return;
  }

  modeNum++;
  if (modeNum > NUMMODES - 1) {
    modeNum = 0;
    return;
  }
}

void getHumidityData() {
  humidityValueRaw = scd30.relative_humidity;

  humidityValue = (int)round(humidityValueRaw);
}

void getTempData() {
  tempValueRaw = scd30.temperature;

  tempValue_Celsius = (int)round(tempValueRaw);
  tempValue_Fahrenheit = (int)round(tempValueRaw * 1.8 + 32);
}

void getCO2Data() {
  co2ValueRaw = scd30.CO2;

  co2Value = (int)round(co2ValueRaw);
}

void displayDrawValue(char index, unsigned long value, bool isRightJustified = false) {
  if (index < 1 || index > 8) return;
  if (value < 0 || value > 99999) return;

  toDigitArray(value);

  if (!isRightJustified) {
    for (int i = 0;
         i <= _digitArrayLast && (i + index) < 9;
         i++) {

      byte curDigit = _digitArray[i];
      displayDrawDigit(i + index, curDigit);
    }
  }

  else {
    for (int i = _digitArrayLast;
         i >= 0 && index - (_digitArrayLast - i) > 0;
         i--) {

      byte curDigit = _digitArray[i];
      displayDrawDigit(index - (_digitArrayLast - i), curDigit);
    }
  }
}

void displayHumidityData() {
  displayBuffer[1] = 0b00111110;
  displayBuffer[2] = 0b11110110;

  displayDrawValue(4, humidityValue);

  displayBuffer[6] = 0b10011100;
}

void displayTempData() {
  int value;
  if (modeNum == 1) {
    value = tempValue_Celsius;
  } else {
    value = tempValue_Fahrenheit;
  }

  displayDrawValue(1, value);

  // hacky, but since the temp was the last thing put through toDigitArray(),
  // I can use this to determine how many characters-long it is.
  int offset = _digitArrayLast + 2;

  displayBuffer[offset] = 0b00001111;
  if (modeNum == 1) {
    displayBuffer[offset + 1] = 0b01010011;
  } else {
    displayBuffer[offset + 1] = 0b00011011;
  }
}

void displayCO2Data() {
  displayBuffer[1] = 0b01010011;
  displayDrawValue(2, 0);
  displayBuffer[3] = 0b11011101;
  displayDrawValue(8, co2Value, true);
}

void displaySettings() {
  displayClearBuffer();

  switch (settingsPageNum) {
    case 0:
      displayBuffer[1] = 0b01101011;
      displayBuffer[2] = 0b01011011;
      displayBuffer[3] = 0b01010010;
      displayBuffer[4] = 0b00011011;
      displayBuffer[5] = 0b01010011;
      displayBuffer[6] = 0b00111111;
      displayBuffer[7] = 0b11010010;

      displayDrawDigit(
        8,
        0 + scd30.selfCalibrationEnabled());
      break;

    case 1:
      displayBuffer[1] = 0b00011000;
      displayBuffer[2] = 0b01011011;
      displayBuffer[3] = 0b01010011;
      displayBuffer[4] = 0b00111111;
      displayBuffer[5] = 0b11010010;
      displayDrawDigit(
        8,
        0 + settingsRecalTriggered);
      break;

    default:
      displayBuffer[1] = 0b00000001;
      displayBuffer[2] = 0b00000010;
      displayBuffer[3] = 0b00000100;
      displayBuffer[4] = 0b00001000;
      displayBuffer[5] = 0b00010000;
      displayBuffer[6] = 0b00100000;
      displayBuffer[7] = 0b01000000;
      displayBuffer[8] = 0b10000000;
  }
}

void displayDrawDigit(char index, char num) {
  if (index < 1 || index > 8) return;
  if (num < 0 || num > 9) return;

  displayBuffer[index] = displayBufferNumberLookup[num];
}

void displayClearBuffer() {
  for (int i = 0; i < 9; i++) {
    displayBuffer[i] = 0;
  }
}



void toDigitArray(unsigned long n) {
  if (n < 0 || n > 99999) {
    for (int i = 0; i < 6; i++) {
      _digitArray[i] = 10; // Since each val in this array represents a "digit", 10 is used as an "invalid" value.

    }
    _digitArrayLast = 0;
    return;
  }

  snprintf(_digitArray, 6, "%lu", n);

  bool reachedEnd = false;
  char curChar;
  for (int i = 0; i < 6; i++) {
    if (reachedEnd) {
      _digitArray[i] = 10;
      continue;
    }

    curChar = _digitArray[i];

    if (curChar == (char)'\0') {
      reachedEnd = true;
      i--;
      _digitArrayLast = i;
      continue;
    }

    _digitArray[i] = curChar - '0';
  }
}

void tubeRefresh() {
  for (int i = 0; i < REFRESHCYCLENUM; i++) {
    for (int i = 0; i < 9; i++) {
      if (displayBuffer[i] == 0) continue;
      tubeSetBuffer(i, displayBuffer[i]);
      tubeBlank();
      tubeSendBuffer();
      tubeLatch();
      tubeUnblank();
    }
  }
}

void tubeSetBuffer(char grid, byte segments) {
  grid = tubeGridTranslationLayer[grid];

  tubeResetBuffer();

  if (grid <= 5) {
    tubeOutBuffer[1] |= (0b00000100 << grid);
  } else {
    tubeOutBuffer[0] |= (1 << (grid - 6));
  }

  tubeOutBuffer[2] |= (segments << 2);
  tubeOutBuffer[1] |= (segments >> 6);
}

void tubeResetBuffer() {
  tubeOutBuffer[0] = 0;
  tubeOutBuffer[1] = 0;
  tubeOutBuffer[2] = 0;
}

void tubeSendBuffer() {
  SPI.transfer(tubeOutBuffer, 3);
}

void tubeBlank() {
  digitalWrite(18, 1);
}

void tubeUnblank() {
  digitalWrite(18, 0);
}

void tubeLatch() {
  PORTB |= 1 << 3;
  serialMessage = ""; // Adds a tiny delay, seems to be necessary
  PORTB &= !((byte)1 << 3);
}

void initSerial() {
  Serial.begin(SERBAUDRATE);
  delay(10);
  while (Serial.available()) Serial.read(); // Clear serial buffer at init
}

void initTube() {
  pinMode(P_BLANK, OUTPUT);
  pinMode(P_LOAD, OUTPUT);

  digitalWrite(P_BLANK, 0);
  digitalWrite(P_LOAD, 0);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}

void initButton() {
  pinMode(P_BUTTON, INPUT_PULLUP);
}

void initSCD() {
  pinMode(P_SCDREADY, INPUT);

  if (!scd30.begin()) {
    Serial.println(F("Failed to find SCD30 chip"));
    while (1) {
      delay(100);
    }
  }
  Wire.setClock(100000);
  delay(10);

  scd30.startContinuousMeasurement();
  delay(5);
}

void getSerialMessage() {
  serialMessage = "";

  if (!Serial.available()) return;

  char serialCurChar;
  delay(10);
  do {
    serialCurChar = Serial.read();

    if (serialCurChar == '\n') return;
    serialMessage.concat(serialCurChar);

  } while (Serial.available());

  Serial.print(F("Error: received incomplete serial message (no newline): \""));
  Serial.print(serialMessage);
  Serial.println(F("\""));
  serialMessage = "";
}

void handleSerialMessage() {
  if (serialMessage.length() <= 0) return;

  Serial.print(F("> "));
  Serial.println(serialMessage);

  // For "help" command, use necessary logic to read the help-message from progmem.
  if (serialMessage.equals(F("help"))) {
    int helpMessageLength = strlen_P(helpMessage);
    char c;

    for (int k = 0; k < helpMessageLength; k++) {
      c = pgm_read_byte_near(helpMessage + k);
      Serial.print(c);
    }
    Serial.println();

    return;
  }

  // Simple commands
  if (serialMessage.equals(F("ping"))) {
    Serial.println("pong");
    return;
  }

  if (serialMessage.equals(F("clr"))) {
    displayClearBuffer();
    Serial.println("display cleared.");
    return;
  }

  if (serialMessage.equals(F("p"))) {
    serialPauseDisplay();
    return;
  }

  if (serialMessage.equals(F("u"))) {
    serialUnpauseDisplay();
    return;
  }

  if (serialMessage.equals(F("b"))) {
    Serial.print(F("Button pin is "));
    if (digitalRead(P_BUTTON)) Serial.println(F("HIGH."));
    else Serial.println(F("LOW."));

    return;
  }

  if (serialMessage.equals(F("scdRead"))) {
    serialRequestData();
    return;
  }

  if (serialMessage.equals(F("scdReady"))) {
    serialCheckDataReady();
    return;
  }

  if (serialMessage.equals(F("scdSensor"))) {
    serialReadData();
    return;
  }

  if (serialMessage.equals(F("scdCalAuto"))) {
    Serial.print(F("scd-30 self-calibration setting: "));
    pbool(scd30.selfCalibrationEnabled());
    return;
  }

  if (serialMessage.startsWith(F("scdCalAuto,"))) {
    bool setting =  serialMessage.charAt(11) == '1';
    Serial.print(F("setting scd-30 self-calibration to: "));
    pbool(setting);
    scd30.selfCalibrationEnabled(setting);
    return;
  }

  if (serialMessage.equals(F("scdAltOffset"))) {
    Serial.print(F("scd-30 altitude offset: "));
    Serial.println(scd30.getAltitudeOffset());
    return;
  }

  if (serialMessage.startsWith(F("scdAltOffset,"))) {
    int newAltOffset =  serialMessage.substring(13).toInt();
    Serial.print(F("setting scd-30 altitude offset to: "));
    Serial.println(newAltOffset);
    scd30.setAltitudeOffset(newAltOffset);
    return;
  }

  if (serialMessage.equals(F("scdTempOffset"))) {
    Serial.print(F("scd-30 temperature offset: "));
    Serial.println(scd30.getTemperatureOffset());
    return;
  }

  if (serialMessage.startsWith(F("scdTempOffset,"))) {
    int newTempOffset =  serialMessage.substring(14).toInt();
    Serial.print(F("setting scd-30 temperature offset to: "));
    Serial.println(newTempOffset);
    scd30.setTemperatureOffset(newTempOffset);
    return;
  }

  if (serialMessage.equals(F("scdCalValue"))) {
    Serial.print(F("scd-30 calibration value: "));
    Serial.println(scd30.getForcedCalibrationReference());
    return;
  }

  if (serialMessage.startsWith(F("scdForceRecalReference,"))) {
    int newCalibrationReference =  serialMessage.substring(23).toInt();
    Serial.print(F("setting scd-30 calreference, w/ current ppm as: "));
    Serial.println(newCalibrationReference);
    scd30.forceRecalibrationWithReference(newCalibrationReference);
    return;
  }

  // Complex commands w/ arguments
  if (serialMessage.startsWith(F("dv"))) { //ex: dv8,1,99834 (5 digits max)
    serialDisplayValue(serialMessage);
    return;
  }

  if (serialMessage.startsWith(F("db"))) { //ex: db8,10101010 (8 bits, 1 for each segment)
    serialDisplayBuffer(serialMessage);
    return;
  }

  if (serialMessage.startsWith(F("m"))) { //ex: m2
    serialSetMode(serialMessage);
    return;
  }
}

void serialReadData() {
  Serial.print(F("Temperature: "));
  Serial.print(scd30.temperature);
  Serial.println(F(" degrees C"));

  Serial.print(F("Relative Humidity: "));
  Serial.print(scd30.relative_humidity);
  Serial.println(F(" %"));

  Serial.print(F("CO2: "));
  Serial.print(scd30.CO2, 3);
  Serial.println(F(" ppm"));
}

void serialCheckDataReady() {
  Serial.print(F("Sensor says ready: "));
  pbool(scdIsReady());
}

void serialRequestData() {
  scd30.read();
  Serial.println(F("Initiated sensor read."));
}

void serialSetMode(String message) {
  int mode =  message.substring(1).toInt();

  Serial.print(F("Setting mode to "));
  Serial.println(mode);

  modeNum = mode;
  modeNum_last = mode;
}

void serialPauseDisplay() {
  modeNum_last = modeNum;
  modeNum = -1;
  Serial.println(F("Display paused."));
}

void serialUnpauseDisplay() {
  modeNum = (modeNum_last != -1) ? modeNum_last : 0;
  Serial.println(F("Display unpaused."));
}

void serialDisplayValue(String message) {
  Serial.println(F("received display-value command."));

  byte gridNum = message.charAt(2) - '0';
  bool isRightJustified = message.charAt(4) == '1';
  unsigned long value =  message.substring(6).toInt();

  // Provide the programmer with a line of code that does what they just commanded
  // This allows me to draw stuff on the display in realtime, then copy what I just drew into sourcecode
  Serial.print(F("parsed: "));
  Serial.print(F("displayDrawValue("));
  Serial.print(gridNum);
  Serial.print(F(", "));
  Serial.print(value);
  Serial.print(F(", "));
  Serial.print(isRightJustified);
  Serial.println(F(");"));

  displayDrawValue(gridNum, value, isRightJustified);
}

void serialDisplayBuffer(String message) {
  Serial.println(F("received display-buffer command."));

  byte gridNum = message.charAt(2) - '0';
  byte segments = strtol(message.substring(4).c_str(), NULL, 2);

  // See prev comment block
  Serial.print(F("parsed: "));
  Serial.print(F("displayBuffer["));
  Serial.print(gridNum);
  Serial.print(F("] = "));
  Serial.print(segments, BIN);
  Serial.println(F(";"));

  displayBuffer[gridNum] = segments;
}

void pbool(bool inp) {
  if (inp) {
    Serial.println(F("TRUE"));
  } else {
    Serial.println(F("FALSE"));
  }
}
