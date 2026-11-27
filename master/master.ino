/*
 * master.ino
 * IT6512 Embedded Robotics - Assignment 1: I2C-linked Arduino velocity controller
 * Student ID: 258843A
 *
 * Master Arduino Uno (ATmega328P, 16 MHz, Proteus 8.5)
 *   - Reads the speed setpoint from the pot on A0 (0..250 RPM) and the
 *     direction from the switch on D6 (LOW = forward, HIGH = reverse)
 *   - Every 100 ms: writes a command frame to the slave (0x08), then reads
 *     back a 5-byte status frame and checks its XOR checksum
 *   - Shows SP, PV, duty and slave state on the 16x2 LCD (LM016L)
 *   - Never drives the motor itself
 *
 * Command frame: CMD(0x01), SP_HI, SP_LO, FLAGS, CHK
 * Status frame : STATUS, PV_HI, PV_LO, DUTY, CHK
 * CHK = byte0 XOR byte1 XOR byte2 XOR byte3, values are int16 high byte first
 *
 * Test keys typed in the master's Virtual Terminal (9600 baud):
 *   S          step test: 0 RPM for 1 s, then 150 RPM          (Task 4)
 *   T          sequence 150 -> 80 -> -100 -> 0 RPM, 2 s each    (Task 5)
 *   P          pause / resume I2C sending (watchdog test)       (Task 2)
 *   X          bad-checksum mode on / off (checksum test)       (Task 2)
 *   M          back to pot + switch control
 *   <number>   e.g. 120 or -80 then Enter: fixed setpoint
 *
 * Only the Arduino core and Wire library are used, so the LCD is driven by
 * the short 4-bit routines at the end of this file.
 */

#include <Wire.h>

// ---------------- I2C protocol ----------------
const uint8_t SLAVE_ADDRESS = 0x08;
const uint8_t FRAME_LEN     = 5;
const uint8_t CMD_SET_SPEED = 0x01;

const uint8_t FLAG_ENABLE      = 0x01;  // FLAGS bit 0
const uint8_t FLAG_RESET_INTEG = 0x02;  // FLAGS bit 1

const uint8_t STATUS_RUNNING   = 0x01;  // STATUS bit 0
const uint8_t STATUS_WDOG      = 0x02;  // STATUS bit 1
const uint8_t STATUS_SATURATED = 0x04;  // STATUS bit 2

// ---------------- Pins ----------------
const uint8_t PIN_POT    = A0;
const uint8_t PIN_DIR_SW = 6;
const uint8_t LCD_RS = 12, LCD_E = 11;
const uint8_t LCD_D4 = 5, LCD_D5 = 4, LCD_D6 = 3, LCD_D7 = 2;

// ---------------- Timing ----------------
const unsigned long COMMAND_MS = 100;  // command period (brief)
const unsigned long LOG_MS     = 500;  // master log period
const int16_t SP_MAX = 250;

// Task 5 setpoint sequence
const int16_t SEQ_SP[] = {150, 80, -100, 0};
const uint8_t SEQ_LEN = 4;
const unsigned long SEQ_STEP_MS = 2000;

// ---------------- Setpoint source ----------------
enum Mode { MODE_POT, MODE_FIXED, MODE_STEP, MODE_SEQUENCE };
Mode mode = MODE_POT;
unsigned long modeStartMs = 0;
int16_t fixedSp = 0;

// ---------------- Link state ----------------
bool paused = false;          // P key: stop sending (watchdog test)
bool badChecksumMode = false; // X key: send frames with a wrong checksum
bool resetIntegNext = true;   // FLAGS bit 1 on the next frame
int16_t setpointRpm = 0;
int16_t lastSentSp = 0;

uint8_t slaveStatus = 0;
int16_t slaveSpeed = 0;
uint8_t slaveDuty = 0;
bool linkOk = false;
unsigned int commErrors = 0;

unsigned long lastCommandMs = 0;
unsigned long lastLogMs = 0;

char inputBuf[8];
uint8_t inputLen = 0;

uint8_t checksum(const uint8_t *f) {
  return f[0] ^ f[1] ^ f[2] ^ f[3];
}

// I2C clock = 10 kHz: SCL = 16 MHz / (16 + 2 * TWBR * 4) with prescaler 4.
// At 100 kHz the slave could not load its first reply byte in time in the
// Proteus 8.5 model (the reply arrived one bit late), so the bus is slowed.
// One command + status exchange still takes only about 11 ms.
void setI2cClock() {
  TWSR = (TWSR & 0xFC) | 0x01;  // prescaler = 4
  TWBR = 198;
}

// ---------------- Setpoint ----------------
void startMode(Mode m) {
  mode = m;
  modeStartMs = millis();
  resetIntegNext = true;      // new test: clear the slave's integrator
}

int16_t readSetpoint() {
  unsigned long t = millis() - modeStartMs;
  switch (mode) {
    case MODE_FIXED:
      return fixedSp;
    case MODE_STEP:
      return (t < 1000) ? 0 : 150;
    case MODE_SEQUENCE: {
      unsigned long i = t / SEQ_STEP_MS;
      return (i < SEQ_LEN) ? SEQ_SP[i] : 0;
    }
    default: {                // MODE_POT
      int16_t mag = map(analogRead(PIN_POT), 0, 1023, 0, SP_MAX);
      return (digitalRead(PIN_DIR_SW) == HIGH) ? -mag : mag;
    }
  }
}

// ---------------- I2C transaction: command, then status ----------------
void exchangeWithSlave() {
  uint8_t f[FRAME_LEN];
  uint8_t flags = FLAG_ENABLE;

  // reset the integrator when the direction changes
  if ((setpointRpm > 0 && lastSentSp < 0) || (setpointRpm < 0 && lastSentSp > 0)) {
    resetIntegNext = true;
  }
  if (resetIntegNext) flags |= FLAG_RESET_INTEG;

  f[0] = CMD_SET_SPEED;
  f[1] = (uint8_t)((uint16_t)setpointRpm >> 8);
  f[2] = (uint8_t)(setpointRpm & 0xFF);
  f[3] = flags;
  f[4] = checksum(f);
  if (badChecksumMode) {
    f[4] ^= 0xFF;             // deliberately wrong checksum (test only)
  }

  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(f, FRAME_LEN);
  if (Wire.endTransmission() != 0) commErrors++;   // NACK or bus timeout
  resetIntegNext = false;
  lastSentSp = setpointRpm;

  // Give the slave time to finish its onReceive handler before reading.
  // The AVR Wire slave releases the bus before that handler has run, and
  // the Proteus model does not stretch the clock, so an immediate read
  // returned corrupted bytes.
  delayMicroseconds(500);

  // read the 5-byte status frame
  uint8_t s[FRAME_LEN];
  uint8_t n = Wire.requestFrom(SLAVE_ADDRESS, FRAME_LEN);
  for (uint8_t i = 0; i < n && i < FRAME_LEN; i++) s[i] = Wire.read();

  if (n == FRAME_LEN && checksum(s) == s[4]) {
    slaveStatus = s[0];
    slaveSpeed  = (int16_t)(((uint16_t)s[1] << 8) | s[2]);
    slaveDuty   = s[3];
    linkOk = true;
  } else {
    commErrors++;
    linkOk = false;
  }

  // a bus timeout resets the TWI hardware to 100 kHz, so set 10 kHz again
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    setI2cClock();
  }
}

// ---------------- Serial test keys ----------------
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= 'a' && c <= 'z') c -= 32;   // accept lower case too

    if (c == 'S') {
      startMode(MODE_STEP);
      Serial.println(F("# step test: 0 RPM for 1 s, then 150 RPM"));
    } else if (c == 'T') {
      startMode(MODE_SEQUENCE);
      Serial.println(F("# sequence: 150, 80, -100, 0 RPM (2 s each)"));
    } else if (c == 'M') {
      startMode(MODE_POT);
      Serial.println(F("# pot and switch control"));
    } else if (c == 'P') {
      paused = !paused;
      if (!paused) resetIntegNext = true;
      Serial.println(paused ? F("# I2C paused") : F("# I2C resumed"));
    } else if (c == 'X') {
      badChecksumMode = !badChecksumMode;
      Serial.println(badChecksumMode ? F("# bad-checksum frames ON")
                                     : F("# bad-checksum frames OFF"));
    } else if ((c >= '0' && c <= '9') || c == '-') {
      if (inputLen < sizeof(inputBuf) - 1) inputBuf[inputLen++] = c;
    } else if ((c == '\r' || c == '\n') && inputLen > 0) {
      inputBuf[inputLen] = '\0';
      fixedSp = atoi(inputBuf);         // the slave clamps it to +-250
      inputLen = 0;
      startMode(MODE_FIXED);
      Serial.print(F("# fixed setpoint "));
      Serial.println(fixedSp);
    }
  }
}

// ---------------- LCD (HD44780, 4-bit, write only) ----------------
void lcdPulse() {
  digitalWrite(LCD_E, HIGH);
  delayMicroseconds(1);
  digitalWrite(LCD_E, LOW);
  delayMicroseconds(50);
}

void lcdNibble(uint8_t n) {
  digitalWrite(LCD_D4, n & 0x01);
  digitalWrite(LCD_D5, (n >> 1) & 0x01);
  digitalWrite(LCD_D6, (n >> 2) & 0x01);
  digitalWrite(LCD_D7, (n >> 3) & 0x01);
  lcdPulse();
}

void lcdSend(uint8_t value, bool isData) {
  digitalWrite(LCD_RS, isData ? HIGH : LOW);
  lcdNibble(value >> 4);
  lcdNibble(value & 0x0F);
}

void lcdCommand(uint8_t cmd) {
  lcdSend(cmd, false);
  if (cmd <= 0x03) delay(2);   // clear and home are slow
}

void lcdInit() {
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_E, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D7, OUTPUT);
  delay(50);
  digitalWrite(LCD_RS, LOW);
  lcdNibble(0x03); delay(5);   // HD44780 reset sequence
  lcdNibble(0x03); delayMicroseconds(150);
  lcdNibble(0x03);
  lcdNibble(0x02);             // 4-bit mode
  lcdCommand(0x28);            // 2 lines, 5x8 font
  lcdCommand(0x0C);            // display on, cursor off
  lcdCommand(0x06);            // cursor moves right
  lcdCommand(0x01);            // clear
}

void lcdPrintLine(uint8_t row, const char *text) {
  lcdCommand(row == 0 ? 0x80 : 0xC0);
  for (uint8_t i = 0; i < 16; i++) {
    lcdSend(*text ? *text++ : ' ', true);   // pad to 16 characters
  }
}

void updateLcd() {
  char line[17];
  const char *state;

  if (paused)                              state = "PAUSED";
  else if (!linkOk)                        state = "NO LINK";
  else if (slaveStatus & STATUS_WDOG)      state = "WDOG";
  else if (slaveStatus & STATUS_SATURATED) state = "SAT";
  else if (slaveStatus & STATUS_RUNNING)   state = "RUN";
  else                                     state = "STOP";

  snprintf(line, sizeof(line), "SP:%+4d PV:%+4d", setpointRpm, slaveSpeed);
  lcdPrintLine(0, line);
  snprintf(line, sizeof(line), "D:%3u %s", slaveDuty, state);
  lcdPrintLine(1, line);
}

// ---------------- Master log ----------------
void logStatus() {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(setpointRpm);
  Serial.print(',');
  Serial.print(slaveSpeed);
  Serial.print(',');
  Serial.print(slaveDuty);
  Serial.print(',');
  Serial.print(slaveStatus);
  Serial.print(',');
  Serial.println(commErrors);
}

void setup() {
  pinMode(PIN_DIR_SW, INPUT);
  Serial.begin(9600);
  Wire.begin();                // master
  setI2cClock();               // 10 kHz, see setI2cClock()
  Wire.setWireTimeout(25000, true);  // 25 ms: never hang on a stuck bus
  lcdInit();
  lcdPrintLine(0, "IT6512 258843A");
  lcdPrintLine(1, "I2C velocity");
  delay(500);

  Serial.println(F("# master ready. Keys: S T P X M, or a setpoint + Enter"));
  Serial.println(F("t_ms,SP,PV,duty,status,errors"));
  modeStartMs = millis();
}

void loop() {
  handleSerial();

  // command + status exchange every 100 ms
  if (millis() - lastCommandMs >= COMMAND_MS) {
    lastCommandMs += COMMAND_MS;
    setpointRpm = readSetpoint();
    if (!paused) exchangeWithSlave();
    updateLcd();
  }

  if (millis() - lastLogMs >= LOG_MS) {
    lastLogMs += LOG_MS;
    logStatus();
  }
}
