/*
 * slave.ino
 * IT6512 Embedded Robotics - Assignment 1: I2C-linked Arduino velocity controller
 * Student ID: 258843A
 *
 * Slave Arduino Uno (ATmega328P, 16 MHz, Proteus 8.5)
 *   - I2C slave at address 0x08 (Wire library, onReceive / onRequest)
 *   - Command frame (master -> slave): CMD, SP_HI, SP_LO, FLAGS, CHK
 *   - Status frame  (slave -> master): STATUS, PV_HI, PV_LO, DUTY, CHK
 *     CHK = byte0 XOR byte1 XOR byte2 XOR byte3
 *   - Watchdog: no valid command for 500 ms -> motor stopped, fault bit set
 *   - L298 drive: PWM on D9 (ENA), direction on D7 (IN1) and D8 (IN2)
 *   - Encoder A/B on D2/D3 (INT0/INT1), x4 quadrature decoding with direction
 *   - Speed in RPM over a fixed 50 ms sample: RPM = 60 * dn / (4 * N * T)
 *   - Serial log every 50 ms at 9600 baud: t_ms,SP,PV,duty,status
 *
 * Stage 2 (Task 3): speed is measured, but there is still no speed loop.
 * The PWM duty follows |SP| (open loop) so the measured speed can be
 * compared with the motor model at fixed duties. The PI controller
 * (Task 4) comes next.
 */

#include <Wire.h>

// ---------------- I2C protocol ----------------
const uint8_t I2C_ADDRESS   = 0x08;
const uint8_t FRAME_LEN     = 5;
const uint8_t CMD_SET_SPEED = 0x01;

const uint8_t FLAG_ENABLE      = 0x01;  // FLAGS bit 0
const uint8_t FLAG_RESET_INTEG = 0x02;  // FLAGS bit 1

const uint8_t STATUS_RUNNING   = 0x01;  // STATUS bit 0
const uint8_t STATUS_WDOG      = 0x02;  // STATUS bit 1
const uint8_t STATUS_SATURATED = 0x04;  // STATUS bit 2

// ---------------- Pins ----------------
const uint8_t PIN_PWM  = 9;   // L298 ENA
const uint8_t PIN_DIR1 = 7;   // L298 IN1
const uint8_t PIN_DIR2 = 8;   // L298 IN2
const uint8_t PIN_ENC_A = 2;  // encoder channel A (INT0)
const uint8_t PIN_ENC_B = 3;  // encoder channel B (INT1)

// ---------------- Encoder ----------------
const long ENC_PPR = 100;     // pulses per revolution (MOTOR-ENCODER property)
const long ENC_DECODE = 4;    // x4: count every edge of A and B
const int8_t ENC_SIGN = 1;    // set to -1 if forward drive reads negative

// x4 quadrature table, index = (old AB << 2) | new AB, AB = (A << 1) | B
// +1 for one step forward (A leads B), -1 for one step reverse, 0 = no move
const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// ---------------- Timing and limits ----------------
const unsigned long SAMPLE_MS   = 50;   // control / log period
const unsigned long WATCHDOG_MS = 500;  // command timeout
const int16_t SP_MAX  = 250;            // setpoint limit, RPM
const int16_t PWM_MAX = 255;            // analogWrite range

// ---------------- Shared with the I2C handlers ----------------
// The handlers only copy bytes; all checking is done in loop().
volatile uint8_t rxFrame[FRAME_LEN];
volatile uint8_t rxCount = 0;
volatile bool    rxReady = false;
volatile uint8_t txFrame[FRAME_LEN];

// ---------------- Controller state ----------------
int16_t setpointRpm   = 0;
bool    enabled       = false;
bool    watchdogFault = false;
int16_t speedRpm      = 0;      // measured speed, RPM
uint8_t duty          = 0;      // present PWM duty 0-255
bool    saturated     = false;  // used by the PI controller in Task 4
uint8_t statusByte    = 0;

// encoder count, changed only inside the encoder interrupt
volatile long encoderCount = 0;
volatile uint8_t encoderState = 0;
long lastCount = 0;

unsigned long lastValidCmdMs = 0;
unsigned long lastSampleMs   = 0;
unsigned int  badFrames      = 0;

// XOR checksum of the first four bytes of a frame
uint8_t checksum(const uint8_t *f) {
  return f[0] ^ f[1] ^ f[2] ^ f[3];
}

// ---------------- Encoder interrupt (both A and B, every edge) ----------------
void encoderISR() {
  uint8_t a = (PIND >> 2) & 0x01;          // D2 = PD2
  uint8_t b = (PIND >> 3) & 0x01;          // D3 = PD3
  uint8_t newState = (a << 1) | b;
  encoderCount += QUAD_TABLE[(encoderState << 2) | newState];
  encoderState = newState;
}

// ---------------- Speed over the fixed 50 ms sample ----------------
void measureSpeed() {
  noInterrupts();                          // read the count in one piece
  long count = encoderCount;
  interrupts();

  long dn = count - lastCount;             // counts in this sample
  lastCount = count;
  // RPM = 60 * dn / (m * N * T), T = SAMPLE_MS / 1000 s
  speedRpm = (int16_t)(ENC_SIGN * dn * 60000L / (ENC_DECODE * ENC_PPR * (long)SAMPLE_MS));
}

// ---------------- I2C handlers (kept short) ----------------
void receiveEvent(int howMany) {
  uint8_t i = 0;
  while (Wire.available()) {
    uint8_t b = Wire.read();
    if (i < FRAME_LEN) rxFrame[i] = b;
    i++;
  }
  rxCount = (uint8_t)howMany;
  rxReady = true;
}

void requestEvent() {
  Wire.write((const uint8_t *)txFrame, FRAME_LEN);
}

// ---------------- Command frame check ----------------
void handleCommand() {
  uint8_t f[FRAME_LEN];
  uint8_t n;

  noInterrupts();               // copy the frame so it cannot change mid-read
  n = rxCount;
  for (uint8_t i = 0; i < FRAME_LEN; i++) f[i] = rxFrame[i];
  rxReady = false;
  interrupts();

  // Wrong length, wrong command or wrong checksum: ignore the frame.
  // The watchdog timer is NOT reset by a bad frame.
  if (n != FRAME_LEN || f[0] != CMD_SET_SPEED || checksum(f) != f[4]) {
    badFrames++;
    Serial.print(F("# bad frame ignored, total "));
    Serial.println(badFrames);
    return;
  }

  int16_t sp = (int16_t)(((uint16_t)f[1] << 8) | f[2]);  // high byte first
  setpointRpm = constrain(sp, -SP_MAX, SP_MAX);           // clamp to +-250 RPM
  enabled = (f[3] & FLAG_ENABLE) != 0;
  if (f[3] & FLAG_RESET_INTEG) {
    // integrator reset - used once the PI controller is added (Task 4)
  }

  lastValidCmdMs = millis();
  if (watchdogFault) Serial.println(F("# command received, watchdog cleared"));
  watchdogFault = false;
}

// ---------------- Motor output ----------------
// u > 0 forward, u < 0 reverse, u = 0 stop. |u| is the PWM duty.
void setMotor(int16_t u) {
  u = constrain(u, -PWM_MAX, PWM_MAX);
  if (u > 0) {
    digitalWrite(PIN_DIR1, HIGH);
    digitalWrite(PIN_DIR2, LOW);
  } else if (u < 0) {
    digitalWrite(PIN_DIR1, LOW);
    digitalWrite(PIN_DIR2, HIGH);
  } else {
    digitalWrite(PIN_DIR1, LOW);
    digitalWrite(PIN_DIR2, LOW);
  }
  duty = (uint8_t)abs(u);
  analogWrite(PIN_PWM, duty);
}

// ---------------- Control step (every 50 ms) ----------------
void controlStep() {
  int16_t u = 0;
  if (enabled && !watchdogFault) {
    u = setpointRpm;            // Stage 2: duty follows |SP| (open loop)
  }
  setMotor(u);
}

// ---------------- Status frame for the next master read ----------------
void updateStatusFrame() {
  statusByte = 0;
  if (enabled && !watchdogFault) statusByte |= STATUS_RUNNING;
  if (watchdogFault)             statusByte |= STATUS_WDOG;
  if (saturated)                 statusByte |= STATUS_SATURATED;

  uint8_t f[FRAME_LEN];
  f[0] = statusByte;
  f[1] = (uint8_t)((uint16_t)speedRpm >> 8);
  f[2] = (uint8_t)(speedRpm & 0xFF);
  f[3] = duty;
  f[4] = checksum(f);

  noInterrupts();
  for (uint8_t i = 0; i < FRAME_LEN; i++) txFrame[i] = f[i];
  interrupts();
}

// ---------------- CSV log line ----------------
void logSample() {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(setpointRpm);
  Serial.print(',');
  Serial.print(speedRpm);
  Serial.print(',');
  Serial.print(duty);
  Serial.print(',');
  Serial.println(statusByte);
}

void setup() {
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_DIR2, OUTPUT);
  setMotor(0);

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  encoderState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  Serial.begin(9600);
  Serial.println(F("# slave 0x08 ready"));
  Serial.println(F("t_ms,SP,PV,duty,status"));

  updateStatusFrame();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  if (rxReady) handleCommand();

  // Watchdog: stop the motor if no valid command for 500 ms
  if (!watchdogFault && (millis() - lastValidCmdMs >= WATCHDOG_MS)) {
    watchdogFault = true;
    setMotor(0);
    Serial.println(F("# watchdog timeout, motor stopped"));
  }

  // Fixed 50 ms sample period
  if (millis() - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs += SAMPLE_MS;
    measureSpeed();
    controlStep();
    updateStatusFrame();
    logSample();
  }
}
