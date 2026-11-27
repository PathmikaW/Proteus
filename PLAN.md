# IT6512 Assignment 1 – I2C-Linked Arduino Velocity Controller (Proteus)

Plan of work, kept strictly to the brief:
`../Assignment1- Proteus/Assignment_I2C_Arduino_Velocity_Controller_Proteus.pdf`

Marks: Task 1 Circuit 15 · Task 2 I2C 20 · Task 3 Speed 15 · Task 4 Controller 25 · Task 5 Tracking/load 10 · Task 6 Report/video/viva 15 = **100**

---

## 1. Toolchain and deviations from the brief

| Brief says | We use | Reason (stated in the report) |
|---|---|---|
| Proteus 8 | Proteus 8 Professional **8.5 SP0** | – |
| Arduino Uno "from your Arduino library" | **ATMEGA328P** (the Uno's MCU) at 16 MHz | Proteus 8.5 has no Arduino Uno model. Same program-file and 16 MHz clock settings as setup step 2 |
| Arduino IDE 2.x | Arduino IDE **1.8.19** (AVR core 1.8.3) | Installed version. *Export compiled Binary* gives the same `.hex` |
| 100 pulses per rev (or nearest) | **100 PPR** (MOTOR-ENCODER allows 1–360) | Exact |
| Plot in Excel or Python | Python 3.10 (pandas, matplotlib) | – |

**Library rule (brief):** only Arduino core and Wire. No PID, encoder or LCD libraries, so the master drives the LM016L with its own short 4-bit routine.

**Academic integrity (brief):** AI assistance must be acknowledged in the report.

---

## 2. Repository layout (only what the deliverables need)

```
Proteus/
├── 258843A.pdsprj   Proteus project (deliverable)
├── master/master.ino            + master.ino.standard.hex (deliverable)
├── slave/slave.ino              + slave.ino.standard.hex  (deliverable)
├── data/                        CSV logs copied from the slave's Virtual Terminal
├── scripts/plot.py              plots + rise / overshoot / settling / SSE
├── report/                      figures and report source (PDF is the deliverable)
├── PLAN.md
└── README.md
```

Folders are created only when they get content. The demo video is submitted in the zip and is not kept in git.

Arduino requires each sketch to sit in a folder with the same name (`master/master.ino`). The `.hex` is exported next to the `.ino`, and the Proteus program file points to `master\master.ino.standard.hex`, a sub-folder of the project, so the zip opens and runs without changes.

---

## 3. Design (from the brief)

### Pin allocation

| Board | Arduino pin | ATMEGA328P pin (Proteus) | Net | Brief reference |
|---|---|---|---|---|
| Slave | D2 | PD2/INT0 | ENC_A | "encoder A, B to pins 2, 3" |
| Slave | D3 | PD3/INT1 | ENC_B | " |
| Slave | D9 | PB1/OC1A | PWM | "PWM on pin 9" |
| Slave | D7, D8 | PD7, PB0 | DIR1, DIR2 | "DIR on pins 7, 8" |
| Both | A4, A5 | PC4/SDA, PC5/SCL | SDA, SCL | "A4 = SDA, A5 = SCL, 4.7 kΩ pull-ups" |
| Both | D0, D1 | PD0/RXD, PD1/TXD | – | Virtual Terminal, 9600 baud |
| Master | A0 | PC0/ADC0 | SP_POT | "Reads setpoint pot (A0)" |
| Master | D6 | PD6 | DIR_SW | Direction / enable switch |
| Master | D12, D11 | PB4, PB3 | LCD_RS, LCD_E | 16×2 LCD |
| Master | D5, D4, D3, D2 | PD5, PD4, PD3, PD2 | LCD_D4…D7 | 16×2 LCD |

(The master's D2/D3 are free because only the slave reads the encoder.)

### I2C frames (exactly as in the brief)
- **Command**, master → 0x08, every 100 ms: `CMD=0x01 | SP_HI | SP_LO | FLAGS | CHK`. FLAGS bit0 = enable, bit1 = reset integrator.
- **Status**, 5-byte read from 0x08 after each command: `STATUS | PV_HI | PV_LO | DUTY | CHK`. STATUS bit0 = running, bit1 = watchdog fault, bit2 = output saturated.
- `CHK = b0 ^ b1 ^ b2 ^ b3`. Signed 16-bit, high byte first. SP clamped to −250…+250.
- A wrong length or checksum means the frame is ignored and does **not** reset the watchdog.
- `onReceive` / `onRequest` only copy bytes. All maths happens in `loop()`.
- Watchdog: no valid command for 500 ms → motor stopped, fault bit set.

### Master operation
- Normal: the pot gives |SP| (0–250 RPM), the switch gives the direction. The LCD shows SP, PV and duty.
- Typed keys in the master's Virtual Terminal, needed to produce the brief's evidence repeatably:
  - `S`: 0 → 150 RPM step (Task 4, and the Task 5 load test)
  - `T`: sequence 150 → 80 → −100 → 0 RPM (Task 5)
  - `P`: pause/resume sending, "watchdog stopping the motor when the master is paused" (Task 2)
  - `X`: send one frame with a wrong checksum, "discard bad frames" proof (Task 2)
  - `M`: back to pot control

### Speed measurement
ω_RPM = 60·Δn / (m·N·T), N = 100, T = 0.05 s → x1 = 12, x2 = 6, **x4 = 3 RPM/count**.

x4 is chosen because the SSE target is ≤ 3 RPM. A and B both use `CHANGE` interrupts, and the direction comes from the quadrature phase. Counts are read with interrupts briefly disabled.

### Controller (slave, every 50 ms)
- `CONTROL_MODE` constant in `slave.ino`:
  - `OPEN_LOOP`: duty = |SP| (SP 50…250 used as a duty value for the open-loop table)
  - `P`
  - `PI`
  - `PID` only if PI cannot meet the targets
- Brief formula: u[k] = Kp·e[k] + Ki·T·Σe[j]. Output limited to ±255 (|u| on D9, sign on D7/D8).
- Anti-windup (brief): stop adding to Σe while the output is saturated.
- Targets for 0 → 150 RPM: **OS ≤ 10 %, Ts(±5 %) ≤ 1.0 s, SSE ≤ 3 RPM**
- Slave log line every 50 ms: `t_ms,SP,PV,duty` (brief: "log time, setpoint and measured speed")

---

## 4. Tasks, steps and commits

🧑 = you in Proteus / Arduino IDE · 🤖 = Claude (code, scripts, report draft)

### Task 1 – Circuit build (15)
1. 🧑 Proteus 8.5:
   - *File → New Project*
   - Name `IT6512_I2C_Velocity`, Path = this repo folder
   - *Landscape A3* template
   - *Do not create a PCB layout*
   - *No Firmware Project*
   - *Finish*
2. 🧑 Press **P** and pick, as in the brief's parts table:
   - ATMEGA328P ×2 (references MASTER, SLAVE)
   - L298
   - MOTOR-ENCODER
   - LM016L
   - POT-HG
   - SW-SPDT
   - RES ×2 (4.7k)
   - 1N4007 ×4
   - POWER terminal `+12V`
3. 🧑 From *Virtual Instruments Mode*: I2C DEBUGGER, OSCILLOSCOPE, VIRTUAL TERMINAL ×2.
4. 🧑 Wire using net-label terminals **SDA, SCL, PWM, DIR1, DIR2, ENC_A, ENC_B** (the brief names these):
   - pull-ups from SDA/SCL to +5V
   - all grounds common
   - L298: VS = +12V, VCC = +5V, SENSE pins to GND
   - 4 × 1N4007 flyback diodes on OUT1/OUT2
   - I2C Debugger on SDA/SCL
   - scope A = PWM, B = ENC_A, C = ENC_B
5. 🧑 Properties:
   - each ATMEGA328P: *Program File* = `master\master.ino.standard.hex` / `slave\slave.ino.standard.hex`, *CKSEL* = `(1111) Ext. Crystal 8.0- MHz`, *CLKDIV8* = `(1) Unprogrammed`, *Clock Frequency* = `16MHz`
   - MOTOR-ENCODER: 12 V, PPR = 100 (note the default Zero-Load RPM and Load %)
   - Virtual Terminals: 9600 8N1
6. Evidence: a schematic screenshot with every net labelled → `report/`

Commit: `feat(proteus): add circuit schematic with labelled nets`

### Task 2 – I2C link (20)
1. 🤖 `slave.ino`: Wire slave 0x08, `onReceive`/`onRequest`, checksum check, SP clamp, 500 ms watchdog, status frame. 🤖 `master.ino`: pot/switch, LCD, 100 ms command + status read, checksum check, keys.
2. 🧑 Arduino IDE 1.8.19, for each sketch:
   - *File → Open*
   - *Tools → Board → Arduino AVR Boards → Arduino Uno*
   - *Sketch → Export compiled Binary* (Ctrl+Alt+S)
3. 🧑 Proteus ▶:
   - I2C Debugger capture of one write and one read frame → 🤖 byte-by-byte annotation
   - Key `P`: the slave stops the motor after 500 ms and sets the fault bit (VT log + LCD)
   - Key `X`: the frame is discarded

Commits: `feat(slave): add I2C slave with checksum and watchdog`, `feat(master): add setpoint input, LCD and I2C command loop`

### Task 3 – Speed measurement (15)
1. 🤖 x4 quadrature ISR on pins 2/3 with direction. RPM every 50 ms.
2. 🧑 Oscilloscope screenshot of A and B (forward and reverse).
3. 🧑 Measured RPM vs. the motor model's displayed RPM at three duties → 🤖 table and % error.
4. 🤖 Resolution calculation (x1, x2, x4) and justification.

Commit: `feat(slave): add quadrature speed measurement`

### Task 4 – Velocity controller (25)
1. 🧑 `OPEN_LOOP`: speed at 5 duties → 🤖 linearity / dead-band comment.
2. 🤖 P, then PI, every 50 ms, with output limits and anti-windup.
3. 🧑 For each gain set: edit Kp/Ki in `slave.ino` → export → run `S` → copy the slave log to `data/`. 🤖 Compute rise, OS, Ts, SSE → tuning table.
4. 🤖 Step-response plot of the final controller.

Commits: `feat(slave): add P and PI velocity control with anti-windup`, `test(controller): add open-loop and tuning results`

### Task 5 – Tracking and disturbance (10)
1. 🧑 Key `T` → log → 🤖 SP and PV on one graph.
2. 🧑 MOTOR-ENCODER *Load/Max Torque %* × 1.5 → key `S` → 🤖 compare with the unloaded step.
3. 🤖 Explain why the integral term matters under load.

Commit: `test(controller): add tracking and load results`

### Task 6 – Report and demonstration (15)
1. 🤖 Draft the report (≤ 12 pages + appendices) in the brief's 8 sections. 🧑 Review it in your own words and export to PDF.
2. 🧑 Video ≤ 3 min: setpoint change, LCD, I2C Debugger.
3. 🧑 Zip `IT6512_258843A_I2C_Velocity.zip`: .pdsprj, master.ino, slave.ino, both .hex, report PDF, video. Unzip it to a new folder and check it runs.
4. 🧑 Viva preparation: be able to explain the code, the frames and the tuning.

Commit: `docs(report): add final report`

---

## 5. Progress

Note: the 5 V terminals are labelled **VCC** (Proteus default 5 V rail). A separate `+5V` rail could not be merged with the chips' hidden VCC pins.

30 steps in total. We complete them one at a time in the chat.

| Date | Milestone | Status |
|---|---|---|
| 2026-11-27 | Task 3 done: x4 decoding (3 RPM/count), direction from phase, scope captures both ways, accuracy within 2 counts. Motor model changed to Load 20 % and Effective Mass 0.004 (default 0.01 too slow for Ts ≤ 1 s) | ✅ Done |
| 2026-11-27 | Task 2 evidence captured: write/read frames, FLAGS=03 reset-integrator frame, watchdog test (master paused), bad-checksum test (frames ignored, watchdog trips) | ✅ Done |
| 2026-11-27 | I2C link working both ways. Fix: I2C clock lowered to 10 kHz, because at 100 kHz the slave's first reply byte arrived one bit late in Proteus 8.5 | ✅ Done |
| 2026-11-27 | Task 1 schematic complete: all nets labelled, 16 MHz clocks, 100 PPR, 9600 baud, exported to report/ | ✅ Done |
| 2026-11-26 | Brief read, tools verified (Proteus 8.5 SP0, Arduino IDE 1.8.19, Python 3.10), plan aligned to brief, repo pushed to GitHub | ✅ Done |

| # | Task | Step | Status |
|---|---|---|---|
| 1 | Task 1 | Create the Proteus project | ✅ Done |
| 2 | Task 1 | Place the two ATMEGA328P chips (MASTER, SLAVE) | ✅ Done |
| 3 | Task 1 | Place the motor parts: L298, MOTOR-ENCODER, 4× 1N4007 | ✅ Done |
| 4 | Task 1 | Place the operator parts: LM016L, POT-HG, SW-SPDT, 2× 4.7k | ✅ Done |
| 5 | Task 1 | Place power/ground terminals and virtual instruments | ✅ Done |
| 6 | Task 1 | Wire the I2C bus, pull-ups and I2C Debugger | ✅ Done |
| 7 | Task 1 | Wire slave → L298 → motor, flyback diodes, +12V | ✅ Done |
| 8 | Task 1 | Wire encoder A/B → slave pins 2/3 | ✅ Done |
| 9 | Task 1 | Wire master pot, switch and LCD | ✅ Done |
| 10 | Task 1 | Wire Virtual Terminals and oscilloscope | ✅ Done |
| 11 | Task 1 | Set part properties (16 MHz, 100 PPR, 9600 baud) | ✅ Done |
| 12 | Task 1 | Schematic screenshot + commit | ✅ Done |
| 13 | Task 2 | Arduino IDE 1.8.19 setup and first export | ✅ Done |
| 14 | Task 2 | slave.ino: I2C, checksum, watchdog → export hex | ✅ Done |
| 15 | Task 2 | master.ino: pot, switch, LCD, frames → export hex | ✅ Done |
| 16 | Task 2 | Load hex files in Proteus, first run | ✅ Done |
| 17 | Task 2 | I2C Debugger capture (write + read frame) | ✅ Done |
| 18 | Task 2 | Watchdog and bad-checksum tests + commit | ✅ Done |
| 19 | Task 3 | Add quadrature encoder code, run | ✅ Done |
| 20 | Task 3 | Oscilloscope screenshots (forward/reverse) | ✅ Done |
| 21 | Task 3 | Accuracy check at 3 duties + resolution calc + commit | ✅ Done |
| 22 | Task 4 | Open-loop test at 5 duties | ⏳ Next |
| 23 | Task 4 | P controller runs | ⬜ |
| 24 | Task 4 | PI tuning runs until targets met | ⬜ |
| 25 | Task 4 | Final step-response plot + tuning table + commit | ⬜ |
| 26 | Task 5 | Tracking sequence 150 → 80 → −100 → 0 | ⬜ |
| 27 | Task 5 | 50 % load test + integral explanation + commit | ⬜ |
| 28 | Task 6 | Write the report (8 sections, ≤ 12 pages) | ⬜ |
| 29 | Task 6 | Record the demo video (≤ 3 min) | ⬜ |
| 30 | Task 6 | Build the zip, test it, viva preparation | ⬜ |
