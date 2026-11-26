# IT6512 Assignment 1 – I2C-Linked Arduino Velocity Controller (Proteus)

Step-by-step plan. Brief: `../Assignment1- Proteus/Assignment_I2C_Arduino_Velocity_Controller_Proteus.pdf`

Marks: Circuit 15 · I2C 20 · Speed 15 · Controller 25 · Tracking/load 10 · Report/video/viva 15 = **100**

---

## 0. Toolchain (fixed – all instructions below match these versions)

| Tool | Version on this PC | Notes |
|---|---|---|
| Proteus 8 Professional | **8.5 SP0 (Build 22067)** | No "Arduino Uno" part in 8.5, so each Uno is modelled by its **ATMEGA328P** chip (AVR2.LIB) |
| Arduino IDE | **1.8.19** (AVR core 1.8.3) | Board: *Arduino Uno*. *Sketch → Export compiled Binary* (Ctrl+Alt+S) |
| Python | **3.10** (pandas 2.2.1, matplotlib 3.7.2) | Plots and step metrics |
| Git | Repo-local identity PathmikaW | Conventional Commits, one or more per phase |

**Library rule:** the brief allows only the Arduino core and Wire libraries. So we do **not** use `LiquidCrystal`. The master has its own small HD44780 4-bit LCD driver.

**Who does what:** 🧑 = you, in the Proteus / Arduino GUI · 🤖 = me (code, scripts, report text)

---

## 1. Design decisions

### 1.1 Pin allocation (Arduino pin → ATmega328P pin name in Proteus)

Proteus labels ATMEGA328P pins by port name, so wire by the right-hand column.

**Slave (I2C 0x08, motor control)**

| Arduino | ATmega328P pin | Net label | Function |
|---|---|---|---|
| D0 / D1 | PD0/RXD, PD1/TXD | S_RX, S_TX | Virtual Terminal (CSV log, 9600 8N1) |
| D2 | PD2/INT0 | ENC_A | Encoder A, `CHANGE` interrupt |
| D3 | PD3/INT1 | ENC_B | Encoder B, `CHANGE` interrupt |
| D7 | PD7 | DIR1 | L298 IN1 |
| D8 | PB0 | DIR2 | L298 IN2 |
| D9 | PB1/OC1A | PWM | L298 ENA (Timer1, ~490 Hz) |
| A4 / A5 | PC4/SDA, PC5/SCL | SDA, SCL | I2C bus |

**Master (commands + UI)**

| Arduino | ATmega328P pin | Net label | Function |
|---|---|---|---|
| D0 / D1 | PD0/RXD, PD1/TXD | M_RX, M_TX | Virtual Terminal (commands + log) |
| D2, D3, D4, D5 | PD2, PD3, PD4, PD5 | LCD_D7, LCD_D6, LCD_D5, LCD_D4 | LM016L data (4-bit) |
| D6 | PD6 | DIR_SW | SW-SPDT direction switch |
| D11 | PB3 | LCD_E | LM016L E |
| D12 | PB4 | LCD_RS | LM016L RS |
| A0 | PC0/ADC0 | SP_POT | POT-HG wiper |
| A4 / A5 | PC4/SDA, PC5/SCL | SDA, SCL | I2C bus |

### 1.2 Operator interface (master)
- **Pot** = |setpoint| 0…250 RPM, **switch** = direction (+/−). Enable bit = 1 while running.
- **Typed commands** in the master's Virtual Terminal (for repeatable tests):

| Key | Action | Used for |
|---|---|---|
| `150` + Enter (any −250…250) | Fixed setpoint override | Accuracy and open-loop checks |
| `M` | Back to pot/switch control | Demo |
| `S` | Scripted step 0 → 150 RPM | Task 4 |
| `T` | Scripted 150 → 80 → −100 → 0 RPM (2 s each) | Task 5 |
| `P` | Pause/resume I2C transmission | Task 2 watchdog test |
| `X` | Send one frame with a corrupted checksum | Task 2 checksum test |
| `R` | Send one frame with the reset-integrator flag | Task 2 flags |
| `D` + value (slave test build only) | Fixed PWM duty, open loop | Tasks 3 and 4 open loop |

- LCD: line 1 `SP:+150 PV:+148`, line 2 `D:182 RUN` / `WDOG` / `SAT`

### 1.3 I2C frames (exactly as in the brief)
- Command (master → 0x08, every 100 ms): `[0x01, SP_HI, SP_LO, FLAGS, CHK]`. FLAGS bit0 = enable, bit1 = reset integrator.
- Status (master reads 5 bytes after each command): `[STATUS, PV_HI, PV_LO, DUTY, CHK]`. STATUS bit0 = running, bit1 = watchdog fault, bit2 = saturated.
- `CHK = b0 ^ b1 ^ b2 ^ b3`. int16 values are sent high byte first.
- On the I2C Debugger the address byte shows as **0x10** for a write and **0x11** for a read (0x08 shifted left by 1, plus the R/W bit).
- Slave `onReceive`: copy the bytes and the length into a volatile buffer, set a flag, return.
- Slave `onRequest`: `Wire.write(statusFrame, 5)` from a buffer that `loop()` prepares.
- `loop()` validates length = 5, CMD = 0x01 and CHK, clamps SP to ±250, then resets the watchdog. Bad frames are counted and logged, and do **not** reset the watchdog.

### 1.4 Speed measurement: x4 quadrature
ω = 60·Δn / (m·N·T), with N = 100 PPR and T = 0.05 s:

| Decoding | Counts/rev | Resolution |
|---|---|---|
| x1 | 100 | 12 RPM/count |
| x2 | 200 | 6 RPM/count |
| **x4 (chosen)** | 400 | **3 RPM/count** |

The SSE target (≤ 3 RPM) equals one x4 count, so only x4 can meet it. Both channels use `CHANGE` interrupts and a 16-entry state table that gives ±1 per edge, which also gives the direction. The count is read with `noInterrupts()`/`interrupts()`.

### 1.5 Controller (slave, every 50 ms)
- `e = SP − PV`; `u = Kp·e + Ki·T·Σe` (+ optional feed-forward `Kff·SP` from the open-loop slope)
- Clamp `u` to ±255. PWM = |u| on D9, sign → IN1/IN2.
- Anti-windup: conditional integration. Σe is frozen while the output is saturated **and** e would push it further into saturation.
- Watchdog (500 ms without a valid frame): PWM = 0, Σe = 0, fault bit set. It clears on the next valid frame.
- CSV log every 50 ms: `t,sp,pv,duty,st` (short lines so 9600 baud keeps up)

---

## 2. Repository layout

```
Proteus/
├── PLAN.md, README.md
├── firmware/
│   ├── master/master.ino        (folder name must equal sketch name – Arduino rule)
│   └── slave/slave.ino
├── proteus/
│   ├── IT6512_I2C_Velocity.pdsprj
│   ├── master.hex, slave.hex    (copied from the Arduino export; same folder = portable paths)
├── scripts/
│   ├── update_hex.bat           (copies *.ino.standard.hex → proteus/)
│   ├── plot_log.py              (CSV → plot)
│   └── step_metrics.py          (rise, overshoot, Ts 5 %, SSE)
├── data/                        (CSV logs pasted from the Virtual Terminal)
├── report/figures/              (screenshots + plots)
└── video/                       (not committed)
```

---

## 3. Phases (follow in order)

### Phase A – Circuit build (Task 1, 15 marks) 🧑

**A1. Create the project (Proteus 8.5)**
1. *File → New Project* → Name `IT6512_I2C_Velocity`, Path `E:\MSC\IT6512\Assignment1-Proteus\Proteus\proteus` → **Next**
2. Schematic: *Create a schematic from the selected template* → **Landscape A3** → **Next**
3. PCB: *Do not create a PCB layout* → **Next**
4. Firmware: *No Firmware Project* → **Next** → **Finish**

**A2. Place parts**

In *Component Mode*, press **P** (Pick Devices) and type each keyword:

| Keyword | Qty | Ref |
|---|---|---|
| ATMEGA328P | 2 | U1 = MASTER, U2 = SLAVE (set in *Edit Properties → Part Reference*) |
| L298 | 1 | U3 |
| MOTOR-ENCODER | 1 | M1 |
| LM016L | 1 | LCD1 |
| POT-HG | 1 | RV1 |
| SW-SPDT | 1 | SW1 |
| RES | 2 | R1, R2 = 4.7k |
| 1N4007 | 4 | D1–D4 |

Then in the left toolbar:
- *Virtual Instruments Mode* → **I2C DEBUGGER**, **OSCILLOSCOPE**, **VIRTUAL TERMINAL** ×2
- *Terminals Mode* → **POWER** (labels `+5V`, `+12V`), **GROUND**, **DEFAULT** (for net labels)

**A3. Wire it** (use net-label terminals, not long wires, between blocks)
- **I2C:** PC4/SDA of both MCUs → `SDA`. PC5/SCL of both → `SCL`. R1 from `SDA` to `+5V`, R2 from `SCL` to `+5V`. I2C Debugger SDA → `SDA`, SCL → `SCL`.
- **Slave → L298:** PB1 → `PWM` → ENA. PD7 → `DIR1` → IN1. PB0 → `DIR2` → IN2. L298 VS → `+12V`, VCC → `+5V`, GND and SENSA → GND.
- **L298 → motor:** OUT1 → motor terminal 1, OUT2 → motor terminal 2.
- **Flyback diodes:** each output gets one diode to +12V (cathode on +12V) and one from GND (anode on GND).
- **Encoder:** motor encoder outputs → `ENC_A` → PD2 and `ENC_B` → PD3 (hover the motor pins to read their names; leave the index pin unconnected).
- **Master UI:**
  - POT-HG ends to `+5V` and GND, wiper → `SP_POT` → PC0.
  - SW-SPDT common → `DIR_SW` → PD6, one throw to GND, the other to `+5V`.
  - LM016L: VSS and VEE → GND, VDD → `+5V`, RW → GND, RS → PB4, E → PB3, D4–D7 → PD5, PD4, PD3, PD2. Leave D0–D3 open.
- **Virtual Terminals:** master TXD → VT1 RXD and VT1 TXD → master RXD. Slave TXD → VT2 RXD.
- **Oscilloscope:** A = `PWM`, B = `ENC_A`, C = `ENC_B`, D = `DIR1`.
- **RESET pin** (PC6/RESET) of each MCU → `+5V`.
- Check *Design → Configure Power Rails*: the `+12V` net must be on a 12 V rail, and VCC/GND hold the MCUs' hidden power pins.

**A4. Set properties** (double-click each part)
- **Each ATMEGA328P:**
  - *Program File* = `master.hex` / `slave.hex` (file name only; the hex sits next to the .pdsprj)
  - *CKSEL Fuses* = `(1111) Ext. Crystal 8.0- MHz`
  - *CLKDIV8* = `(1) Unprogrammed`
  - *Advanced Properties → Clock Frequency* = `16MHz`
- **MOTOR-ENCODER:** Nominal Voltage `12V`, *Pulses per Revolution* `100`.
  - Record the default *Zero Load RPM* and *Load/Max Torque %*.
  - If Zero Load RPM is below ~300, raise it to 300–400 so ±250 RPM is reachable, and state this in the report.
- **Virtual Terminals:** Baud 9600, Data 8, Parity NONE, Stop 1.
- **R1/R2:** 4.7k.

**A5. Evidence:** *File → Export Graphics → Export Bitmap* (or a screenshot) of the full sheet, plus zoomed views → `report/figures/schematic_*.png`.

✅ Commit: `feat(proteus): add schematic for master, slave, L298 and encoder motor`

---

### Phase B – I2C link and watchdog (Task 2, 20 marks)

**B1. 🤖 Write the code**
- `slave.ino`: Wire slave 0x08, frame validation, watchdog, status frame, CSV log. The motor is still open loop (duty ∝ SP).
- `master.ino`: pot/switch read, LCD driver, 100 ms command + read-back, checksum check, serial commands.
- `scripts/update_hex.bat`

**B2. 🧑 Build (Arduino IDE 1.8.19)** – repeat for each sketch:
1. *File → Open* → `firmware\master\master.ino`
2. *Tools → Board → Arduino AVR Boards → **Arduino Uno***
3. *Sketch → **Export compiled Binary*** (Ctrl+Alt+S). This writes `master.ino.standard.hex` and `master.ino.with_bootloader.hex` into the sketch folder.
4. Double-click `scripts\update_hex.bat`. It copies the **standard** hex (not the bootloader one) to `proteus\master.hex` / `slave.hex`.

**B3. 🧑 Run and capture**
1. Press ▶ (bottom-left). If an instrument window is closed, reopen it from the *Debug* menu (*I2C Debugger*, *Virtual Terminal*, *Digital Oscilloscope*).
2. **I2C Debugger:** pause (⏸) and expand one write frame and one read frame. Screenshot → `report/figures/i2c_write.png`, `i2c_read.png`. 🤖 then annotates each byte and checks the CHK by hand.
3. **Watchdog test:** click in VT1, type `P`. VT2 should show the fault bit set and duty → 0 about 500 ms (sim time) after the last frame, and the LCD shows `WDOG`. Screenshot the VT2 log, LCD and scope.
4. **Bad-checksum test:** type `X`. VT2 should log `BADCHK` and the watchdog timer must not reset.

✅ Commits: `feat(slave): add I2C slave with frame validation and watchdog`, `feat(master): add command loop, LCD driver and serial test commands`, `test(i2c): add debugger captures and watchdog evidence`

---

### Phase C – Speed measurement (Task 3, 15 marks)

1. 🤖 Add the x4 quadrature ISR and the 50 ms RPM calculation to `slave.ino`. Build a **test mode** where `D<duty>` sets a fixed PWM.
2. 🧑 Export hex, run, and capture the scope screenshot of A/B in forward **and** reverse (A leads B vs. B leads A) → `report/figures/encoder_fwd.png`, `encoder_rev.png`.
3. 🧑 At duty 64, 128 and 192, write down the RPM shown on the motor part and the slave's PV → `data/accuracy.csv`.
4. 🤖 Write the resolution calculation (x1/x2/x4) and the accuracy table with % error.

✅ Commit: `feat(slave): add x4 quadrature decoding and 50 ms speed measurement`

---

### Phase D – Velocity controller (Task 4, 25 marks)

1. **Open loop:** 🧑 set duty 51, 102, 153, 204 and 255 (20–100 %) and log steady RPM → `data/openloop.csv`. 🤖 plots it and comments on linearity and dead band.
2. 🤖 Add the PI controller with clamp, anti-windup and saturation bit. Gains are set over serial (`KP<value>`, `KI<value>`), so each tuning step doesn't need a rebuild.
3. **Tuning loop.** For each gain set:
   - 🧑 type the gains, then `S`
   - copy the VT2 log (right-click in the VT window → copy) to `data/step_kpX_kiY.csv`
   - 🤖 runs `step_metrics.py` and adds a row to the tuning table
4. Order of tuning:
   - P only: 3–4 Kp values, which show the SSE
   - add Ki and increase it until SSE ≤ 3 RPM
   - trim to meet **OS ≤ 10 %, Ts(5 %) ≤ 1.0 s, SSE ≤ 3 RPM**
   - add `Kff` only if needed
5. 🤖 Final step plot → `report/figures/step_final.png`

✅ Commits: `feat(slave): add PI velocity controller with anti-windup`, `feat(scripts): add log plotting and step metrics`, `test(controller): add open-loop and tuning logs`

---

### Phase E – Tracking and load (Task 5, 10 marks)

1. 🧑 Type `T` and save the log → `data/tracking.csv`. 🤖 plots SP and PV on one graph.
2. 🧑 Stop the sim and raise the MOTOR-ENCODER *Load/Max Torque %* by 50 % (e.g. 20 → 30; note both values). Run `S` → `data/step_loaded.csv`. 🤖 overlays it on the unloaded step.
3. 🤖 Explanation: extra load torque needs a constant extra duty. P-only can supply it only through a persistent error. The integrator builds up exactly that duty, so the error goes to zero.

✅ Commit: `test(controller): add tracking and load disturbance results`

---

### Phase F – Report, video, packaging (Task 6, 15 marks)

1. 🤖 Draft the report (≤ 12 pages + appendices) in the brief's 8-section structure, with pin tables, block diagram, frames, resolution, tuning table and plots. Include an AI-assistance acknowledgement, as the brief requires. 🧑 Review it, add your student ID, and export it to PDF.
2. 🧑 Record the video (≤ 3 min) with Snipping Tool → Record or Win+G: schematic tour → pot change → LCD → I2C Debugger → `P` watchdog.
3. 🤖 Packaging script → `IT6512_<StudentID>_I2C_Velocity.zip`: `.pdsprj`, `master.ino`, `slave.ino`, both `.hex`, PDF and video.
4. 🧑 Unzip into a fresh folder, open the `.pdsprj` and press ▶. It must run without changes.

✅ Commits: `docs(report): add final report`, `chore: add submission packaging script`

---

## 4. Risks

| Risk | Mitigation |
|---|---|
| MCU runs 8× slow | CLKDIV8 must be *Unprogrammed* and Clock Frequency 16 MHz (A4) |
| Wrong hex loaded | Always use `*.ino.standard.hex` via `update_hex.bat` |
| Sim slower than real time | Report times from the slave's `millis()` column (sim time) |
| VT log loses lines at 9600 | Short CSV lines; runs of about 5 s |
| PI alone misses Ts ≤ 1 s | Add feed-forward from the open-loop slope |
| .pdsprj does not open on the marker's PC | Hex beside the project, file-name-only paths; test the unzipped copy |

## 5. Progress

- [x] 0 Plan and toolchain verified
- [ ] A Circuit
- [ ] B I2C link + watchdog
- [ ] C Speed measurement
- [ ] D Controller tuning
- [ ] E Tracking + load
- [ ] F Report, video, zip
