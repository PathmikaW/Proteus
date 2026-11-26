# IT6512 Assignment 1 – I2C-Linked Arduino Velocity Controller (Proteus)

Plan of work. Brief: `../Assignment1- Proteus/Assignment_I2C_Arduino_Velocity_Controller_Proteus.pdf`

Marks: Circuit 15 · I2C 20 · Speed 15 · Controller 25 · Tracking/load 10 · Report/video/viva 15 = **100**

---

## 0. Prerequisites

| Item | Status | Action |
|---|---|---|
| Proteus 8 Professional | **8.5 (8.5.22067)** installed | Has MOTOR-ENCODER, L298, LM016L, POT-HG, SW-SPDT, 1N4007, ATMEGA328P, I2C Debugger, scope, Virtual Terminal |
| Arduino Uno in Proteus | No "ARDUINO UNO" library part in 8.5 | Use the **Arduino 328** board (New Project → From Development Board), or ATMEGA328P parts at 16 MHz. Optional: third-party Arduino library |
| Proteus licence | Not yet confirmed | Check that a project saves and a non-sample design simulates |
| Arduino IDE | **1.8.19** (AVR core 1.8.3, Wire included) | Works. *Sketch → Export compiled Binary* gives the `.hex`. Brief says 2.x, but the output is the same |
| Python 3.10 | pandas 2.2.1, matplotlib 3.7.2 | Ready |

Check the MOTOR-ENCODER model first. Note its properties (nominal voltage, zero-load RPM, load %, pulses/rev, and whether it outputs A and B channels). Every later number depends on these.

---

## 1. Design decisions (fixed up front)

### Pin allocation

**Slave (I2C 0x08, motor control)**

| Pin | Net | Function |
|---|---|---|
| D2 (INT0) | ENC_A | Encoder channel A, `CHANGE` interrupt |
| D3 (INT1) | ENC_B | Encoder channel B, `CHANGE` interrupt |
| D9 (Timer1 PWM) | PWM | L298 ENA |
| D7 | DIR1 | L298 IN1 |
| D8 | DIR2 | L298 IN2 |
| A4 / A5 | SDA / SCL | I2C bus |
| D0 / D1 | TX/RX | Virtual Terminal (CSV log, 9600) |

**Master (commands + UI)**

| Pin | Net | Function |
|---|---|---|
| A0 | SP_POT | Setpoint potentiometer (POT-HG) |
| D6 | DIR_SW | SW-SPDT direction switch (INPUT_PULLUP) |
| D12, D11 | LCD_RS, LCD_E | LM016L |
| D5, D4, D3, D2 | LCD_D4..D7 | LM016L 4-bit mode |
| A4 / A5 | SDA / SCL | I2C bus (4.7 kΩ pull-ups to 5 V) |
| D0 / D1 | TX/RX | Virtual Terminal (commands + log, 9600) |

PWM on D9 uses Timer1, so `millis()` (Timer0) is unaffected.

### Operator interface (master)

- **Pot** sets the magnitude, 0 to 250 RPM. **Switch** sets the direction (+/−). Enable bit = 1 while running.
- **Serial commands** on the master's Virtual Terminal give repeatable tests:
  - `<number>`: fixed setpoint override (e.g. `150`, `-100`); `M` returns to pot control
  - `S`: scripted step 0 → 150 RPM (Task 4)
  - `T`: scripted sequence 150 → 80 → −100 → 0 RPM, ~2 s each (Task 5)
  - `P`: pause/resume transmission, for the **watchdog test** (Task 2)
  - `R`: send one frame with the reset-integrator flag
  - `X`: send one frame with a corrupted checksum, for the **bad-frame test** (Task 2)
- LCD line 1: `SP:+150 PV:+148`, line 2: `D:182 RUN` / `WDOG` / `SAT`

### I2C frames (exactly per brief)

- Command (master → 0x08, every 100 ms): `[0x01, SP_HI, SP_LO, FLAGS, CHK]`
- Status (read 5 bytes from 0x08 after each command): `[STATUS, PV_HI, PV_LO, DUTY, CHK]`
- `CHK = b0 ^ b1 ^ b2 ^ b3`; int16 values sent big-endian
- Slave `onReceive`: copy bytes to a volatile buffer and set a flag, nothing else. `loop()` validates length, CMD and CHK, clamps SP to ±250, and only then resets the watchdog timer.
- Slave `onRequest`: `Wire.write(statusFrame, 5)` from a buffer that `loop()` rebuilds every cycle.
- Master validates the status checksum and counts bad frames (shown on the terminal).

### Speed measurement

N = 100 PPR, T = 0.05 s, ω = 60·Δn / (m·N·T):

| Decoding | Counts/rev | Resolution (RPM/count) |
|---|---|---|
| x1 | 100 | 12.0 |
| x2 | 200 | 6.0 |
| **x4** | 400 | **3.0** |

**Choose x4.** The steady-state error target (≤ 3 RPM) equals one x4 count, so x1 and x2 are too coarse. x4 uses `CHANGE` interrupts on both A and B with a state-transition lookup table, which also gives the direction. Read the count with `noInterrupts()`/`interrupts()`. Optionally average the last 2 samples, and report its effect on noise and phase lag.

### Controller (slave, every 50 ms)

- `e = SP − PV`; `u = Kp·e + Ki·T·Σe`; signed output clamped to ±255 → |u| on PWM, sign on IN1/IN2
- Anti-windup: only add to Σe when the output is not saturated, or when e would drive it back from saturation. STATUS bit 2 = saturated.
- If SP = 0 and enable = 0: coast/brake, clear the integrator
- Watchdog: no valid frame for 500 ms → PWM = 0, integrator cleared, STATUS bit 1 set. Clear the bit on the next valid frame.
- Optional feed-forward `u_ff = SP·k_ff` taken from the open-loop slope, if PI alone cannot meet settling ≤ 1 s
- CSV log on the slave: `t_ms,SP,PV,duty,status` every 50 ms (keep lines short at 9600 baud)

---

## 2. Work breakdown (in order)

### Phase A – Circuit (Task 1, 15)
1. Create the Proteus project `proteus/IT6512_I2C_Velocity.pdsprj`.
2. Place 2× Uno, L298, MOTOR-ENCODER, LM016L, POT-HG, SW-SPDT, 2× 4.7 kΩ, 4× 1N4007, 12 V supply, I2C Debugger, oscilloscope, 2× Virtual Terminal.
3. Wire using **net-label terminals** for SDA, SCL, PWM, DIR1, DIR2, ENC_A, ENC_B, plus LCD nets. Use a common GND. L298 VS = 12 V, VCC = 5 V, SENSE pins to GND, flyback diodes on OUT1/OUT2.
4. Set motor to 12 V and 100 PPR (record the actual value), and each Uno to 16 MHz with its `.hex`.
5. Evidence: full schematic screenshot, plus zoomed screenshots of each section.

### Phase B – I2C link (Task 2, 20)
1. Write `slave.ino` (I2C + watchdog only, motor driven open-loop from SP) and `master.ino` (pot, switch, LCD, frames).
2. Verify with the I2C Debugger: one write frame and one read frame. Annotate each byte and check the CHK by hand.
3. Watchdog test: press `P` on the master. The slave stops the motor ~500 ms later and sets the fault bit. Capture a scope/terminal timestamp and the LCD.
4. Bad-checksum test: press `X`. The slave ignores the frame and the log shows it was rejected.

### Phase C – Speed measurement (Task 3, 15)
1. Add the x4 quadrature ISR and the 50 ms RPM calculation.
2. Scope screenshot of ENC_A/ENC_B in both directions, showing the phase lead/lag.
3. Accuracy table: measured RPM vs. the motor model's displayed RPM at 3 duties (e.g. 25%, 50%, 75%) with % error.

### Phase D – Controller (Task 4, 25)
1. Open loop: RPM vs. duty at 5 duties (e.g. 50, 100, 150, 200, 255). Plot it and comment on linearity and dead band.
2. P only: sweep Kp and record the steady-state error, which should be non-zero.
3. PI: add Ki and tune for the 0 → 150 step to meet **OS ≤ 10 %, Ts(±5 %) ≤ 1.0 s, ess ≤ 3 RPM**.
4. Log every gain set, using `scripts/step_metrics.py` to compute rise time, OS, Ts and ess from each CSV.
5. Final step-response plot.

### Phase E – Tracking and load (Task 5, 10)
1. Sequence `T`: 150 → 80 → −100 → 0. Plot SP and PV together.
2. Increase the motor load by 50 % in its properties and repeat the 150 step. Overlay it with the unloaded run.
3. Explain: the load torque needs a constant extra duty to hold speed. With P only, that duty comes from a persistent error. The integrator accumulates until it supplies the extra duty, so ess → 0.

### Phase F – Report, video, packaging (Task 6, 15)
1. Report (≤ 12 pages + appendices), following the brief's 8-section structure.
2. Demo video (≤ 3 min): schematic tour → setpoint change via pot → LCD → I2C Debugger frames → watchdog test.
3. Zip `IT6512_<StudentID>_I2C_Velocity.zip`: `.pdsprj`, `master.ino`, `slave.ino`, both `.hex`, report PDF, video.
4. Test that the zipped `.pdsprj` opens and runs from a fresh folder. The `.hex` paths must be relative.

---

## 3. Repository layout

```
Proteus/
├── PLAN.md                 this file
├── README.md
├── firmware/
│   ├── master/master.ino
│   └── slave/slave.ino
├── hex/                    exported master.hex, slave.hex
├── proteus/                .pdsprj project
├── data/                   CSV logs from the Virtual Terminals
├── scripts/                plot + step-metric scripts (Python)
├── report/                 figures/, report source, final PDF
└── video/                  demo video (not committed if large)
```

## 4. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Proteus runs slower than real time | Report all times as simulation time, taken from `millis()` in the CSV |
| 9600 baud logging stalls the 50 ms loop | Short CSV lines (~20 chars), print after the control update |
| x4 interrupt load at high RPM | 250 RPM × 400 = ~1.7 k ISR/s, which is fine for a 16 MHz AVR |
| MOTOR-ENCODER PPR or load property differs | Use the nearest value and state it in the report; recompute resolution |
| Settling ≤ 1 s not met by PI | Add feed-forward from the open-loop slope, or a D term |
| Academic integrity (AI use) | Acknowledge AI assistance in the report as the brief requires |

## 5. Progress

- [ ] A Circuit
- [ ] B I2C link + watchdog
- [ ] C Speed measurement
- [ ] D Controller tuning
- [ ] E Tracking + load
- [ ] F Report, video, zip
