# IT6512 – I2C-Linked Arduino Velocity Controller (Proteus)

Two Arduino Unos (modelled as ATmega328P at 16 MHz) simulated in Proteus 8.5. The **master** reads a setpoint (pot + direction switch), shows SP/PV/duty on a 16×2 LCD, and sends a command frame over I2C every 100 ms. The **slave** (address 0x08) measures speed from a 100 PPR quadrature encoder (x4 decoding), runs a PI velocity loop every 50 ms, drives an L298 H-bridge, and stops the motor if no valid command arrives for 500 ms.

See [PLAN.md](PLAN.md) for the full plan, pin map, frame formats and progress.

## Toolchain
- Proteus 8 Professional 8.5 SP0
- Arduino IDE 1.8.19 (board: Arduino Uno)
- Python 3.10 with pandas and matplotlib

## Build and run
1. In Arduino IDE 1.8.19, open `firmware/master/master.ino` and `firmware/slave/slave.ino`. For each, choose *Tools → Board → Arduino Uno*, then *Sketch → Export compiled Binary*.
2. Run `scripts\update_hex.bat`. It copies each `*.ino.standard.hex` to `proteus\master.hex` and `proteus\slave.hex`.
3. Open `proteus/IT6512_I2C_Velocity.pdsprj` in Proteus 8.5 and press ▶.
