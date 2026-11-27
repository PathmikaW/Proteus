# IT6512 – I2C-Linked Arduino Velocity Controller (Proteus)

Two Arduino Unos (modelled as ATmega328P at 16 MHz) simulated in Proteus 8.5:

- The **master** reads a setpoint (pot + direction switch), shows SP/PV/duty on a 16×2 LCD, and sends a command frame over I2C every 100 ms.
- The **slave** (address 0x08) measures speed from a 100 PPR quadrature encoder, runs a PI velocity loop every 50 ms, drives an L298 H-bridge, and stops the motor if no valid command arrives for 500 ms.

See [PLAN.md](PLAN.md) for the plan and progress.

## Build and run
1. Arduino IDE 1.8.19: open `master/master.ino` and `slave/slave.ino`. For each, select *Tools → Board → Arduino Uno*, then *Sketch → Export compiled Binary*.
2. Proteus 8.5: open `258843A.pdsprj` and press ▶.
