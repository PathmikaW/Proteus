# IT6512 – I2C-Linked Arduino Velocity Controller (Proteus)

Two Arduino Unos simulated in Proteus 8. The **master** reads a setpoint (pot + direction switch), shows SP/PV/duty on a 16×2 LCD, and sends a command frame over I2C every 100 ms. The **slave** (address 0x08) measures speed from a 100 PPR quadrature encoder (x4 decoding), runs a PI velocity loop every 50 ms, drives an L298 H-bridge, and stops the motor if no valid command arrives for 500 ms.

See [PLAN.md](PLAN.md) for the full plan, pin map, frame formats and progress.

## Build
1. Open `firmware/master/master.ino` and `firmware/slave/slave.ino` in Arduino IDE 2.x → *Sketch → Export Compiled Binary*, then copy the `.hex` files into `hex/`.
2. Open `proteus/*.pdsprj`, check each Uno's program file points at `hex/` and the clock is 16 MHz, then run.
