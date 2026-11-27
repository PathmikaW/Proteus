"""
block_diagram.py - block diagram of the master and slave firmware (IT6512, 258843A)
Writes report/block_diagram.png.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

OUT = os.path.join(os.path.dirname(__file__), "..", "report", "block_diagram.png")
TEXT, TEXT2, EDGE, FILL, FILL2 = "#191919", "#666666", "#257de0", "#e9f1fb", "#f4f4f4"

fig, ax = plt.subplots(figsize=(10, 6.4))
ax.set_xlim(0, 100)
ax.set_ylim(0, 64)
ax.axis("off")


def box(x, y, w, h, title, lines, fill=FILL, edge=EDGE, lw=1.6):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2",
                                fc=fill, ec=edge, lw=lw))
    ax.text(x + w / 2, y + h - 2.0, title, ha="center", va="top", fontsize=10, weight="bold", color=TEXT)
    for i, s in enumerate(lines):
        ax.text(x + 1.5, y + h - 5.8 - i * 2.75, s, ha="left", va="top", fontsize=7.8, color=TEXT)


def arrow(x1, y1, x2, y2, both=False):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="<->" if both else "->", color=TEXT2, lw=1.3))


def label(x, y, s, ha="center"):
    ax.text(x, y, s, ha=ha, va="center", fontsize=7.5, color=TEXT2)


box(2, 23, 30, 39, "MASTER (ATmega328P, 16 MHz)", [
    "setup: Wire master, I2C 10 kHz, 25 ms timeout",
    "loop, every 100 ms:",
    "  1  read setpoint: pot A0 + switch D6",
    "     (or test key S / T / O / number)",
    "  2  build command frame + XOR CHK",
    "  3  write 5 bytes to 0x08",
    "  4  read 5-byte status frame,",
    "     check CHK, count errors",
    "  5  update LCD: SP, PV, duty, state",
    "serial keys: S T O M P X  (9600 baud)",
])
box(52, 23, 46, 39, "SLAVE (ATmega328P, 16 MHz, I2C 0x08)", [
    "onReceive: copy frame -> buffer, set flag",
    "onRequest: send prepared status frame",
    "INT0/INT1 (D2, D3, CHANGE): x4 quadrature",
    "  table -> encoderCount +1 / -1",
    "loop:",
    "  - new frame? check length, CMD, CHK,",
    "    clamp SP to +-250, reset watchdog",
    "  - no valid frame for 500 ms -> stop, fault",
    "  - every 50 ms: PV = 3 x counts (RPM)",
    "    PI: u = Kp e + Ki T sum(e), +-255 limit,",
    "    anti-windup -> PWM D9, DIR D7/D8",
    "  - update status frame, CSV log",
])
arrow(32.6, 50, 51.4, 50, both=True)
label(42, 54, "I2C  SDA A4 / SCL A5")
label(42, 52, "4.7 k pull-ups")
label(42, 47.5, "command every 100 ms ->")
label(42, 45.5, "<- status frame")

box(2, 1.5, 30, 13, "Operator I/O", ["pot (setpoint), direction switch", "16x2 LCD (own 4-bit driver)",
                                     "Virtual Terminal (keys + log)"], fill=FILL2, edge=TEXT2, lw=1.1)
arrow(17, 15.0, 17, 22.4, both=True)

box(36, 1.5, 13, 13, "I2C Debugger", ["decodes every", "frame on SDA/SCL"], fill=FILL2, edge=TEXT2, lw=1.1)
ax.plot([42.5, 42.5], [15.0, 43.5], color=TEXT2, lw=1.0, ls=":")

box(53, 1.5, 19, 13, "L298 H-bridge", ["ENA <- PWM D9", "IN1/IN2 <- D7/D8", "VS 12 V, 4 x 1N4007"],
    fill=FILL2, edge=TEXT2, lw=1.1)
box(78, 1.5, 20, 13, "MOTOR-ENCODER", ["12 V, 360 RPM no-load", "load 20 %, 100 PPR",
                                       "Q1/Q2 = ENC_A/ENC_B"], fill=FILL2, edge=TEXT2, lw=1.1)
arrow(62, 22.4, 62, 15.0)
label(67, 18.7, "PWM, DIR", ha="left")
arrow(72.4, 8, 77.6, 8)
arrow(88, 15.0, 88, 22.4)
label(86, 18.7, "ENC_A, ENC_B", ha="right")

fig.tight_layout()
fig.savefig(OUT, dpi=170)
print("written", os.path.abspath(OUT))
