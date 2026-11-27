"""
plot_results.py - report figures from the slave CSV logs (IT6512, 258843A)

Creates in report/:
  openloop.png        speed vs PWM duty, open loop (Task 4)
  tuning_compare.png  0 -> 150 RPM step for every gain set tried (Task 4)
  step_final.png      final PI controller step response with targets (Task 4)
  tracking.png        setpoint sequence 150 -> 80 -> -100 -> 0 RPM (Task 5)

All times are Proteus simulation time, measured from the step.
Usage:  python scripts/plot_results.py
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))
from step_metrics import load, metrics  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "report")

# colours: reference palette (light mode)
BLUE, ORANGE, AQUA, YELLOW, MAGENTA = "#257de0", "#f47230", "#00b894", "#f2a600", "#ee7fac"
GREEN, VIOLET = "#008a17", "#6a4ce2"
TEXT, TEXT2, GRID, BAND = "#191919", "#666666", "#e6e6e6", "#e9f1fb"

plt.rcParams.update({
    "font.size": 10, "axes.edgecolor": TEXT2, "axes.labelcolor": TEXT,
    "xtick.color": TEXT2, "ytick.color": TEXT2, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "legend.frameon": False, "figure.dpi": 150,
})


def step_series(path):
    rows = load(path)
    m = metrics(rows)
    t0 = m["t_step_s"] * 1000
    t = [(r[0] - t0) / 1000 for r in rows]
    return rows, t, m


def plot_openloop():
    duty, pv = [], []
    with open(os.path.join(DATA, "openloop.csv")) as f:
        for r in csv.DictReader(f):
            duty.append(float(r["duty"]))
            pv.append(float(r["pv_rpm_mean_last_1s"]))
    fig, ax = plt.subplots(figsize=(6, 3.6))
    ax.plot([0] + duty, [0] + pv, color=BLUE, lw=2, marker="o", ms=6,
            markeredgecolor="white", markeredgewidth=1.5)
    ax.plot([0, 255], [0, 267 * 255 / 250], color=TEXT2, lw=1, ls="--")
    ax.text(150, 185, "straight line through\n0 and 250 for reference", color=TEXT2, fontsize=8)
    for d, v in zip(duty, pv):
        ax.annotate(f"{v:.0f}", (d, v), textcoords="offset points", xytext=(-14, 6),
                    fontsize=8, color=TEXT)
    ax.set_xlabel("PWM duty (0-255)")
    ax.set_ylabel("Speed (RPM)")
    ax.set_title("Open-loop speed vs PWM duty (mean of last 1 s at each duty)",
                 fontsize=10, color=TEXT, loc="left")
    ax.set_xlim(0, 260)
    ax.set_ylim(0, 290)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "openloop.png"))
    plt.close(fig)


def plot_tuning():
    runs = [("P  Kp=1", "tune_P1_kp1.csv", YELLOW), ("P  Kp=2", "tune_P2_kp2.csv", ORANGE),
            ("P  Kp=4", "tune_P3_kp4.csv", MAGENTA), ("PI Kp=2 Ki=4", "tune_PI1_kp2_ki4.csv", VIOLET),
            ("PI Kp=1 Ki=10 (final)", "tune_PI2_kp1_ki10.csv", BLUE),
            ("PI Kp=2 Ki=10", "tune_PI3_kp2_ki10.csv", AQUA)]
    fig, ax = plt.subplots(figsize=(8.4, 4))
    ax.axhspan(142.5, 157.5, color=BAND, zorder=0)
    ax.axhline(150, color=TEXT2, lw=1, ls="--")
    for label, fname, col in runs:
        rows, t, _ = step_series(os.path.join(DATA, fname))
        pts = [(ti, r[2]) for ti, r in zip(t, rows) if -0.2 <= ti <= 3.0]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=col,
                lw=2.4 if "final" in label else 1.6, label=label)
    ax.text(2.0, 160, "SP 150 RPM, +-5 % band", color=TEXT2, fontsize=8)
    ax.set_xlabel("Time from step (s, simulation time)")
    ax.set_ylabel("Measured speed PV (RPM)")
    ax.set_title("Step 0 -> 150 RPM for every gain set tried", fontsize=10, color=TEXT, loc="left")
    ax.set_xlim(-0.2, 3.0)
    ax.set_ylim(0, 175)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "tuning_compare.png"))
    plt.close(fig)


def plot_final():
    rows, t, m = step_series(os.path.join(DATA, "tune_PI2_kp1_ki10.csv"))
    sel = [(ti, r) for ti, r in zip(t, rows) if -0.25 <= ti <= 4.0]
    ts = [s[0] for s in sel]
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(7, 5.2), sharex=True,
                                  gridspec_kw={"height_ratios": [3, 1.3]})
    ax.axhspan(142.5, 157.5, color=BAND, zorder=0)
    ax.step(ts, [s[1][1] for s in sel], where="post", color=TEXT2, lw=1.2, ls="--", label="Setpoint SP")
    ax.plot(ts, [s[1][2] for s in sel], color=BLUE, lw=2, label="Measured speed PV")
    pk = max(sel, key=lambda s: s[1][2])
    ax.plot(pk[0], pk[1][2], "o", color=BLUE, ms=6, markeredgecolor="white", markeredgewidth=1.5)
    ax.annotate(f"peak {pk[1][2]} RPM\novershoot {m['overshoot_pct']:.0f} %", (pk[0], pk[1][2]),
                xytext=(14, 4), textcoords="offset points", fontsize=8, color=TEXT)
    ax.axvline(m["settling_s"], color=TEXT2, lw=1, ls=":")
    ax.text(m["settling_s"] + 0.05, 40, f"settled (+-5 %)\nat {m['settling_s']:.2f} s",
            fontsize=8, color=TEXT)
    ax.text(2.6, 125, "+-5 % band\n142.5 - 157.5 RPM", fontsize=8, color=TEXT2)
    ax.set_ylabel("Speed (RPM)")
    ax.set_ylim(0, 180)
    ax.set_title(f"Final PI controller (Kp = 1.0, Ki = 10.0): step 0 -> 150 RPM\n"
                 f"overshoot {m['overshoot_pct']:.0f} % (<= 10 %),  settling {m['settling_s']:.2f} s "
                 f"(<= 1.0 s),  steady-state error {m['sse_rpm']:.1f} RPM (<= 3)",
                 fontsize=9.5, color=TEXT, loc="left")
    ax.legend(loc="lower right", fontsize=8)
    ax2.plot(ts, [s[1][3] for s in sel], color=ORANGE, lw=1.6)
    ax2.axhline(255, color=TEXT2, lw=1, ls="--")
    ax2.text(3.0, 232, "PWM limit 255", fontsize=8, color=TEXT2)
    ax2.set_ylabel("PWM duty")
    ax2.set_ylim(0, 270)
    ax2.set_xlabel("Time from step (s, simulation time)")
    ax2.set_xlim(-0.25, 4.0)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "step_final.png"))
    plt.close(fig)


def plot_tracking():
    rows = load(os.path.join(DATA, "tracking.csv"))
    t0 = next(r[0] for r in rows if r[1] != 0)
    t = [(r[0] - t0) / 1000 for r in rows]
    fig, ax = plt.subplots(figsize=(8.5, 4))
    ax.axhline(0, color=TEXT2, lw=0.8)
    ax.step(t, [r[1] for r in rows], where="post", color=TEXT2, lw=1.4, ls="--", label="Setpoint SP")
    ax.plot(t, [r[2] for r in rows], color=BLUE, lw=2, label="Measured speed PV")
    for x, txt in [(0.05, "150"), (2.05, "80"), (4.05, "-100"), (6.05, "0")]:
        ax.text(x, 168, f"SP {txt}", fontsize=8, color=TEXT2)
    ax.annotate("undershoot to 45 RPM:\nintegral wound down\nwhile braking", (3.40, 45), xytext=(1.2, -70),
                fontsize=8, color=TEXT, arrowprops=dict(arrowstyle="-", color=TEXT2, lw=0.8))
    ax.annotate("reverses through 0", (4.22, 0), xytext=(4.45, 40), fontsize=8, color=TEXT,
                arrowprops=dict(arrowstyle="-", color=TEXT2, lw=0.8))
    ax.annotate("overshoot to +24, then slow\ncreep back to 0 as the\nintegral unwinds", (6.7, 24),
                xytext=(7.4, 60), fontsize=8, color=TEXT,
                arrowprops=dict(arrowstyle="-", color=TEXT2, lw=0.8))
    ax.set_xlabel("Time from start of sequence (s, simulation time)")
    ax.set_ylabel("Speed (RPM)")
    ax.set_title("Tracking test: SP 150 -> 80 -> -100 -> 0 RPM, 2 s each (PI Kp = 1.0, Ki = 10.0)",
                 fontsize=10, color=TEXT, loc="left")
    ax.set_xlim(-0.25, t[-1])
    ax.set_ylim(-140, 180)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "tracking.png"))
    plt.close(fig)


if __name__ == "__main__":
    plot_openloop()
    plot_tuning()
    plot_final()
    plot_tracking()
    print("figures written to", os.path.abspath(OUT))
