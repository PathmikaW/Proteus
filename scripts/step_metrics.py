"""
step_metrics.py - step-response metrics from a slave CSV log (IT6512, 258843A)

The slave logs "t_ms,SP,PV,duty,status" every 50 ms. Lines starting with '#'
are comments. The step starts at the first sample where SP changes to the
final setpoint.

Metrics (all times are Proteus simulation time):
  rise time  : 10 % -> 90 % of the setpoint (or of the final value if the
               response never reaches the setpoint, as with P-only control)
  overshoot  : (max PV - SP) / SP * 100 %
  settling   : time after the step until PV stays within +-5 % of SP
  SSE        : |SP - mean PV over the last 1 s|

Usage:  python scripts/step_metrics.py data/tune_P1.csv [more files ...]
"""
import csv
import sys


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("t_ms"):
                continue
            t, sp, pv, duty, st = (int(x) for x in line.split(",")[:5])
            rows.append((t, sp, pv, duty, st))
    return rows


def metrics(rows):
    sp_final = rows[-1][1]
    i0 = next(i for i, r in enumerate(rows) if r[1] == sp_final and (i == 0 or rows[i - 1][1] != sp_final))
    t0 = rows[i0][0]
    step = rows[i0:]
    t_end = step[-1][0]
    final = [r[2] for r in step if r[0] > t_end - 1000]
    pv_final = sum(final) / len(final)
    sse = abs(sp_final - pv_final)

    ref = sp_final if max(r[2] for r in step) >= 0.9 * sp_final else pv_final
    t10 = next((r[0] for r in step if r[2] >= 0.1 * ref), None)
    t90 = next((r[0] for r in step if r[2] >= 0.9 * ref), None)
    rise = (t90 - t10) / 1000 if t10 is not None and t90 is not None else None

    pv_max = max(r[2] for r in step)
    overshoot = max(0.0, (pv_max - sp_final) / sp_final * 100)

    band = 0.05 * sp_final
    outside = [i for i, r in enumerate(step) if abs(r[2] - sp_final) > band]
    if not outside:
        settling = 0.0
    elif outside[-1] == len(step) - 1:
        settling = None                     # never settles inside +-5 %
    else:
        settling = (step[outside[-1] + 1][0] - t0) / 1000

    return {
        "sp": sp_final, "t_step_s": t0 / 1000, "pv_final": round(pv_final, 1),
        "rise_s": rise, "rise_ref": "SP" if ref == sp_final else "final",
        "overshoot_pct": round(overshoot, 1), "settling_s": settling,
        "sse_rpm": round(sse, 1), "duty_final": step[-1][3],
    }


if __name__ == "__main__":
    for path in sys.argv[1:]:
        m = metrics(load(path))
        print(path, m)
