"""
Performance-metric analyzer for the ESP32-S3 standalone-controller runs.

Reads a telemetry CSV (columns: reference, position, control, time_us), segments
the multi-step reference, and computes the four metrics reported in the paper:

  e_ss  : mean absolute tracking error over the final 20% of each step segment [deg]
  t_s   : time (from the step) at which the output last leaves the +/-2% band of
          the step amplitude [s]
  M_p   : maximum overshoot beyond the final value, as % of the step amplitude
  Var u : total variation of the control signal, sum |u[k] - u[k-1]|

Usage:
    python analyze_metrics.py                 # newest Prueba_Control_*.csv in the folder
    python analyze_metrics.py data/pid_run.csv
    python analyze_metrics.py data/pid_run.csv --no-plot
"""

import sys
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def _find_columns(df):
    """Detect the ref / pos / control / time columns (Spanish or English headers)."""
    cols = {c.lower(): c for c in df.columns}

    def pick(keys):
        for k in keys:
            for lc, orig in cols.items():
                if k in lc:
                    return orig
        return None

    return (pick(['ref']),
            pick(['pos']),
            pick(['control', 'pwm', 'ctrl', 'salida']),
            pick(['time', 'tiempo', 'timestamp']))


def analyze(path, plot=True):
    df = pd.read_csv(path)
    c_ref, c_pos, c_ctrl, c_time = _find_columns(df)
    if None in (c_ref, c_pos, c_ctrl, c_time):
        print("[ERROR] Columns not recognized:", list(df.columns))
        return []

    time_us = df[c_time].to_numpy(dtype=float)
    t = (time_us - time_us[0]) / 1e6           # seconds, relative
    r = df[c_ref].to_numpy(dtype=float)
    y = df[c_pos].to_numpy(dtype=float)
    u = df[c_ctrl].to_numpy(dtype=float)

    # segment boundaries = where the reference changes
    changes = np.where(np.abs(np.diff(r)) > 0.1)[0]
    starts = [0] + list(changes + 1)

    print(f"\nFile: {os.path.basename(path)}   ({len(df)} samples, {t[-1]:.2f} s)")
    print(f"{'Transition':<12}{'e_ss[deg]':>11}{'t_s[s]':>9}{'M_p[%]':>9}{'Var u':>12}")
    print("-" * 53)

    results = []
    for i, a in enumerate(starts):
        b = starts[i + 1] if i < len(starts) - 1 else len(df) - 1
        t_seg = t[a:b] - t[a]
        r_seg, y_seg, u_seg = r[a:b], y[a:b], u[a:b]
        if len(r_seg) < 10:
            continue

        ref_final = float(np.mean(r_seg[-50:]))
        ref_init = float(r[a - 1]) if a > 0 else 0.0
        step = ref_final - ref_init
        if abs(step) < 1.0:                    # skip "no-change" segments
            continue

        # e_ss : mean |error| over the last 20% of the segment
        win = max(1, int(len(y_seg) * 0.2))
        e_ss = float(np.mean(np.abs(r_seg[-win:] - y_seg[-win:])))

        # M_p : max overshoot beyond the final value, % of step amplitude
        if step > 0:
            m_p = (float(np.max(y_seg)) - ref_final) / abs(step) * 100.0
        else:
            m_p = (ref_final - float(np.min(y_seg))) / abs(step) * 100.0
        m_p = max(0.0, m_p)

        # t_s : last instant outside the +/-2% band of the step amplitude
        band = 0.02 * abs(step)
        outside = np.where(np.abs(y_seg - ref_final) > band)[0]
        t_s = float(t_seg[outside[-1]]) if len(outside) > 0 else 0.0

        # Var u : total variation of the control signal
        var_u = float(np.sum(np.abs(np.diff(u_seg))))

        label = f"{ref_init:.0f}->{ref_final:.0f}"
        print(f"{label:<12}{e_ss:>11.2f}{t_s:>9.2f}{m_p:>9.2f}{var_u:>12.1f}")
        results.append({'transition': label, 'e_ss': e_ss, 't_s': t_s,
                        'M_p': m_p, 'Var_u': var_u})

    print("-" * 53)
    print("Note: any final N->0 segment is the safety return to zero and is NOT part\n"
          "of the reported multi-step reference (0->60, 60->100, 100->180).")
    print("Var u is expressed in the raw control units logged by the firmware.")

    if plot:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        ax1.plot(t, r, 'g--', lw=1.2, label='Reference')
        ax1.plot(t, y, 'b-',  lw=1.5, label='Position')
        ax1.set_ylabel('Position [deg]')
        ax1.legend(loc='lower right'); ax1.grid(True)
        ax2.plot(t, u, 'r-', lw=1.0, label='Control')
        ax2.set_ylabel('Control [PWM, 10-bit]')
        ax2.set_xlabel('Time [s]')
        ax2.legend(loc='upper right'); ax2.grid(True)
        plt.tight_layout()
        plt.show()

    return results


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    do_plot = '--no-plot' not in sys.argv

    if args:
        csv_path = args[0]
    else:
        candidates = sorted(glob.glob("Prueba_Control_*.csv"))
        if not candidates:
            print("Usage: python analyze_metrics.py <file.csv> [--no-plot]")
            raise SystemExit(1)
        csv_path = candidates[-1]
        print(f"[info] no file given; using most recent: {csv_path}")

    analyze(csv_path, plot=do_plot)
