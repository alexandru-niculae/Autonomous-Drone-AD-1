#!/usr/bin/env python3
"""AD-1 - EDF vibration bench test, host side.

Usage
  pip install pyserial numpy pandas scipy matplotlib
  python vibration_test.py capture /dev/cu.usbmodemXXXX     # runs the test, writes vib_<date>.csv
  python vibration_test.py analyze vib_<date>.csv           # spectra, peaks, RMS, saturation

During capture, Ctrl+C sends the stop command to the board (motor off).
"""
import sys
import time
import datetime
from pathlib import Path

import numpy as np
import pandas as pd

GYRO_AXES = ["gx_dps", "gy_dps", "gz_dps"]
ACC_AXES = ["ax_g", "ay_g", "az_g"]


# --------------------------------------------------------------------------- capture
def capture(port: str) -> Path:
    import serial

    out = Path(f"vib_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv")
    ser = serial.Serial(port, 115200, timeout=1)
    print("Waiting for the board...")
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("#ERROR"):
            sys.exit(f"Board reports: {line}")
        if line.startswith("#READY"):
            break

    input("Board ready. EDF fixed on the bench, area clear? Press Enter to start (Ctrl+C = stop) ")
    ser.write(b"GO\n")

    n_rows = 0
    try:
        with out.open("w") as f:
            started = False
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                if line == "#START":
                    started = True
                    continue
                if not started:
                    continue
                if line in ("#DONE", "#ABORTED"):
                    print(line)
                    break
                if line.startswith("# step") or line.startswith("# arming"):
                    print(line[2:])
                f.write(line + "\n")
                if not line.startswith("#") and not line.startswith("step"):
                    n_rows += 1
    except KeyboardInterrupt:
        ser.write(b"s")
        time.sleep(0.2)
        print("\nStop sent to the board.")
    finally:
        ser.close()

    print(f"{n_rows} samples written to {out}")
    return out


# --------------------------------------------------------------------------- analysis
def read_csv(path: Path):
    meta = {}
    for line in path.read_text().splitlines():
        if line.startswith("#") and "=" in line:
            k, v = line[1:].strip().split("=", 1)
            try:
                meta[k.strip()] = float(v)
            except ValueError:
                pass
    df = pd.read_csv(path, comment="#")
    # CSV from another logger: rebuild the optional columns if they are missing
    if "step" not in df:
        df["step"] = 0
    if "throttle_pct" not in df:
        df["throttle_pct"] = 0
    for flag, axes in (("gyr_new", GYRO_AXES), ("acc_new", ACC_AXES)):
        if flag not in df:
            df[flag] = (df[axes].diff().abs().sum(axis=1) != 0).astype(int)
            df.loc[df.index[0], flag] = 1
    return df, meta


def spectrum(t, x, nperseg=1024):
    """Amplitude spectral density of a (nearly) uniformly sampled signal."""
    from scipy.signal import welch

    fs = (len(t) - 1) / (t[-1] - t[0])                   # mean rate (accel: ~1600 Hz)
    t_u = np.arange(t[0], t[-1], 1.0 / fs)               # resample on a uniform grid
    x_u = np.interp(t_u, t, x)
    f, p = welch(x_u - x_u.mean(), fs=fs, nperseg=min(nperseg, len(x_u)))
    return f, p, fs


def total_spectrum(sub, axes, flag):
    s = sub[sub[flag] == 1]
    t = s["t_s"].to_numpy()
    psd = None
    for ax in axes:
        f, p, fs = spectrum(t, s[ax].to_numpy())
        psd = p if psd is None else psd + p
    return f, np.sqrt(psd), fs                           # combined 3-axis ASD


def top_peaks(f, asd, n=3, fmin=5.0):
    from scipy.signal import find_peaks

    m = f >= fmin
    idx, _ = find_peaks(asd[m], prominence=asd[m].max() * 0.05)
    idx = idx[np.argsort(asd[m][idx])[::-1][:n]]
    return [(f[m][i], asd[m][i]) for i in sorted(idx, key=lambda i: -asd[m][i])]


def analyze(path_str: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = Path(path_str)
    df, meta = read_csv(path)
    kv, vbat = meta.get("motor_kv", 1900), meta.get("vbat_v", 22.2)
    acc_lim, gyr_lim = meta.get("acc_range_g", 24), meta.get("gyr_range_dps", 2000)

    def f_motor(pct):            # rough: rpm ~ KV x V x 0.8 (loaded) x throttle
        return kv * vbat * 0.8 * pct / 100 / 60

    fig, (axg, axa) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    lines = [f"Vibration analysis - {path.name}", f"KV={kv:.0f}, Vbat={vbat:.1f} V", ""]
    hdr = f"{'thr%':>4} | {'gyro RMS dps':>12} | {'acc RMS g':>9} | {'sat':>4} | gyro peaks Hz | acc peaks Hz | expected Hz"
    lines += [hdr, "-" * len(hdr)]
    peak_rows = []

    for step, sub in df.groupby("step"):
        pct = int(sub["throttle_pct"].iloc[0])
        fg, ag, fsg = total_spectrum(sub, GYRO_AXES, "gyr_new")
        fa, aa, fsa = total_spectrum(sub, ACC_AXES, "acc_new")
        axg.semilogy(fg, ag, label=f"{pct} %")
        axa.semilogy(fa, aa, label=f"{pct} %")

        g = sub[GYRO_AXES].to_numpy(); a = sub[ACC_AXES].to_numpy()
        g_rms = np.sqrt(((g - g.mean(0)) ** 2).sum(1).mean())
        a_rms = np.sqrt(((a - a.mean(0)) ** 2).sum(1).mean())
        sat = int((np.abs(a) >= 0.99 * acc_lim).sum() + (np.abs(g) >= 0.99 * gyr_lim).sum())
        pg, pa = top_peaks(fg, ag), top_peaks(fa, aa)
        peak_rows += [(pct, f, "gyro") for f, _ in pg] + [(pct, f, "acc") for f, _ in pa]
        fmt = lambda pk: ", ".join(f"{f:.0f}" for f, _ in pk) or "-"
        lines.append(f"{pct:>4} | {g_rms:>12.2f} | {a_rms:>9.3f} | {sat:>4} | {fmt(pg):<13} | {fmt(pa):<12} | {f_motor(pct):.0f}")

    for ax, title, unit in ((axg, "Gyro (3 axes combined)", "dps/sqrt(Hz)"), (axa, "Accel (3 axes combined)", "g/sqrt(Hz)")):
        for pct in sorted(df["throttle_pct"].unique()):
            if pct > 0:
                ax.axvline(f_motor(pct), ls="--", lw=0.7, color="grey")
        ax.set_title(title + "  -  dashed: expected motor rotation frequency")
        ax.set_ylabel(unit); ax.grid(True, which="both", alpha=0.3); ax.legend(title="throttle")
    axa.set_xlabel("Frequency [Hz]")
    fig.tight_layout()
    fig.savefig(path.with_name(path.stem + "_spectra.png"), dpi=130)

    # Peak frequency vs throttle: a peak following the dashed line comes from the motor,
    # a horizontal row of peaks is a structural resonance.
    fig2, ax2 = plt.subplots(figsize=(8, 5))
    pr = pd.DataFrame(peak_rows, columns=["pct", "f", "sensor"])
    for sensor, mk in (("gyro", "o"), ("acc", "x")):
        s = pr[pr.sensor == sensor]
        ax2.scatter(s.pct, s.f, marker=mk, label=sensor)
    th = np.linspace(0, 100, 50)
    ax2.plot(th, f_motor(th), "k--", lw=0.8, label="expected motor 1x")
    ax2.set_xlabel("Throttle [%]"); ax2.set_ylabel("Peak frequency [Hz]")
    ax2.grid(alpha=0.3); ax2.legend()
    fig2.tight_layout()
    fig2.savefig(path.with_name(path.stem + "_peaks.png"), dpi=130)

    lines += ["", "sat = samples at >= 99 % of sensor range (should be 0)"]
    report = "\n".join(lines)
    path.with_name(path.stem + "_summary.txt").write_text(report + "\n")
    print(report)
    print(f"\nFigures: {path.stem}_spectra.png, {path.stem}_peaks.png")


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("capture", "analyze"):
        sys.exit(__doc__)
    if sys.argv[1] == "capture":
        csv_path = capture(sys.argv[2])
        if csv_path.exists() and csv_path.stat().st_size > 0:
            analyze(str(csv_path))
    else:
        analyze(sys.argv[2])
