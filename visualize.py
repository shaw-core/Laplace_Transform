#!/usr/bin/env python3
"""Visualise 3x MPU6050 data from imu_stream.ino / imu_logger.py.

Two modes:

  live    Real-time scrolling tilt plot, straight from the ESP32 serial port
          or replayed from a recorded trial (handy for filming the demo).

              python visualize.py live --port COM9
              python visualize.py live --replay data/20260925_S01_01_trial.csv

  trial   Offline report for one recorded trial: tilt traces for every IMU
          with the label phases shaded, plus per-perturbation recovery
          metrics (peak tilt, time to stabilisation) and a left/right summary.

              python visualize.py trial data/20260925_S01_01_trial.csv --board imu1

Tilt is estimated per IMU with a complementary filter (gyro integration
corrected by the accelerometer gravity direction).
"""

import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

ALPHA = 0.98                 # complementary filter weight on the gyro path
LIVE_WINDOW_S = 10.0         # seconds shown in live mode
LIVE_REFRESH_MS = 50         # plot refresh interval in live mode

# Recovery metrics (applied to the board IMU)
STABLE_TILT_DEG = 2.0        # |tilt - baseline| must stay below this ...
STABLE_RATE_DPS = 10.0       # ... and |angular rate| below this ...
STABLE_HOLD_S = 0.5          # ... for this long to count as stabilised
ANALYSIS_WINDOW_S = 3.0      # window after onset used for peak tilt / TTS
BASELINE_WINDOW_S = 0.5      # window before onset used as the baseline

LABEL_NAMES = {0: "unlabeled", 1: "balanced", 2: "perturbation", 3: "recovery"}
LABEL_COLOURS = {1: "#d9ead3", 2: "#f4cccc", 3: "#fff2cc"}
PERTURBATION_LABEL = 2

IMU_IDS = (1, 2, 3)
IMU_COLS = [f"imu{i}_{c}" for i in IMU_IDS
            for c in ("ax", "ay", "az", "gyro_x", "gyro_y", "gyro_z")]


# ---------------------------------------------------------------------------
# Orientation
# ---------------------------------------------------------------------------

class ComplementaryFilter:
    """Roll/pitch from one IMU. Roll is about the sensor x axis, pitch about y."""

    def __init__(self, alpha=ALPHA):
        self.alpha = alpha
        self.roll = None
        self.pitch = None

    def update(self, ax, ay, az, gx, gy, dt):
        if any(math.isnan(v) for v in (ax, ay, az, gx, gy)):
            return self.roll, self.pitch
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        if self.roll is None or dt <= 0 or dt > 0.5:
            self.roll, self.pitch = roll_acc, pitch_acc
        else:
            a = self.alpha
            self.roll = a * (self.roll + gx * dt) + (1 - a) * roll_acc
            self.pitch = a * (self.pitch + gy * dt) + (1 - a) * pitch_acc
        return self.roll, self.pitch


def add_tilt_columns(df):
    """Add imuN_roll / imuN_pitch (deg) columns to a trial DataFrame."""
    t = df["t_ms"].to_numpy() / 1000.0
    dt = np.diff(t, prepend=t[0])
    for i in IMU_IDS:
        f = ComplementaryFilter()
        cols = [f"imu{i}_{c}" for c in ("ax", "ay", "az", "gyro_x", "gyro_y")]
        vals = df[cols].to_numpy(dtype=float)
        roll = np.full(len(df), np.nan)
        pitch = np.full(len(df), np.nan)
        for k in range(len(df)):
            r, p = f.update(*vals[k], dt[k])
            if r is not None:
                roll[k], pitch[k] = r, p
        df[f"imu{i}_roll"] = roll
        df[f"imu{i}_pitch"] = pitch
    return df


# ---------------------------------------------------------------------------
# Trial loading
# ---------------------------------------------------------------------------

def load_any_trial(csv_path):
    """Load a trial via load_trial.py (bias-corrected) when its meta file
    exists, otherwise read the CSV as-is."""
    csv_path = Path(csv_path)
    try:
        from load_trial import load_trial
        df, meta = load_trial(csv_path)
    except FileNotFoundError:
        df, meta = pd.read_csv(csv_path), {}
    df = df.sort_values("t_ms").reset_index(drop=True)
    return df, meta


def segment_names(meta):
    seg = meta.get("segment_map", {}) if meta else {}
    return {i: seg.get(f"imu{i}", f"imu{i}") for i in IMU_IDS}


# ---------------------------------------------------------------------------
# Recovery metrics
# ---------------------------------------------------------------------------

def perturbation_onsets(df):
    """Times (s) where the label switches into 'perturbation'."""
    lab = df["label"].to_numpy()
    t = df["t_ms"].to_numpy() / 1000.0
    idx = np.where((lab[1:] == PERTURBATION_LABEL) & (lab[:-1] != PERTURBATION_LABEL))[0] + 1
    return t[idx]


def time_to_stabilise(t, tilt, rate, baseline):
    """First time after t[0] from which tilt stays within STABLE_TILT_DEG of
    baseline and |rate| within STABLE_RATE_DPS for STABLE_HOLD_S."""
    ok = (np.abs(tilt - baseline) < STABLE_TILT_DEG) & (np.abs(rate) < STABLE_RATE_DPS)
    if len(t) < 2:
        return np.nan
    hold = max(1, int(round(STABLE_HOLD_S / np.median(np.diff(t)))))
    run = 0
    for k, good in enumerate(ok):
        run = run + 1 if good else 0
        if run >= hold:
            return t[k - hold + 1] - t[0]
    return np.nan


def recovery_metrics(df, board):
    """One row per perturbation onset, computed on the board IMU."""
    t = df["t_ms"].to_numpy() / 1000.0
    t_start = t[0]
    rows = []
    for n, t0 in enumerate(perturbation_onsets(df), start=1):
        pre = (t >= t0 - BASELINE_WINDOW_S) & (t < t0)
        post = (t >= t0) & (t <= t0 + ANALYSIS_WINDOW_S)
        if post.sum() < 5:
            continue
        row = {"event": n, "onset_s": round(t0 - t_start, 3)}
        for axis, gyro in (("roll", "gyro_x"), ("pitch", "gyro_y")):
            tilt = df[f"{board}_{axis}"].to_numpy()
            # 50 ms moving average so sensor noise does not reset the stability run
            rate = df[f"{board}_{gyro}"].rolling(5, center=True, min_periods=1).mean().to_numpy()
            base = np.nanmean(tilt[pre]) if pre.any() else tilt[post][0]
            dev = tilt[post] - base
            k = int(np.nanargmax(np.abs(dev)))
            row[f"peak_{axis}_deg"] = round(float(dev[k]), 2)
            tts = time_to_stabilise(t[post], tilt[post], rate[post], base)
            row[f"tts_{axis}_s"] = round(float(tts), 3) if not np.isnan(tts) else np.nan
        # Direction inferred from the sign of the first large roll excursion.
        row["direction"] = "left" if row["peak_roll_deg"] < 0 else "right"
        rows.append(row)
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Trial report
# ---------------------------------------------------------------------------

def shade_labels(ax, t, labels):
    if len(t) == 0:
        return
    start = 0
    for k in range(1, len(labels) + 1):
        if k == len(labels) or labels[k] != labels[start]:
            colour = LABEL_COLOURS.get(int(labels[start]))
            if colour:
                ax.axvspan(t[start], t[k - 1], color=colour, lw=0, zorder=0)
            start = k


def plot_trial(csv_path, board, out_path=None, show=True):
    df, meta = load_any_trial(csv_path)
    df = add_tilt_columns(df)
    names = segment_names(meta)
    t = (df["t_ms"].to_numpy() - df["t_ms"].iloc[0]) / 1000.0
    t_abs0 = df["t_ms"].iloc[0] / 1000.0
    labels = df["label"].to_numpy()
    onsets = perturbation_onsets(df) - t_abs0

    metrics = recovery_metrics(df, board)

    fig = plt.figure(figsize=(13, 9), constrained_layout=True)
    gs = fig.add_gridspec(4, 3)
    title = Path(csv_path).stem
    if meta:
        title += f"  |  subject {meta.get('subject')}  |  {meta.get('condition')}"
    fig.suptitle(title, fontsize=13, fontweight="bold")

    for row, i in enumerate(IMU_IDS):
        ax = fig.add_subplot(gs[row, :])
        shade_labels(ax, t, labels)
        ax.plot(t, df[f"imu{i}_roll"], lw=1.2, label="roll (about x)")
        ax.plot(t, df[f"imu{i}_pitch"], lw=1.2, label="pitch (about y)")
        for o in onsets:
            ax.axvline(o, color="#cc0000", lw=0.8, ls="--")
        tag = "  (board)" if f"imu{i}" == board and names[i] != "board" else ""
        ax.set_ylabel(f"{names[i]}{tag}\ntilt (deg)")
        ax.grid(alpha=0.3)
        if row == 0:
            ax.legend(loc="upper right", fontsize=8)
        if row == 2:
            ax.set_xlabel("time (s)")

    ax_bar = fig.add_subplot(gs[3, 0])
    ax_peak = fig.add_subplot(gs[3, 1])
    ax_txt = fig.add_subplot(gs[3, 2])
    ax_txt.axis("off")

    if metrics.empty:
        for a in (ax_bar, ax_peak):
            a.text(0.5, 0.5, "no perturbation labels (label 2) in this trial",
                   ha="center", va="center", fontsize=9, transform=a.transAxes)
            a.set_axis_off()
    else:
        summary = metrics.groupby("direction").agg(
            n=("event", "count"),
            tts_mean=("tts_roll_s", "mean"),
            tts_sd=("tts_roll_s", "std"),
            peak_mean=("peak_roll_deg", lambda s: np.mean(np.abs(s))),
            peak_sd=("peak_roll_deg", lambda s: np.std(np.abs(s), ddof=1) if len(s) > 1 else 0),
        ).reindex(["left", "right"])
        dirs = summary.index.tolist()
        colours = ["#3d85c6", "#e69138"]
        ax_bar.bar(dirs, summary["tts_mean"], yerr=summary["tts_sd"].fillna(0),
                   color=colours, capsize=6)
        ax_bar.set_title("Time to stabilisation (board roll)", fontsize=10)
        ax_bar.set_ylabel("s")
        ax_bar.grid(axis="y", alpha=0.3)
        ax_peak.bar(dirs, summary["peak_mean"], yerr=summary["peak_sd"].fillna(0),
                    color=colours, capsize=6)
        ax_peak.set_title("Peak tilt (board roll)", fontsize=10)
        ax_peak.set_ylabel("deg")
        ax_peak.grid(axis="y", alpha=0.3)

        lines = [f"Perturbations: {len(metrics)}"]
        for d in dirs:
            if not np.isnan(summary.loc[d, "n"]):
                lines.append(f"{d:>5}: n={int(summary.loc[d, 'n'])}, "
                             f"TTS {summary.loc[d, 'tts_mean']:.2f}s, "
                             f"peak {summary.loc[d, 'peak_mean']:.1f} deg")
        l, r = summary.loc["left", "tts_mean"], summary.loc["right", "tts_mean"]
        if not (np.isnan(l) or np.isnan(r)) and (l + r) > 0:
            lines.append(f"Asymmetry (TTS): {100 * (l - r) / ((l + r) / 2):+.0f}%  (+ = left slower)")
        lines += ["", "Direction inferred from sign of board roll.",
                  f"Stable = |tilt| < {STABLE_TILT_DEG} deg and |rate| < {STABLE_RATE_DPS} deg/s",
                  f"held for {STABLE_HOLD_S} s."]
        ax_txt.text(0, 1, "\n".join(lines), va="top", family="monospace", fontsize=9)

    legend_patches = [plt.Rectangle((0, 0), 1, 1, color=c) for c in LABEL_COLOURS.values()]
    fig.legend(legend_patches, [LABEL_NAMES[k] for k in LABEL_COLOURS],
               loc="upper left", ncol=3, fontsize=8, frameon=False)

    out_path = Path(out_path) if out_path else Path(csv_path).with_name(Path(csv_path).stem + "_report.png")
    fig.savefig(out_path, dpi=150)
    metrics_path = out_path.with_name(Path(csv_path).stem + "_metrics.csv")
    metrics.to_csv(metrics_path, index=False)
    print(f"Saved figure:  {out_path}")
    print(f"Saved metrics: {metrics_path}")
    if not metrics.empty:
        print(metrics.to_string(index=False))
    if show:
        plt.show()
    plt.close(fig)
    return metrics


# ---------------------------------------------------------------------------
# Live view
# ---------------------------------------------------------------------------

def parse_line(line):
    """Parse one firmware CSV line into (t_ms, 18 floats) or None."""
    parts = line.strip().split(",")
    if len(parts) != 19:
        return None
    try:
        return float(parts[0]), [float(p) for p in parts[1:]]
    except ValueError:
        return None


def serial_source(port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=0.05)
    ser.reset_input_buffer()
    while True:
        raw = ser.readline()
        if not raw:
            yield None
            continue
        text = raw.decode("utf-8", errors="replace")
        if text.startswith("#"):
            print(text.strip())
            yield None
            continue
        yield parse_line(text)


def replay_source(csv_path, speed):
    df, _ = load_any_trial(csv_path)
    rows = df[["t_ms"] + IMU_COLS + ["label"]].to_numpy(dtype=float)
    t0_data = rows[0, 0]
    t0_wall = time.time()
    k = 0
    while k < len(rows):
        due = t0_wall + (rows[k, 0] - t0_data) / 1000.0 / speed
        if time.time() < due:
            yield None
            continue
        yield rows[k, 0], list(rows[k, 1:19]), int(rows[k, 19])
        k += 1
    while True:
        yield "EOF"


def run_live(source, names, board):
    filters = {i: ComplementaryFilter() for i in IMU_IDS}
    maxlen = int(LIVE_WINDOW_S * 120)
    buf_t = deque(maxlen=maxlen)
    buf = {(i, a): deque(maxlen=maxlen) for i in IMU_IDS for a in ("roll", "pitch")}
    state = {"last_t": None, "count": 0, "rate_t": time.time(), "rate": 0.0, "label": None}

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle("Live IMU tilt", fontsize=13, fontweight="bold")
    lines = {}
    for ax, i in zip(axes, IMU_IDS):
        lines[(i, "roll")], = ax.plot([], [], lw=1.4, label="roll")
        lines[(i, "pitch")], = ax.plot([], [], lw=1.4, label="pitch")
        tag = "  (board)" if f"imu{i}" == board and names[i] != "board" else ""
        ax.set_ylabel(f"{names[i]}{tag}\ntilt (deg)")
        ax.set_ylim(-25, 25)
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time (s)")
    status = fig.text(0.01, 0.01, "", family="monospace", fontsize=9)

    def consume():
        for _ in range(200):
            item = next(source)
            if item is None:
                return
            if item == "EOF":
                status.set_text("replay finished")
                return
            if len(item) == 3:
                t_ms, vals, label = item
                state["label"] = label
            else:
                t_ms, vals = item
            t = t_ms / 1000.0
            dt = 0.0 if state["last_t"] is None else t - state["last_t"]
            state["last_t"] = t
            buf_t.append(t)
            for i in IMU_IDS:
                b = (i - 1) * 6
                ax_, ay_, az_, gx_, gy_ = vals[b:b + 5]
                r, p = filters[i].update(ax_, ay_, az_, gx_, gy_, dt)
                buf[(i, "roll")].append(np.nan if r is None else r)
                buf[(i, "pitch")].append(np.nan if p is None else p)
            state["count"] += 1

    def update(_frame):
        consume()
        if not buf_t:
            return list(lines.values()) + [status]
        t = np.fromiter(buf_t, float)
        for key, ln in lines.items():
            ln.set_data(t, np.fromiter(buf[key], float))
        for ax in axes:
            ax.set_xlim(t[-1] - LIVE_WINDOW_S, t[-1])
        now = time.time()
        if now - state["rate_t"] >= 1.0:
            state["rate"] = state["count"] / (now - state["rate_t"])
            state["count"], state["rate_t"] = 0, now
        board_i = int(board[-1])
        roll = buf[(board_i, "roll")][-1]
        pitch = buf[(board_i, "pitch")][-1]
        label = "" if state["label"] is None else f" | label: {LABEL_NAMES.get(state['label'], state['label'])}"
        if not status.get_text().startswith("replay finished"):
            status.set_text(f"{state['rate']:5.1f} Hz | board roll {roll:+6.1f} deg, "
                            f"pitch {pitch:+6.1f} deg{label}")
        return list(lines.values()) + [status]

    anim = FuncAnimation(fig, update, interval=LIVE_REFRESH_MS, blit=False, cache_frame_data=False)
    plt.show()
    return anim


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="Visualise 3x MPU6050 IMU data")
    sub = p.add_subparsers(dest="mode", required=True)

    pl = sub.add_parser("live", help="real-time tilt plot from serial or a replayed trial")
    src = pl.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the ESP32-S3, e.g. COM9 or /dev/ttyACM0")
    src.add_argument("--replay", help="recorded trial CSV to replay")
    pl.add_argument("--baud", type=int, default=921600)
    pl.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    pl.add_argument("--board", default="imu1", help="which IMU is on the balance board")
    pl.add_argument("--segments", default=None, help="e.g. 'imu1=board,imu2=platform,imu3=shank'")

    pt = sub.add_parser("trial", help="offline report for a recorded trial")
    pt.add_argument("csv_path")
    pt.add_argument("--board", default="imu1", help="which IMU is on the balance board")
    pt.add_argument("--out", default=None, help="output PNG path")
    pt.add_argument("--no-show", action="store_true", help="save the figure without opening a window")

    args = p.parse_args()

    if args.mode == "trial":
        plot_trial(args.csv_path, args.board, args.out, show=not args.no_show)
        return

    names = {i: f"imu{i}" for i in IMU_IDS}
    if args.segments:
        for pair in args.segments.split(","):
            if "=" in pair:
                k, v = pair.split("=", 1)
                if k.strip().startswith("imu") and k.strip()[-1].isdigit():
                    names[int(k.strip()[-1])] = v.strip()
    if args.replay:
        _, meta = load_any_trial(args.replay)
        if meta and not args.segments:
            names = segment_names(meta)
        source = replay_source(args.replay, args.speed)
    else:
        source = serial_source(args.port, args.baud)
    run_live(source, names, args.board)


if __name__ == "__main__":
    sys.exit(main())
