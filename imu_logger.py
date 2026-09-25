#!/usr/bin/env python3
"""Serial logger for an ESP32-S3 streaming 3x MPU6050 IMU data over USB serial.

Usage:
    python imu_logger.py --subject S01 --condition walk
    python imu_logger.py --port COM9 --subject S01 --condition walk --trial 2

See README.md for full usage.
"""

import argparse
import csv
import json
import math
import queue
import sys
import threading
import time
from collections import namedtuple
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports
from pynput import keyboard

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

LABEL_MAP = {
    0: "unlabeled",
    1: "balanced",
    2: "perturbation",
    3: "recovery",
}

DEFAULT_SEGMENT_MAP = {
    "imu1": "pelvis",
    "imu2": "trunk",
    "imu3": "shank",
}

ACCEL_RANGE_G = 4
GYRO_RANGE_DPS = 500
TARGET_SAMPLE_RATE_HZ = 100

FLUSH_EVERY_N_ROWS = 50
FLUSH_INTERVAL_S = 1.0
STATUS_INTERVAL_S = 0.5
NAN_WARNING_THRESHOLD_S = 1.0
RECONNECT_ATTEMPTS = 5
RECONNECT_DELAY_S = 1.0

IMU_COLUMNS = []
for _i in (1, 2, 3):
    IMU_COLUMNS += [
        f"imu{_i}_ax", f"imu{_i}_ay", f"imu{_i}_az",
        f"imu{_i}_gyro_x", f"imu{_i}_gyro_y", f"imu{_i}_gyro_z",
    ]
CSV_HEADER = ["pc_time", "t_ms", "label"] + IMU_COLUMNS

RawLine = namedtuple("RawLine", ["text", "recv_time"])


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class SessionState:
    def __init__(self, initial_label=0, recording=True):
        self.lock = threading.Lock()
        self.label = initial_label
        self.recording = recording
        self.quit = False


# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------

class SerialReaderThread(threading.Thread):
    """Reads lines from the serial port in the background and pushes them
    onto a queue so the main loop never blocks on I/O."""

    def __init__(self, port, baud, out_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.ready_event = threading.Event()
        self.fatal_event = threading.Event()
        self.error_message = None

    def run(self):
        ser = None
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            ser.reset_input_buffer()
        except serial.SerialException as e:
            self.error_message = str(e)
            self.fatal_event.set()
            return

        self.ready_event.set()
        attempts_left = RECONNECT_ATTEMPTS

        while not self.stop_event.is_set():
            try:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                self.out_queue.put(RawLine(text, time.time()))
                attempts_left = RECONNECT_ATTEMPTS
            except (serial.SerialException, OSError) as e:
                self.error_message = str(e)
                try:
                    ser.close()
                except Exception:
                    pass

                reconnected = False
                while attempts_left > 0 and not self.stop_event.is_set():
                    attempts_left -= 1
                    time.sleep(RECONNECT_DELAY_S)
                    try:
                        ser = serial.Serial(self.port, self.baud, timeout=1)
                        ser.reset_input_buffer()
                        reconnected = True
                        break
                    except serial.SerialException:
                        continue

                if not reconnected:
                    self.fatal_event.set()
                    return

        try:
            ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def choose_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Plug in the board and try again, or pass --port explicitly.")
        sys.exit(1)
    print("Available serial ports:")
    for idx, p in enumerate(ports):
        print(f"  [{idx}] {p.device} - {p.description}")
    while True:
        sel = input("Select port index: ").strip()
        try:
            idx = int(sel)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except ValueError:
            pass
        print("Invalid selection, try again.")


def parse_segments(spec):
    if not spec:
        return dict(DEFAULT_SEGMENT_MAP)
    mapping = dict(DEFAULT_SEGMENT_MAP)
    for pair in spec.split(","):
        pair = pair.strip()
        if not pair:
            continue
        if "=" not in pair:
            raise ValueError(f"Invalid --segments entry: {pair!r} (expected imuN=segment)")
        key, val = pair.split("=", 1)
        mapping[key.strip()] = val.strip()
    return mapping


def parse_data_line(line):
    """Parse a CSV data line into (t_ms, [18 floats]). Returns None if malformed."""
    parts = line.split(",")
    if len(parts) != 19:
        return None
    try:
        t_ms = int(round(float(parts[0])))
        vals = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    return t_ms, vals


def fmt_value(v):
    return "" if math.isnan(v) else f"{v:.4f}"


def resolve_paths(data_dir, subject, condition, trial):
    data_dir.mkdir(parents=True, exist_ok=True)
    date_str = datetime.now().strftime("%Y%m%d")
    t = trial
    while True:
        stem = f"{date_str}_{subject}_{t:02d}_{condition}"
        csv_path = data_dir / f"{stem}.csv"
        if not csv_path.exists():
            break
        t += 1
    meta_path = data_dir / f"{stem}_meta.json"
    events_path = data_dir / f"{stem}_events.csv"
    return stem, csv_path, meta_path, events_path, t


def calibrate_gyro_bias(raw_queue, duration_s, status_lines, fatal_event):
    print(f"Calibrating gyro bias -- keep all IMUs still for {duration_s:.1f}s...")
    sums = {1: [0.0, 0.0, 0.0], 2: [0.0, 0.0, 0.0], 3: [0.0, 0.0, 0.0]}
    counts = {1: 0, 2: 0, 3: 0}
    end_time = time.time() + duration_s

    while time.time() < end_time:
        if fatal_event.is_set():
            break
        try:
            raw = raw_queue.get(timeout=0.2)
        except queue.Empty:
            continue

        text = raw.text
        if text.startswith("#"):
            print(text)
            status_lines.append(text)
            continue

        parsed = parse_data_line(text)
        if parsed is None:
            continue
        _t_ms, vals = parsed
        for i in (1, 2, 3):
            base = (i - 1) * 6
            gx, gy, gz = vals[base + 3], vals[base + 4], vals[base + 5]
            if not (math.isnan(gx) or math.isnan(gy) or math.isnan(gz)):
                sums[i][0] += gx
                sums[i][1] += gy
                sums[i][2] += gz
                counts[i] += 1

    bias = {}
    for i in (1, 2, 3):
        if counts[i] > 0:
            bias[f"imu{i}"] = {
                "gyro_x": sums[i][0] / counts[i],
                "gyro_y": sums[i][1] / counts[i],
                "gyro_z": sums[i][2] / counts[i],
                "samples": counts[i],
            }
        else:
            bias[f"imu{i}"] = {"gyro_x": None, "gyro_y": None, "gyro_z": None, "samples": 0}
    print("Calibration done:")
    for i in (1, 2, 3):
        b = bias[f"imu{i}"]
        if b["samples"] > 0:
            print(f"  imu{i}: bias=({b['gyro_x']:.3f}, {b['gyro_y']:.3f}, {b['gyro_z']:.3f}) deg/s "
                  f"from {b['samples']} samples")
        else:
            print(f"  imu{i}: no valid samples (not connected?)")
    return bias


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 3x MPU6050 serial logger")
    parser.add_argument("--port", default=None, help="Serial port (auto-list/prompt if omitted)")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--subject", default="S1")
    parser.add_argument("--condition", default="trial")
    parser.add_argument("--trial", type=int, default=1, help="Auto-increments if the file already exists")
    parser.add_argument("--data-dir", default="./data")
    parser.add_argument("--calib-seconds", type=float, default=2.0)
    parser.add_argument("--segments", default=None,
                         help="IMU-to-body-segment mapping, e.g. 'imu1=pelvis,imu2=trunk,imu3=shank'")
    args = parser.parse_args()

    try:
        segment_map = parse_segments(args.segments)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)

    port = args.port or choose_port()

    raw_queue = queue.Queue()
    stop_event = threading.Event()
    reader = SerialReaderThread(port, args.baud, raw_queue, stop_event)
    reader.start()

    if not reader.ready_event.wait(timeout=5) or reader.fatal_event.is_set():
        print(f"Could not open serial port {port}: {reader.error_message}")
        sys.exit(1)

    print(f"Connected to {port} @ {args.baud} baud")

    status_lines = []

    # Give the board a moment to print its startup status lines before calibrating.
    settle_end = time.time() + 1.0
    while time.time() < settle_end:
        try:
            raw = raw_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        if raw.text.startswith("#"):
            print(raw.text)
            status_lines.append(raw.text)
        else:
            # Not a comment -- put it back conceptually by re-queueing so no data is lost.
            raw_queue.put(raw)
            break

    bias = calibrate_gyro_bias(raw_queue, args.calib_seconds, status_lines, reader.fatal_event)

    if reader.fatal_event.is_set():
        print(f"Serial connection lost during calibration: {reader.error_message}")
        sys.exit(1)

    data_dir = Path(args.data_dir)
    stem, csv_path, meta_path, events_path, resolved_trial = resolve_paths(
        data_dir, args.subject, args.condition, args.trial
    )
    if resolved_trial != args.trial:
        print(f"Trial {args.trial:02d} already exists, using trial {resolved_trial:02d} instead.")

    start_time_iso = datetime.now().isoformat(timespec="milliseconds")

    meta = {
        "subject": args.subject,
        "condition": args.condition,
        "trial": resolved_trial,
        "start_time": start_time_iso,
        "port": port,
        "baud": args.baud,
        "target_sample_rate_hz": TARGET_SAMPLE_RATE_HZ,
        "accel_range_g": ACCEL_RANGE_G,
        "gyro_range_dps": GYRO_RANGE_DPS,
        "segment_map": segment_map,
        "gyro_bias": bias,
        "firmware_status": status_lines,
        "csv_file": csv_path.name,
        "events_file": events_path.name,
    }
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)

    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(CSV_HEADER)

    events_file = open(events_path, "w", newline="")
    events_writer = csv.writer(events_file)
    events_writer.writerow(["pc_time", "t_ms", "label"])
    events_file.flush()

    state = SessionState(initial_label=0, recording=True)
    last_t_ms_holder = [0]

    def on_press(key):
        try:
            char = key.char
        except AttributeError:
            char = None

        if key == keyboard.Key.space:
            with state.lock:
                state.recording = not state.recording
                rec = state.recording
            print(f"\n[{'RECORDING' if rec else 'PAUSED'}]")
            return

        if char is not None and char.isdigit():
            new_label = int(char)
            with state.lock:
                changed = state.label != new_label
                state.label = new_label
            if changed:
                pc_time = datetime.now().isoformat(timespec="milliseconds")
                events_writer.writerow([pc_time, last_t_ms_holder[0], new_label])
                events_file.flush()
                label_name = LABEL_MAP.get(new_label, str(new_label))
                print(f"\n[LABEL] {new_label} ({label_name})")
            return

        if char == "q":
            with state.lock:
                state.quit = True
            return False  # stop listener

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print()
    print(f"Recording to: {csv_path}")
    print(f"Labels: {LABEL_MAP}")
    print("Keys: 0-9 = set label | SPACE = pause/resume | q = quit and save")
    print()

    samples_total = 0
    dropped = 0
    rows_since_flush = 0
    last_flush_time = time.time()

    rate_counter = 0
    rate_window_start = time.time()
    last_status_time = time.time()

    session_start = time.time()
    last_valid_time = {1: session_start, 2: session_start, 3: session_start}
    nan_warned = {1: False, 2: False, 3: False}

    try:
        while True:
            with state.lock:
                if state.quit:
                    break

            if reader.fatal_event.is_set():
                print(f"\n[ERROR] Serial connection lost and could not reconnect: {reader.error_message}")
                break

            try:
                raw = raw_queue.get(timeout=0.1)
            except queue.Empty:
                raw = None

            now = time.time()

            if raw is not None:
                text = raw.text
                if text.startswith("#"):
                    print(f"\n{text}")
                    status_lines.append(text)
                else:
                    parsed = parse_data_line(text)
                    if parsed is None:
                        dropped += 1
                    else:
                        t_ms, vals = parsed
                        last_t_ms_holder[0] = t_ms
                        samples_total += 1
                        rate_counter += 1

                        for i in (1, 2, 3):
                            base = (i - 1) * 6
                            imu_vals = vals[base:base + 6]
                            if any(math.isnan(v) for v in imu_vals):
                                if not nan_warned[i] and (now - last_valid_time[i]) > NAN_WARNING_THRESHOLD_S:
                                    print(f"\n[WARN] IMU{i} has been NaN for >{NAN_WARNING_THRESHOLD_S:.0f}s")
                                    nan_warned[i] = True
                            else:
                                last_valid_time[i] = now
                                nan_warned[i] = False

                        with state.lock:
                            recording = state.recording
                            label = state.label

                        if recording:
                            pc_time = datetime.now().isoformat(timespec="milliseconds")
                            row = [pc_time, t_ms, label] + [fmt_value(v) for v in vals]
                            csv_writer.writerow(row)
                            rows_since_flush += 1
                            if rows_since_flush >= FLUSH_EVERY_N_ROWS or (now - last_flush_time) > FLUSH_INTERVAL_S:
                                csv_file.flush()
                                rows_since_flush = 0
                                last_flush_time = now

            if now - last_status_time >= STATUS_INTERVAL_S:
                elapsed = now - rate_window_start
                rate = rate_counter / elapsed if elapsed > 0 else 0.0
                with state.lock:
                    label = state.label
                    recording = state.recording
                print(
                    f"\rRate: {rate:5.1f} Hz | Label: {label} ({LABEL_MAP.get(label, '?')}) | "
                    f"{'REC' if recording else 'PAUSED'} | Samples: {samples_total} | Dropped: {dropped}   ",
                    end="", flush=True,
                )
                rate_counter = 0
                rate_window_start = now
                last_status_time = now

    except KeyboardInterrupt:
        print("\nInterrupted, saving and exiting...")

    finally:
        listener.stop()
        stop_event.set()
        reader.join(timeout=2)

        csv_file.flush()
        csv_file.close()
        events_file.flush()
        events_file.close()

        end_time = time.time()
        meta["end_time"] = datetime.now().isoformat(timespec="milliseconds")
        meta["duration_s"] = round(end_time - session_start, 3)
        meta["total_samples"] = samples_total
        meta["dropped_lines"] = dropped
        duration = end_time - session_start
        meta["actual_sample_rate_hz"] = round(samples_total / duration, 2) if duration > 0 else None
        meta["firmware_status"] = status_lines
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)

        print(f"\nSaved {samples_total} samples ({dropped} dropped) to {csv_path}")
        print(f"Meta: {meta_path}")
        print(f"Events: {events_path}")


if __name__ == "__main__":
    main()
