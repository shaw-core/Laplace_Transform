# Ankle Rehab — Adaptive Perturbation Balance Platform

Sensing and visualisation code for the balance platform: an ESP32-S3 reads
three MPU6050 IMUs and streams them over USB serial at ~100 Hz; a browser
dashboard or a Python logger turns the stream into live tilt curves,
labelled recordings and left/right recovery metrics.

中文操作说明见 [README.zh-CN.md](README.zh-CN.md)。

| File | What it does |
|---|---|
| `imu_stream.ino` | ESP32-S3 firmware: 3× MPU6050 → CSV over serial at 100 Hz |
| `index.html` | Browser dashboard (GitHub Pages): live curves, labels, metrics, recording |
| `imu_logger.py` | Python serial logger: labelled CSV + meta + events files |
| `visualize.py` | Python live view / replay, and per-trial PNG report |
| `load_trial.py` | Load a recorded trial into pandas (gyro bias removed) |

## Hardware

| IMU | Bus | Pins | Address | Mount |
|---|---|---|---|---|
| IMU1 | A (`Wire`) | SDA 8, SCL 9 | 0x68 (AD0 → GND) | Balance board, centre |
| IMU2 | A (`Wire`) | SDA 8, SCL 9 | 0x69 (AD0 → 3V3) | Top plate |
| IMU3 | B (`Wire1`) | SDA 4, SCL 5 | 0x68 (AD0 → GND) | Shank of the tested leg |

Serial: **921600 baud on the ESP32-S3 UART port**. Ranges ±4 g, ±500 °/s,
DLPF ≈ 44 Hz. Output line format:

```
t_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,ax3,ay3,az3,gx3,gy3,gz3
```

Lines starting with `#` are status messages (e.g. `# IMU1 (0x68 on bus A): OK`).

Mount the board IMU with its **x axis pointing left–right** across the
athlete: the dashboard and report infer perturbation direction from the sign
of board roll.

---

## Demo Day quick start

### Before the session (10 min)

1. Flash `imu_stream.ino` (Arduino IDE, board: ESP32S3 Dev Module).
2. Plug the ESP32-S3 **UART** USB port into the laptop.
3. Open the dashboard in **desktop Chrome or Edge**:
   `https://naruncheng.github.io/ankle-rehab/`
   (offline fallback: open `index.html` from a local copy in Chrome).
4. **Connect ESP32-S3** → pick the port → baud 921600.
5. Check the device log shows `IMU1 … OK`, `IMU2 … OK`, `IMU3 … OK` and
   the header shows ~100 Hz with `dropped 0`.
6. Everything still → **Calibrate gyro (2 s)**.
7. Athlete standing level on the board → **Zero tilt**.
8. Safety: hard stops in place, handrail fixed, spotter beside the platform.

### Running a demo

1. Athlete stands on the board, spotter ready. Press **1** (balanced).
2. At the moment of each perturbation press **2**; when the board starts
   recovering press **3**; once stable press **1** again.
3. After each perturbation the right panel adds a row (direction, peak tilt,
   time to stabilisation) and updates the left/right bars.
4. To keep the data: **Start recording** before the set, **Stop & download**
   after. You get `…_web.csv` and `…_web_meta.json`.
5. **Clear** resets the metrics between visitors.

### Afterwards: report for one trial

```bash
pip install -r requirements.txt
python visualize.py trial path/to/20260927_120000_S01_web.csv --board imu1
```

Writes `{stem}_report.png` and `{stem}_metrics.csv` next to the CSV.

### If something goes wrong

| Symptom | Fix |
|---|---|
| "No Web Serial support" | Use desktop Chrome or Edge, not Safari/Firefox/mobile |
| Port not listed / fails to open | Close `imu_logger.py`, Arduino Serial Monitor and other tabs using the port; replug USB |
| Garbled or no data | Baud must be 921600; use the UART port, not native USB |
| `IMUn … NOT FOUND` | Check SDA/SCL wiring and AD0 level for that IMU, then press the board's reset |
| One plot says "no data" | That IMU is returning NaN — loose connector; reseat and reset |
| Rate well below 100 Hz, `dropped` rising | Shorten I²C wires; keep the dashboard tab in the foreground |
| Curves drift slowly | Keep still and press **Calibrate gyro** again |
| Board not at 0° when level | Press **Zero tilt** with the athlete standing level |
| Left/right swapped | Board IMU is rotated 180°; turn it round or swap the names mentally |
| Live hardware fails entirely | Replay a recorded trial: `python visualize.py live --replay file.csv` |

---

## Browser dashboard (`index.html`)

Runs entirely in the browser via the Web Serial API; GitHub Pages provides the
HTTPS it needs. To publish: repository **Settings → Pages → Deploy from a
branch → `main` / root**.

- Live roll/pitch for all three IMUs, last 10 s, label phases shaded, dashed
  line at each perturbation onset.
- Labels by keyboard `0`–`3` or buttons (`0` none, `1` balanced,
  `2` perturbation, `3` recovery).
- Recovery panel: per-perturbation direction, peak tilt, time to
  stabilisation (TTS), left/right mean ± SD and asymmetry.
- Setup: which IMU is on the board, IMU names, subject ID, plot range.
- Recording downloads a CSV in the same format as `imu_logger.py` plus a meta
  JSON, so `visualize.py` and `load_trial.py` read it directly.

## Python logger (`imu_logger.py`)

```bash
python imu_logger.py --subject S01 --condition demo \
    --segments "imu1=board,imu2=platform,imu3=shank"
python imu_logger.py --port COM9 --subject S01 --condition demo --trial 2
```

On startup it prints the firmware status lines, calibrates gyro bias for
`--calib-seconds` (default 2 s, keep still), then records to
`./data/{date}_{subject}_{trial:02d}_{condition}.csv`.

While running: `0`–`9` set the label, `SPACE` pauses/resumes, `q` stops and
saves. A warning appears if an IMU has been NaN for over 1 s.

Output per trial in `./data/`:

- `{stem}.csv` — `pc_time, t_ms, label` + 18 IMU columns
  (`imu1_ax … imu3_gyro_z`); failed readings are empty cells.
- `{stem}_meta.json` — subject, condition, trial, times, port, baud, ranges,
  segment map, gyro bias, firmware status, sample counts.
- `{stem}_events.csv` — every label change.

## Visualisation (`visualize.py`)

```bash
# live from the board (close the logger / dashboard first)
python visualize.py live --port COM9 --segments "imu1=board,imu2=platform,imu3=shank"

# replay a recording at real speed
python visualize.py live --replay data/20260925_S01_01_demo.csv

# per-trial report
python visualize.py trial data/20260925_S01_01_demo.csv --board imu1
```

## Loading a trial in pandas

```python
from load_trial import load_trial
df, meta = load_trial("data/20260925_S01_01_demo.csv")
```

Returns the CSV with the recorded gyro bias subtracted from the
`imu{n}_gyro_{x,y,z}` columns, plus the metadata dict.

## Metrics

Computed on the board IMU for each switch into label `2`:

- **Tilt** — complementary filter per IMU (α = 0.98): roll = atan2(ay, az),
  pitch = atan2(−ax, √(ay² + az²)), fused with gyro x/y.
- **Baseline** — mean board roll over 0.5 s before onset.
- **Peak tilt** — largest roll deviation from baseline within 3 s.
- **Direction** — `left` if that deviation is negative, `right` if positive.
- **Time to stabilisation (TTS)** — time from onset until |roll − baseline|
  < 2° and |roll rate| < 10 °/s (50 ms smoothed) hold for 0.5 s; blank or
  "> 3" if not reached within 3 s.
- **Asymmetry** — (TTS_left − TTS_right) / mean × 100 %; positive means the
  left side recovers more slowly.

## Setup

```bash
pip install -r requirements.txt
```
