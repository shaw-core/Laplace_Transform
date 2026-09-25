# Ankle Rehab — 自适应扰动平衡训练平台

平衡平台的感知和可视化代码。ESP32-S3 读取三个 MPU6050 IMU，通过 USB 串口以约 100 Hz 输出数据；浏览器面板或 Python 记录程序会把数据变成实时倾角曲线、带标签的录制文件，以及左右方向的恢复指标。

English version: [README.md](README.md)

| 文件 | 作用 |
|---|---|
| `imu_stream.ino` | ESP32-S3 固件：读取 3 个 MPU6050，以 100 Hz 通过串口输出 CSV |
| `index.html` | 浏览器面板（GitHub Pages）：实时曲线、标签、指标、录制 |
| `imu_logger.py` | Python 串口记录程序：输出带标签的 CSV、meta 和事件文件 |
| `visualize.py` | Python 实时查看或回放，以及单次试验的 PNG 报告 |
| `load_trial.py` | 把录制的试验读入 pandas（已扣除陀螺仪零偏） |

## 硬件

| IMU | 总线 | 引脚 | 地址 | 安装位置 |
|---|---|---|---|---|
| IMU1 | A（`Wire`） | SDA 8，SCL 9 | 0x68（AD0 接 GND） | 平衡板中心 |
| IMU2 | A（`Wire`） | SDA 8，SCL 9 | 0x69（AD0 接 3V3） | 顶板 |
| IMU3 | B（`Wire1`） | SDA 4，SCL 5 | 0x68（AD0 接 GND） | 被测腿的小腿 |

串口：**ESP32-S3 的 UART 口，921600 波特率**。量程 ±4 g、±500 °/s，数字低通约 44 Hz。每行输出格式：

```
t_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,ax3,ay3,az3,gx3,gy3,gz3
```

以 `#` 开头的是状态信息，例如 `# IMU1 (0x68 on bus A): OK`。

平衡板上的 IMU 要让 **x 轴沿运动员的左右方向**安装。面板和报告是根据平衡板 roll 的正负来判断扰动方向的。

---

## Demo Day 现场操作

### 开场前准备（约 10 分钟）

1. 用 Arduino IDE 烧录 `imu_stream.ino`，开发板选 ESP32S3 Dev Module。
2. 用 USB 线把 ESP32-S3 的 **UART 口**接到电脑。
3. 用**电脑上的 Chrome 或 Edge** 打开面板：
   `https://naruncheng.github.io/ankle-rehab/`
   （没网时：直接用 Chrome 打开本地的 `index.html`）
4. 点 **Connect ESP32-S3**，选端口，波特率选 921600。
5. 确认设备日志里 `IMU1 … OK`、`IMU2 … OK`、`IMU3 … OK` 都在，顶部显示约 100 Hz，`dropped 0`。
6. 所有 IMU 保持不动，点 **Calibrate gyro (2 s)**。
7. 运动员站在平衡板上保持水平，点 **Zero tilt**。
8. 安全检查：硬限位装好，扶手固定，保护员站在平台旁边。

### 演示流程

1. 运动员站上平衡板，保护员就位，按 **1**（balanced，平衡）。
2. 扰动发生的那一刻按 **2**；平衡板开始恢复时按 **3**；站稳后再按 **1**。
3. 每次扰动之后，右侧面板会新增一行（方向、峰值倾角、稳定时间），并更新左右对比柱状图。
4. 需要保存数据时：开始前点 **Start recording**，结束后点 **Stop & download**，会下载 `…_web.csv` 和 `…_web_meta.json` 两个文件。
5. 换下一位体验者前，点 **Clear** 清空指标。

### 结束后：生成单次试验报告

```bash
pip install -r requirements.txt
python visualize.py trial path/to/20260927_120000_S01_web.csv --board imu1
```

会在 CSV 旁边生成 `{stem}_report.png` 和 `{stem}_metrics.csv`。

### 常见问题

| 现象 | 处理方法 |
|---|---|
| 提示 "No Web Serial support" | 换成电脑上的 Chrome 或 Edge；Safari、Firefox 和手机都不支持 |
| 找不到端口或打不开 | 关掉 `imu_logger.py`、Arduino 串口监视器和其他占用这个端口的标签页，然后重新插 USB |
| 没有数据或数据乱码 | 波特率必须是 921600；要接 UART 口，不要接原生 USB 口 |
| 显示 `IMUn … NOT FOUND` | 检查这个 IMU 的 SDA/SCL 接线和 AD0 电平，然后按开发板上的复位键 |
| 某个曲线显示 "no data" | 这个 IMU 在返回 NaN，一般是接头松了；重新插好后复位 |
| 采样率明显低于 100 Hz，`dropped` 在增加 | 把 I²C 线缩短；面板标签页要保持在前台 |
| 曲线慢慢漂移 | 保持静止，重新点 **Calibrate gyro** |
| 平衡板放平了但不显示 0° | 让运动员站平后点 **Zero tilt** |
| 左右方向反了 | 平衡板 IMU 装反了 180°，转过来，或者读数时把左右对调 |
| 现场硬件完全用不了 | 回放之前录好的数据：`python visualize.py live --replay file.csv` |

---

## 浏览器面板（`index.html`）

完全在浏览器里运行，通过 Web Serial API 读串口。Web Serial 需要 HTTPS，GitHub Pages 正好提供。发布方法：仓库 **Settings → Pages → Deploy from a branch → `main` / root**。

- 三个 IMU 的 roll/pitch 实时曲线，显示最近 10 秒；背景按标签阶段上色，每次扰动开始处有一条虚线。
- 用键盘 `0`–`3` 或按钮打标签（`0` 无，`1` 平衡，`2` 扰动，`3` 恢复）。
- 恢复面板：每次扰动的方向、峰值倾角、稳定时间（TTS），左右两边的均值 ± 标准差，以及不对称百分比。
- 设置：哪个 IMU 装在平衡板上、IMU 名称、受试者编号、曲线显示范围。
- 录制下载的 CSV 格式和 `imu_logger.py` 相同，另外附带一个 meta JSON，`visualize.py` 和 `load_trial.py` 可以直接读取。

## Python 记录程序（`imu_logger.py`）

```bash
python imu_logger.py --subject S01 --condition demo \
    --segments "imu1=board,imu2=platform,imu3=shank"
python imu_logger.py --port COM9 --subject S01 --condition demo --trial 2
```

启动后会先打印固件状态信息，再用 `--calib-seconds`（默认 2 秒，期间保持静止）校准陀螺仪零偏，然后开始录制到 `./data/{日期}_{受试者}_{试验号:02d}_{条件}.csv`。

运行时：`0`–`9` 设置标签，`空格` 暂停/继续，`q` 停止并保存。某个 IMU 连续 1 秒以上返回 NaN 时会显示警告。

每次试验在 `./data/` 下生成：

- `{stem}.csv`：`pc_time, t_ms, label` 加 18 个 IMU 数据列（`imu1_ax … imu3_gyro_z`），读取失败的格子留空。
- `{stem}_meta.json`：受试者、条件、试验号、时间、端口、波特率、量程、IMU 名称映射、陀螺仪零偏、固件状态、采样数。
- `{stem}_events.csv`：每次标签变化的记录。

## 可视化（`visualize.py`）

```bash
# 直接读开发板实时显示（先关掉 logger 和网页面板）
python visualize.py live --port COM9 --segments "imu1=board,imu2=platform,imu3=shank"

# 按真实速度回放录制的数据
python visualize.py live --replay data/20260925_S01_01_demo.csv

# 单次试验报告
python visualize.py trial data/20260925_S01_01_demo.csv --board imu1
```

## 在 pandas 里读取试验

```python
from load_trial import load_trial
df, meta = load_trial("data/20260925_S01_01_demo.csv")
```

返回的 DataFrame 已经从 `imu{n}_gyro_{x,y,z}` 列中扣除了录制时的陀螺仪零偏，同时返回 meta 字典。

## 指标定义

每次标签切换到 `2` 时，用平衡板 IMU 计算：

- **倾角**：每个 IMU 用互补滤波（α = 0.98）：roll = atan2(ay, az)，pitch = atan2(−ax, √(ay² + az²))，再和陀螺仪 x/y 融合。
- **基线**：扰动开始前 0.5 秒内平衡板 roll 的平均值。
- **峰值倾角**：扰动后 3 秒内 roll 偏离基线的最大值。
- **方向**：这个偏离为负记为 `left`，为正记为 `right`。
- **稳定时间（TTS）**：从扰动开始，到 |roll − 基线| < 2° 且 |roll 角速度| < 10 °/s（50 ms 平滑）连续保持 0.5 秒为止；3 秒内没达到就留空或显示 "> 3"。
- **不对称**：(TTS_左 − TTS_右) / 均值 × 100%；正值表示左侧恢复更慢。

## 安装

```bash
pip install -r requirements.txt
```
