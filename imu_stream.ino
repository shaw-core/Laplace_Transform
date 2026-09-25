// 3x MPU6050 streaming over serial, ESP32-S3
// Bus A (Wire):  SDA=8, SCL=9  -> IMU1 (0x68, AD0=GND), IMU2 (0x69, AD0=3V3)
// Bus B (Wire1): SDA=4, SCL=5  -> IMU3 (0x68, AD0=GND)
//
// Output (CSV, one line per sample, units: g and deg/s):
// t_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,ax3,ay3,az3,gx3,gy3,gz3
//
// Open serial monitor at 921600 baud (use the UART port).
// No external libraries needed.

#include <Arduino.h>
#include <Wire.h>

// ---------- Config ----------
constexpr uint32_t BAUD        = 921600;
constexpr uint32_t SAMPLE_HZ   = 100;
constexpr uint32_t I2C_FREQ    = 400000;

// Full-scale ranges: ACCEL_CFG 0=±2g 1=±4g 2=±8g 3=±16g | GYRO_CFG 0=±250 1=±500 2=±1000 3=±2000 dps
constexpr uint8_t  ACCEL_RANGE = 1;
constexpr uint8_t  GYRO_RANGE  = 1;

constexpr int SDA_A = 8, SCL_A = 9;
constexpr int SDA_B = 4, SCL_B = 5;

// ---------- MPU6050 registers ----------
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

struct Imu {
  TwoWire *bus;
  uint8_t  addr;
  bool     ok;
  float    v[6];  // ax ay az gx gy gz
};

Imu imus[3] = {
  { &Wire,  0x68, false, {0} },  // IMU1: bus A, AD0=GND
  { &Wire,  0x69, false, {0} },  // IMU2: bus A, AD0=3V3
  { &Wire1, 0x68, false, {0} },  // IMU3: bus B, AD0=GND
};

const float ACCEL_LSB_PER_G   = 16384.0f / (1 << ACCEL_RANGE);
const float GYRO_LSB_PER_DPS  = 131.0f   / (1 << GYRO_RANGE);

bool writeReg(Imu &m, uint8_t reg, uint8_t val) {
  m.bus->beginTransmission(m.addr);
  m.bus->write(reg);
  m.bus->write(val);
  return m.bus->endTransmission() == 0;
}

bool readRegs(Imu &m, uint8_t reg, uint8_t *buf, size_t len) {
  m.bus->beginTransmission(m.addr);
  m.bus->write(reg);
  if (m.bus->endTransmission(false) != 0) return false;
  if (m.bus->requestFrom((int)m.addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = m.bus->read();
  return true;
}

bool initImu(Imu &m) {
  uint8_t who = 0;
  if (!readRegs(m, REG_WHO_AM_I, &who, 1)) return false;
  // Genuine MPU6050 returns 0x68 (some clones return 0x70/0x72), so don't be strict.
  if (!writeReg(m, REG_PWR_MGMT_1, 0x01)) return false;   // wake, PLL with X gyro
  delay(10);
  writeReg(m, REG_CONFIG, 0x03);                          // DLPF ~44 Hz
  writeReg(m, REG_SMPLRT_DIV, 0x00);                      // 1 kHz internal rate, we poll
  writeReg(m, REG_GYRO_CONFIG,  GYRO_RANGE  << 3);
  writeReg(m, REG_ACCEL_CONFIG, ACCEL_RANGE << 3);
  return true;
}

bool readImu(Imu &m) {
  uint8_t b[14];
  if (!readRegs(m, REG_ACCEL_XOUT_H, b, 14)) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  m.v[0] = s16(0)  / ACCEL_LSB_PER_G;
  m.v[1] = s16(2)  / ACCEL_LSB_PER_G;
  m.v[2] = s16(4)  / ACCEL_LSB_PER_G;
  // bytes 6-7 are temperature, skipped
  m.v[3] = s16(8)  / GYRO_LSB_PER_DPS;
  m.v[4] = s16(10) / GYRO_LSB_PER_DPS;
  m.v[5] = s16(12) / GYRO_LSB_PER_DPS;
  return true;
}

void setup() {
  Serial.begin(BAUD);
  delay(500);

  Wire.begin(SDA_A, SCL_A, I2C_FREQ);
  Wire1.begin(SDA_B, SCL_B, I2C_FREQ);
  Wire.setTimeOut(20);
  Wire1.setTimeOut(20);

  for (int i = 0; i < 3; i++) {
    imus[i].ok = initImu(imus[i]);
    Serial.printf("# IMU%d (0x%02X on bus %c): %s\n", i + 1, imus[i].addr,
                  imus[i].bus == &Wire ? 'A' : 'B', imus[i].ok ? "OK" : "NOT FOUND");
  }
  Serial.println("# t_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,ax3,ay3,az3,gx3,gy3,gz3");
}

void loop() {
  static uint32_t next = micros();
  const uint32_t period = 1000000UL / SAMPLE_HZ;

  if ((int32_t)(micros() - next) < 0) return;
  next += period;

  for (int i = 0; i < 3; i++) {
    if (imus[i].ok && !readImu(imus[i])) {
      for (int k = 0; k < 6; k++) imus[i].v[k] = NAN;  // read failed this cycle
    }
  }

  Serial.print(millis());
  for (int i = 0; i < 3; i++) {
    for (int k = 0; k < 6; k++) {
      Serial.print(',');
      if (imus[i].ok && !isnan(imus[i].v[k])) Serial.print(imus[i].v[k], 3);
      else Serial.print("nan");
    }
  }
  Serial.println();
}
