// AD-1 flight controller - board bring-up test
// ESP32-S3-WROOM + BMI088 (accel + gyro) + BMP388 (baro)
// Step 1: check chip IDs (proves solder joints + bus wiring)
// Step 2: stream raw-converted data (accel m/s^2, gyro dps, temp C, pressure hPa)
// No external library needed.

#include <SPI.h>
#include <Wire.h>

// ===================== CONFIG (match your schematic) =====================
#define USE_SPI 1            // 1 = SPI, 0 = I2C

// SPI pins (ESP32-S3 GPIO numbers) -> replace with your PCB routing
#define PIN_SCK      12
#define PIN_MISO     13
#define PIN_MOSI     11
#define CS_ACC       10      // BMI088 CSB1 (accelerometer)
#define CS_GYR        9      // BMI088 CSB2 (gyroscope)
#define CS_BARO       8      // BMP388 CSB

// I2C pins / addresses (only if USE_SPI = 0)
#define PIN_SDA       4
#define PIN_SCL       5
#define ADDR_ACC   0x18      // 0x19 if SDO1 = VDDIO
#define ADDR_GYR   0x68      // 0x69 if SDO2 = VDDIO
#define ADDR_BARO  0x76      // 0x77 if SDO = VDDIO
// ========================================================================

enum Dev { ACC, GYR, BARO };

#if USE_SPI
SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);
int csPin(Dev d) { return d == ACC ? CS_ACC : (d == GYR ? CS_GYR : CS_BARO); }
#else
uint8_t i2cAddr(Dev d) { return d == ACC ? ADDR_ACC : (d == GYR ? ADDR_GYR : ADDR_BARO); }
#endif

void readRegs(Dev d, uint8_t reg, uint8_t *buf, uint8_t len) {
#if USE_SPI
  SPI.beginTransaction(spiCfg);
  digitalWrite(csPin(d), LOW);
  SPI.transfer(reg | 0x80);                  // read bit
  if (d == ACC || d == BARO) SPI.transfer(0); // accel & baro send a dummy byte first
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0);
  digitalWrite(csPin(d), HIGH);
  SPI.endTransaction();
#else
  Wire.beginTransmission(i2cAddr(d));
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddr(d), len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
#endif
}

uint8_t readReg(Dev d, uint8_t reg) { uint8_t v = 0; readRegs(d, reg, &v, 1); return v; }

void writeReg(Dev d, uint8_t reg, uint8_t val) {
#if USE_SPI
  SPI.beginTransaction(spiCfg);
  digitalWrite(csPin(d), LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(csPin(d), HIGH);
  SPI.endTransaction();
#else
  Wire.beginTransmission(i2cAddr(d));
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
#endif
  delay(2);
}

// ---------------- BMP388 calibration (datasheet, float version) ----------------
double T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;

void loadBaroCalib() {
  uint8_t c[21];
  readRegs(BARO, 0x31, c, 21);
  T1  = (uint16_t)(c[1] << 8 | c[0]) * 256.0;
  T2  = (uint16_t)(c[3] << 8 | c[2]) / ldexp(1.0, 30);
  T3  = (int8_t)c[4] / ldexp(1.0, 48);
  P1  = ((int16_t)(c[6] << 8 | c[5]) - 16384) / ldexp(1.0, 20);
  P2  = ((int16_t)(c[8] << 8 | c[7]) - 16384) / ldexp(1.0, 29);
  P3  = (int8_t)c[9]  / ldexp(1.0, 32);
  P4  = (int8_t)c[10] / ldexp(1.0, 37);
  P5  = (uint16_t)(c[12] << 8 | c[11]) * 8.0;
  P6  = (uint16_t)(c[14] << 8 | c[13]) / 64.0;
  P7  = (int8_t)c[15] / 256.0;
  P8  = (int8_t)c[16] / 32768.0;
  P9  = (int16_t)(c[18] << 8 | c[17]) / ldexp(1.0, 48);
  P10 = (int8_t)c[19] / ldexp(1.0, 48);
  P11 = (int8_t)c[20] / ldexp(1.0, 65);
}

void readBaro(double &tempC, double &pressPa) {
  uint8_t b[6];
  readRegs(BARO, 0x04, b, 6);
  double up = (uint32_t)b[2] << 16 | (uint32_t)b[1] << 8 | b[0];
  double ut = (uint32_t)b[5] << 16 | (uint32_t)b[4] << 8 | b[3];
  double d = ut - T1;
  double t = d * T2 + d * d * T3;
  double o1 = P5 + P6 * t + P7 * t * t + P8 * t * t * t;
  double o2 = up * (P1 + P2 * t + P3 * t * t + P4 * t * t * t);
  double o3 = up * up * (P9 + P10 * t) + up * up * up * P11;
  tempC = t;
  pressPa = o1 + o2 + o3;
}

bool check(const char *name, uint8_t got, uint8_t expected) {
  bool ok = (got == expected);
  Serial.printf("%-14s ID = 0x%02X (expected 0x%02X) -> %s\n", name, got, expected, ok ? "OK" : "FAIL");
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== AD-1 board bring-up test ===");

#if USE_SPI
  pinMode(CS_ACC, OUTPUT);  digitalWrite(CS_ACC, HIGH);
  pinMode(CS_GYR, OUTPUT);  digitalWrite(CS_GYR, HIGH);
  pinMode(CS_BARO, OUTPUT); digitalWrite(CS_BARO, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  readReg(ACC, 0x00);       // dummy read: rising edge on CSB1 switches accel to SPI mode
  delay(1);
#else
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Serial.print("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
#endif

  bool okAcc  = check("BMI088 accel", readReg(ACC, 0x00), 0x1E);
  bool okGyr  = check("BMI088 gyro",  readReg(GYR, 0x00), 0x0F);
  bool okBaro = check("BMP388 baro",  readReg(BARO, 0x00), 0x50);

  // Accel: power on (off by default), range +-6 g
  writeReg(ACC, 0x7D, 0x04);  // ACC_PWR_CTRL: accel on
  writeReg(ACC, 0x7C, 0x00);  // ACC_PWR_CONF: active mode
  delay(50);
  writeReg(ACC, 0x41, 0x01);  // ACC_RANGE: +-6 g
  // Gyro: default range +-2000 dps, already in normal mode

  // Baro: press x8, temp x1, 25 Hz, normal mode
  loadBaroCalib();
  writeReg(BARO, 0x1C, 0x03);
  writeReg(BARO, 0x1D, 0x03);
  writeReg(BARO, 0x1B, 0x33);
  delay(100);

  Serial.println((okAcc && okGyr && okBaro) ? ">>> ALL SENSORS DETECTED" : ">>> SOME SENSORS MISSING - check solder / wiring");
  Serial.println("ax ay az [m/s2] | gx gy gz [dps] | T [C] | P [hPa]");
}

void loop() {
  uint8_t a[6], g[6];
  readRegs(ACC, 0x12, a, 6);
  readRegs(GYR, 0x02, g, 6);

  const float accScale = 6.0f / 32768.0f * 9.80665f;  // +-6 g
  const float gyrScale = 2000.0f / 32768.0f;          // +-2000 dps
  float ax = (int16_t)(a[1] << 8 | a[0]) * accScale;
  float ay = (int16_t)(a[3] << 8 | a[2]) * accScale;
  float az = (int16_t)(a[5] << 8 | a[4]) * accScale;
  float gx = (int16_t)(g[1] << 8 | g[0]) * gyrScale;
  float gy = (int16_t)(g[3] << 8 | g[2]) * gyrScale;
  float gz = (int16_t)(g[5] << 8 | g[4]) * gyrScale;

  double tC, pPa;
  readBaro(tC, pPa);

  Serial.printf("%6.2f %6.2f %6.2f | %7.2f %7.2f %7.2f | %5.2f | %7.2f\n",
                ax, ay, az, gx, gy, gz, tC, pPa / 100.0);
  delay(100);
}
