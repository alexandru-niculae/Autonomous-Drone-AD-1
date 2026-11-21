// AD-1 - EDF vibration bench test
// Board : custom FC, ESP32-S3-WROOM-1-N16 + BMI088 (SPI)
// Core  : Arduino-ESP32 >= 3.0 (ledcAttach API), "USB CDC On Boot" = Enabled
//
// What it does
//   1. Arms the ESC (1000 us for 3 s)
//   2. For each throttle step: ramp up, settle, record raw IMU at 2 kHz into RAM,
//      cut throttle, dump the recording as CSV over USB
//   3. Host side: vibration_test.py captures the CSV and computes the spectra
//
// Safety: motor only starts after the host sends "GO". Send "s" at any time to stop.

#include <SPI.h>

// ---------------- Pins (from KiCad "carte de vol") ----------------
#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_MOSI  11
#define CS_ACC    10   // BMI088 accel (CSB1)
#define CS_GYR    21   // BMI088 gyro  (CSB2)
#define CS_BARO   14   // BMP390, kept high (unused here)
#define PIN_ESC    6   // j6

// ---------------- Test parameters ----------------
const int   THROTTLE_STEPS[] = {0, 25, 50, 75, 100};   // percent
const int   N_STEPS     = sizeof(THROTTLE_STEPS) / sizeof(THROTTLE_STEPS[0]);
const float RECORD_S    = 3.0f;    // recording length per step
const float SETTLE_S    = 2.0f;    // wait after ramp before recording
const float RAMP_S      = 1.0f;    // throttle ramp duration
const uint32_t PERIOD_US = 500;    // 2 kHz sampling
const int   ESC_MIN_US  = 1000;
const int   ESC_MAX_US  = 2000;

// Metadata written in the CSV header (used by the analysis script)
const float MOTOR_KV = 1900.0f;
const float VBAT_V   = 22.2f;      // 6S nominal, edit with measured voltage

// ---------------- Sensor scaling ----------------
const float GYR_SCALE = 2000.0f / 32768.0f;  // +-2000 dps
const float ACC_SCALE = 24.0f / 32768.0f;    // +-24 g

// ---------------- Recording buffer (RAM) ----------------
struct Sample { uint32_t t_us; int16_t g[3]; int16_t a[3]; };
const int MAX_SAMPLES = (int)(RECORD_S * 1e6 / PERIOD_US);
static Sample buf[MAX_SAMPLES];   // 6000 x 16 B = 96 kB

SPISettings spiCfg(8000000, MSBFIRST, SPI_MODE0);
bool aborted = false;

// ---------------- Low-level SPI ----------------
void readRegs(int cs, bool dummy, uint8_t reg, uint8_t *b, uint8_t n) {
  SPI.beginTransaction(spiCfg);
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  if (dummy) SPI.transfer(0);          // accel sends a dummy byte first
  for (uint8_t i = 0; i < n; i++) b[i] = SPI.transfer(0);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}
uint8_t readReg(int cs, bool dummy, uint8_t reg) { uint8_t v; readRegs(cs, dummy, reg, &v, 1); return v; }
void writeReg(int cs, uint8_t reg, uint8_t v) {
  SPI.beginTransaction(spiCfg);
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(v);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  delay(2);
}

// ---------------- ESC ----------------
void escWriteUs(int us) {
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  ledcWrite(PIN_ESC, (uint32_t)((uint64_t)us * 65535 / 20000));  // 50 Hz, 16 bit
}
int pctToUs(float pct) { return ESC_MIN_US + (int)(pct / 100.0f * (ESC_MAX_US - ESC_MIN_US)); }

void checkAbort() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') { aborted = true; escWriteUs(ESC_MIN_US); }
  }
}

// Wait while keeping an eye on the abort command
void safeDelay(float s) {
  uint32_t t0 = millis();
  while (millis() - t0 < s * 1000 && !aborted) { checkAbort(); delay(5); }
}

void ramp(float fromPct, float toPct) {
  const int n = 50;
  for (int i = 1; i <= n && !aborted; i++) {
    escWriteUs(pctToUs(fromPct + (toPct - fromPct) * i / n));
    safeDelay(RAMP_S / n);
  }
}

// ---------------- Sensor init ----------------
bool initImu() {
  readReg(CS_ACC, true, 0x00);  // rising edge on CSB1 -> accel in SPI mode
  delay(1);
  uint8_t idA = readReg(CS_ACC, true, 0x00);
  uint8_t idG = readReg(CS_GYR, false, 0x00);
  Serial.printf("# accel_id=0x%02X gyro_id=0x%02X\n", idA, idG);
  if (idA != 0x1E || idG != 0x0F) return false;

  writeReg(CS_ACC, 0x7D, 0x04);  // ACC_PWR_CTRL: on
  writeReg(CS_ACC, 0x7C, 0x00);  // ACC_PWR_CONF: active
  delay(50);
  writeReg(CS_ACC, 0x40, 0xAC);  // ACC_CONF: normal filter, ODR 1600 Hz (~234 Hz BW)
  writeReg(CS_ACC, 0x41, 0x03);  // ACC_RANGE: +-24 g (see if vibrations saturate)

  writeReg(CS_GYR, 0x0F, 0x00);  // GYRO_RANGE: +-2000 dps
  writeReg(CS_GYR, 0x10, 0x00);  // GYRO_BANDWIDTH: ODR 2000 Hz, 532 Hz BW (widest)
  delay(50);
  return true;
}

// ---------------- Recording ----------------
int record() {
  int n = 0;
  uint32_t t0 = micros(), next = t0;
  while (n < MAX_SAMPLES && !aborted) {
    while ((int32_t)(micros() - next) < 0) {}
    next += PERIOD_US;
    uint8_t g[6], a[6];
    readRegs(CS_GYR, false, 0x02, g, 6);
    readRegs(CS_ACC, true, 0x12, a, 6);
    Sample &s = buf[n++];
    s.t_us = micros() - t0;
    for (int i = 0; i < 3; i++) {
      s.g[i] = (int16_t)(g[2 * i + 1] << 8 | g[2 * i]);
      s.a[i] = (int16_t)(a[2 * i + 1] << 8 | a[2 * i]);
    }
    if ((n & 63) == 0) checkAbort();
  }
  return n;
}

// Dump one step as CSV. *_new = 1 when the sensor delivered a fresh sample
// (accel runs at 1600 Hz, so some 2 kHz reads repeat the previous value).
void dump(int step, int pct, int n) {
  for (int i = 0; i < n; i++) {
    const Sample &s = buf[i];
    bool gNew = (i == 0) || memcmp(s.g, buf[i - 1].g, 6) != 0;
    bool aNew = (i == 0) || memcmp(s.a, buf[i - 1].a, 6) != 0;
    Serial.printf("%d,%d,%.6f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d\n",
                  step, pct, s.t_us * 1e-6,
                  s.g[0] * GYR_SCALE, s.g[1] * GYR_SCALE, s.g[2] * GYR_SCALE,
                  s.a[0] * ACC_SCALE, s.a[1] * ACC_SCALE, s.a[2] * ACC_SCALE,
                  gNew, aNew);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(CS_ACC, OUTPUT);  digitalWrite(CS_ACC, HIGH);
  pinMode(CS_GYR, OUTPUT);  digitalWrite(CS_GYR, HIGH);
  pinMode(CS_BARO, OUTPUT); digitalWrite(CS_BARO, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  ledcAttach(PIN_ESC, 50, 16);
  escWriteUs(ESC_MIN_US);        // ESC sees min throttle from power-up

  delay(1500);
  bool ok = initImu();

  // Wait for "GO" from the host, re-announce every second
  String cmd;
  uint32_t last = 0;
  while (true) {
    if (millis() - last > 1000) {
      last = millis();
      Serial.println(ok ? "#READY send GO to start" : "#ERROR IMU not detected");
    }
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') { if (cmd == "GO" && ok) goto start; cmd = ""; }
      else cmd += c;
    }
  }

start:
  Serial.println("#START");
  Serial.printf("# fs_loop_hz=%d\n# acc_odr_hz=1600\n# gyr_odr_hz=2000\n", (int)(1e6 / PERIOD_US));
  Serial.printf("# acc_range_g=24\n# gyr_range_dps=2000\n# motor_kv=%.0f\n# vbat_v=%.2f\n", MOTOR_KV, VBAT_V);
  Serial.println("step,throttle_pct,t_s,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,gyr_new,acc_new");

  Serial.println("# arming ESC");
  safeDelay(3.0);

  for (int k = 0; k < N_STEPS && !aborted; k++) {
    int pct = THROTTLE_STEPS[k];
    Serial.printf("# step %d: throttle %d %%\n", k, pct);
    ramp(0, pct);
    safeDelay(SETTLE_S);
    int n = record();
    escWriteUs(ESC_MIN_US);       // motor off while dumping
    if (aborted) break;
    dump(k, pct, n);
    safeDelay(2.0);               // let the motor spin down
  }

  escWriteUs(ESC_MIN_US);
  Serial.println(aborted ? "#ABORTED" : "#DONE");
}

void loop() {
  escWriteUs(ESC_MIN_US);
  delay(100);
}
