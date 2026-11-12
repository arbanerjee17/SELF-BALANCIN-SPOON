#include <Wire.h>
#include <ESP32Servo.h>

// ---------- Pins ----------
const int SDA_PIN   = 6;
const int SCL_PIN   = 7;
const int ROLL_PIN  = 2;    // lower servo (shaft along handle) -> left/right tilt
const int PITCH_PIN = 18;   // upper servo (shaft sideways)     -> up/down tilt
const int LED_PIN   = 10;

// ---------- Modes ----------
const bool TEST_CENTER   = false;  // true = hold servos at centre and stop (mechanical check)
const bool ZERO_AT_START = true;   // true = the level pose at power-up counts as "level"
const bool AUTO_AXES     = false;  // false = use the manual axes below (recommended)

// Sensor axes in the handle frame (0 = X, 1 = Y, 2 = Z).  Sign: +1 or -1
const int MANUAL_FWD_AXIS = 1;     // sensor Y points along the handle toward the spoon
const int MANUAL_FWD_SIGN = 1;
const int MANUAL_UP_AXIS  = 2;     // sensor Z points up out of the board
const int MANUAL_UP_SIGN  = 1;

// ---------- Tuning ----------
const int   ROLL_DIR   = 1;      // flip to -1 if the lower servo goes the wrong way
const int   PITCH_DIR  = 1;      // flip to -1 if the upper servo goes the wrong way
const float ROLL_TRIM  = 0;      // degrees, fine-tune "level" position
const float PITCH_TRIM = 0;
const float ALPHA      = 0.98;   // gyro trust (0.95-0.99). Higher = smoother, slower drift correction
const float ACC_MIN    = 0.85;   // only trust accelerometer when total accel is within this band (g)
const float ACC_MAX    = 1.15;
const float MAX_TILT   = 80;     // max servo travel from centre, degrees
const float DEADBAND   = 0.2;    // ignore servo changes smaller than this (degrees)
const float SMOOTHING  = 0.8;    // 0-1, output filter. Lower = smoother but laggier
const bool  DEBUG      = true;   // print angles to Serial

// ---------- Servo pulse range ----------
const int PULSE_MIN = 500;       // microseconds at 0 deg
const int PULSE_MAX = 2400;      // microseconds at 180 deg

// ---------- MPU6050 ----------
const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SCALE = 16384.0;  // +-2g
const float GYRO_SCALE  = 65.5;     // +-500 deg/s

Servo rollServo, pitchServo;

// Handle frame expressed in sensor axes: forward (Fv), left (Lv), up (Uv)
float Fv[3] = {1, 0, 0};
float Lv[3] = {0, 1, 0};
float Uv[3] = {0, 0, 1};

float gBias[3] = {0, 0, 0};         // raw gyro bias (sensor axes)
float rollOffset = 0, pitchOffset = 0;
float vx = 0, vy = 0, vz = 1;       // gravity direction in the handle frame
float rollAngle = 0, pitchAngle = 0;
float rollOut = 90, pitchOut = 90;
unsigned long lastMicros = 0;
unsigned long lastPrint = 0;

const char *AXIS_NAME[3] = {"X", "Y", "Z"};

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Raw reading in SENSOR axes
bool readRaw(float *a, float *g) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;

  int16_t r[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  a[0] = r[0] / ACCEL_SCALE;
  a[1] = r[1] / ACCEL_SCALE;
  a[2] = r[2] / ACCEL_SCALE;
  // r[3] is temperature, skipped
  g[0] = r[4] / GYRO_SCALE;
  g[1] = r[5] / GYRO_SCALE;
  g[2] = r[6] / GYRO_SCALE;
  return true;
}

// Convert a vector from sensor axes to handle axes (x = forward, y = left, z = up)
void toHandle(const float *s, float &x, float &y, float &z) {
  x = s[0] * Fv[0] + s[1] * Fv[1] + s[2] * Fv[2];
  y = s[0] * Lv[0] + s[1] * Lv[1] + s[2] * Lv[2];
  z = s[0] * Uv[0] + s[1] * Uv[1] + s[2] * Uv[2];
}

// Reading in HANDLE axes, gyro bias removed
bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  float a[3], g[3];
  if (!readRaw(a, g)) return false;
  for (int i = 0; i < 3; i++) g[i] -= gBias[i];
  toHandle(a, ax, ay, az);
  toHandle(g, gx, gy, gz);
  return true;
}

// Build the handle frame from the forward axis and the up axis (left = up x forward)
void buildFrame(int fa, int fs, int ua, int us) {
  for (int i = 0; i < 3; i++) { Fv[i] = 0; Uv[i] = 0; }
  Fv[fa] = fs;
  Uv[ua] = us;
  Lv[0] = Uv[1] * Fv[2] - Uv[2] * Fv[1];
  Lv[1] = Uv[2] * Fv[0] - Uv[0] * Fv[2];
  Lv[2] = Uv[0] * Fv[1] - Uv[1] * Fv[0];
}

void writeServoAngle(Servo &s, float angle) {
  angle = constrain(angle, 0, 180);
  int us = PULSE_MIN + (int)(angle / 180.0 * (PULSE_MAX - PULSE_MIN));
  s.writeMicroseconds(us);
}

void errorBlink() {
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void slowBlink(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    digitalWrite(LED_PIN, (millis() / 300) % 2);
    delay(10);
  }
  digitalWrite(LED_PIN, LOW);
}

// Wait until the handle is tilted and held still; returns the averaged accel
bool captureTilt(const float *g0, float *g1) {
  unsigned long start = millis();
  int stable = 0;
  float prev[3] = {0, 0, 0};
  float sum[3]  = {0, 0, 0};
  while (millis() - start < 20000) {
    float a[3], g[3];
    if (!readRaw(a, g)) { delay(10); continue; }

    float d = max(fabs(a[0] - g0[0]), max(fabs(a[1] - g0[1]), fabs(a[2] - g0[2])));
    float jitter = fabs(a[0] - prev[0]) + fabs(a[1] - prev[1]) + fabs(a[2] - prev[2]);
    for (int i = 0; i < 3; i++) prev[i] = a[i];

    if (d > 0.35 && jitter < 0.08) {
      stable++;
      for (int i = 0; i < 3; i++) sum[i] += a[i];
    } else {
      stable = 0;
      for (int i = 0; i < 3; i++) sum[i] = 0;
    }
    if (stable >= 40) {
      for (int i = 0; i < 3; i++) g1[i] = sum[i] / stable;
      return true;
    }
    digitalWrite(LED_PIN, (millis() / 80) % 2);  // fast blink
    delay(20);
  }
  return false;
}

bool detectAxes() {
  float a[3], g[3], g0[3] = {0, 0, 0}, g1[3] = {0, 0, 0};

  Serial.println("STEP 1/2: hold the handle LEVEL (spoon end horizontal, board facing up).");
  slowBlink(3000);

  int n = 0;
  for (int i = 0; i < 100; i++) {
    if (readRaw(a, g)) {
      for (int k = 0; k < 3; k++) g0[k] += a[k];
      n++;
    }
    delay(10);
  }
  if (n < 50) return false;
  for (int k = 0; k < 3; k++) g0[k] /= n;

  int ua = 0;
  for (int i = 1; i < 3; i++) if (fabs(g0[i]) > fabs(g0[ua])) ua = i;
  int us = (g0[ua] > 0) ? 1 : -1;
  if (fabs(g0[ua]) < 0.8) Serial.println("Warning: handle was not level enough in step 1.");

  Serial.println("STEP 2/2: now tilt the SPOON END DOWN about 45 degrees and HOLD still.");
  if (!captureTilt(g0, g1)) return false;

  int fa = -1;
  float best = 0;
  for (int i = 0; i < 3; i++) {
    if (i == ua) continue;
    float d = fabs(g1[i] - g0[i]);
    if (d > best) { best = d; fa = i; }
  }
  if (fa < 0 || best < 0.2) return false;
  int fs = ((g1[fa] - g0[fa]) < 0) ? 1 : -1;

  buildFrame(fa, fs, ua, us);
  digitalWrite(LED_PIN, LOW);

  Serial.print("Detected: forward = ");
  Serial.print(fs > 0 ? "+" : "-"); Serial.print(AXIS_NAME[fa]);
  Serial.print("   up = ");
  Serial.print(us > 0 ? "+" : "-"); Serial.println(AXIS_NAME[ua]);
  return true;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro - hold the handle still and level...");
  const int N = 500;
  float s[3] = {0, 0, 0};
  float a[3], g[3];
  for (int i = 0; i < N; i++) {
    if (readRaw(a, g)) {
      for (int k = 0; k < 3; k++) s[k] += g[k];
    }
    digitalWrite(LED_PIN, (i / 25) % 2);  // blink while calibrating
    delay(3);
  }
  for (int k = 0; k < 3; k++) gBias[k] = s[k] / N;
  digitalWrite(LED_PIN, LOW);
  Serial.print("Gyro bias X: "); Serial.print(gBias[0]);
  Serial.print("  Y: "); Serial.print(gBias[1]);
  Serial.print("  Z: "); Serial.println(gBias[2]);
}

void calibrateLevel() {
  float sr = 0, sp = 0, sx = 0, sy = 0, sz = 0;
  int n = 0;
  float ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 100; i++) {
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sr += atan2(ay, az) * RAD_TO_DEG;
      sp += atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
      sx += ax; sy += ay; sz += az;
      n++;
    }
    delay(5);
  }
  if (n > 0) {
    rollOffset  = ZERO_AT_START ? sr / n : 0;
    pitchOffset = ZERO_AT_START ? sp / n : 0;
    float m = sqrt(sx * sx + sy * sy + sz * sz);
    if (m > 0.1) { vx = sx / m; vy = sy / m; vz = sz / m; }
  }
  Serial.print("Level offset Roll: "); Serial.print(rollOffset, 1);
  Serial.print("  Pitch: "); Serial.println(pitchOffset, 1);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Servos: centre first so the spoon starts level
  rollServo.setPeriodHertz(50);
  pitchServo.setPeriodHertz(50);
  rollServo.attach(ROLL_PIN, PULSE_MIN, PULSE_MAX);
  pitchServo.attach(PITCH_PIN, PULSE_MIN, PULSE_MAX);
  writeServoAngle(rollServo, 90);
  writeServoAngle(pitchServo, 90);

  // Mechanical check mode: hold centre (trims applied) and stop
  if (TEST_CENTER) {
    writeServoAngle(rollServo,  90 - ROLL_DIR  * ROLL_TRIM);
    writeServoAngle(pitchServo, 90 - PITCH_DIR * PITCH_TRIM);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("TEST_CENTER: servos held at centre. Bowl should be level and pointing forward.");
    while (true) delay(1000);
  }

  // MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 NOT FOUND - check SDA/SCL/power/AD0");
    errorBlink();
  }

  writeReg(0x6B, 0x00);  // wake up
  delay(100);
  writeReg(0x1A, 0x03);  // low-pass filter ~44 Hz (tremor is 4-12 Hz)
  writeReg(0x1B, 0x08);  // gyro +-500 deg/s
  writeReg(0x1C, 0x00);  // accel +-2g
  delay(100);

  delay(1500);           // time to pick up the handle

  // Work out how the sensor is mounted
  if (AUTO_AXES) {
    if (!detectAxes()) {
      Serial.println("Axis detection failed - using the manual axes instead.");
      buildFrame(MANUAL_FWD_AXIS, MANUAL_FWD_SIGN, MANUAL_UP_AXIS, MANUAL_UP_SIGN);
    }
  } else {
    buildFrame(MANUAL_FWD_AXIS, MANUAL_FWD_SIGN, MANUAL_UP_AXIS, MANUAL_UP_SIGN);
    Serial.print("Manual axes: forward = ");
    Serial.print(MANUAL_FWD_SIGN > 0 ? "+" : "-"); Serial.print(AXIS_NAME[MANUAL_FWD_AXIS]);
    Serial.print("   up = ");
    Serial.print(MANUAL_UP_SIGN > 0 ? "+" : "-"); Serial.println(AXIS_NAME[MANUAL_UP_AXIS]);
  }

  Serial.println("Hold the handle LEVEL and keep it still...");
  slowBlink(3000);
  calibrateGyro();
  calibrateLevel();

  lastMicros = micros();
  digitalWrite(LED_PIN, HIGH);  // LED on = stabilizing active
  Serial.println("Stabilizing...");
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  if (!readMPU(ax, ay, az, gx, gy, gz)) return;

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;
  if (dt <= 0 || dt > 0.1) return;  // skip bad timing

  // Angular rates in rad/s (handle frame, bias already removed)
  float wx = gx * DEG_TO_RAD;
  float wy = gy * DEG_TO_RAD;
  float wz = gz * DEG_TO_RAD;

  // 1) Predict: rotate the gravity direction by the gyro rotation
  float px = vx - (wy * vz - wz * vy) * dt;
  float py = vy - (wz * vx - wx * vz) * dt;
  float pz = vz - (wx * vy - wy * vx) * dt;

  // 2) Correct: blend in the accelerometer, only when it is close to 1 g
  float amag = sqrt(ax * ax + ay * ay + az * az);
  if (amag > ACC_MIN && amag < ACC_MAX) {
    px = ALPHA * px + (1.0 - ALPHA) * (ax / amag);
    py = ALPHA * py + (1.0 - ALPHA) * (ay / amag);
    pz = ALPHA * pz + (1.0 - ALPHA) * (az / amag);
  }

  float m = sqrt(px * px + py * py + pz * pz);
  if (m < 0.1) return;
  vx = px / m; vy = py / m; vz = pz / m;

  // 3) Handle tilt from the gravity direction
  float yz = sqrt(vy * vy + vz * vz);
  float pitchRaw = atan2(-vx, yz) * RAD_TO_DEG;
  float rollRaw  = rollAngle + rollOffset;          // hold last roll near gimbal lock
  if (yz > 0.2) rollRaw = atan2(vy, vz) * RAD_TO_DEG;

  rollAngle  = rollRaw  - rollOffset;
  pitchAngle = pitchRaw - pitchOffset;

  // 4) Counter-rotate the servos to keep the spoon upright
  float rollCmd  = -ROLL_DIR  * (rollAngle  + ROLL_TRIM);
  float pitchCmd = -PITCH_DIR * (pitchAngle + PITCH_TRIM);

  rollCmd  = constrain(rollCmd,  -MAX_TILT, MAX_TILT);
  pitchCmd = constrain(pitchCmd, -MAX_TILT, MAX_TILT);

  float rollTarget  = 90 + rollCmd;
  float pitchTarget = 90 + pitchCmd;

  // Smooth output and apply deadband
  float newRoll  = rollOut  + SMOOTHING * (rollTarget  - rollOut);
  float newPitch = pitchOut + SMOOTHING * (pitchTarget - pitchOut);

  if (fabs(newRoll - rollOut) > DEADBAND) {
    rollOut = newRoll;
    writeServoAngle(rollServo, rollOut);
  }
  if (fabs(newPitch - pitchOut) > DEADBAND) {
    pitchOut = newPitch;
    writeServoAngle(pitchServo, pitchOut);
  }

  if (DEBUG && millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print("Roll: "); Serial.print(rollAngle, 1);
    Serial.print("  Pitch: "); Serial.print(pitchAngle, 1);
    Serial.print("  -> servos R:"); Serial.print((int)rollOut);
    Serial.print(" P:"); Serial.println((int)pitchOut);
  }

  delay(5);  // ~150-200 Hz loop
}  
