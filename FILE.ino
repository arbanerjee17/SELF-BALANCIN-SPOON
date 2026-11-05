#include <Wire.h>
#include <ESP32Servo.h>

// ---------- Pins ----------
const int SDA_PIN   = 6;
const int SCL_PIN   = 7;
const int ROLL_PIN  = 2;    // lower servo (shaft along handle)
const int PITCH_PIN = 3;    // upper servo (shaft sideways)
const int LED_PIN   = 10;

// ---------- Tuning (change these while testing) ----------
const int   ROLL_DIR   = 1;      // flip to -1 if roll compensates the wrong way
const int   PITCH_DIR  = 1;      // flip to -1 if pitch compensates the wrong way
const float ROLL_TRIM  = 0;      // degrees, fine-tune "level" position
const float PITCH_TRIM = 0;
const float ALPHA      = 0.98;   // gyro trust (0.95-0.99). Higher = smoother, slower drift correction
const float MAX_TILT   = 80;     // max servo travel from center, degrees
const float DEADBAND   = 0.2;    // ignore changes smaller than this (reduces jitter)
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

float gxBias = 0, gyBias = 0;
float rollAngle = 0, pitchAngle = 0;
float rollOut = 90, pitchOut = 90;
unsigned long lastMicros = 0;
unsigned long lastPrint = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;

  int16_t rax = (Wire.read() << 8) | Wire.read();
  int16_t ray = (Wire.read() << 8) | Wire.read();
  int16_t raz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  int16_t rgx = (Wire.read() << 8) | Wire.read();
  int16_t rgy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip gyro Z

  ax = rax / ACCEL_SCALE;
  ay = ray / ACCEL_SCALE;
  az = raz / ACCEL_SCALE;
  gx = rgx / GYRO_SCALE;
  gy = rgy / GYRO_SCALE;
  return true;
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

void calibrateGyro() {
  Serial.println("Calibrating gyro - hold the handle still...");
  const int N = 500;
  float sx = 0, sy = 0;
  float ax, ay, az, gx, gy;
  for (int i = 0; i < N; i++) {
    if (readMPU(ax, ay, az, gx, gy)) {
      sx += gx;
      sy += gy;
    }
    digitalWrite(LED_PIN, (i / 25) % 2);  // blink while calibrating
    delay(3);
  }
  gxBias = sx / N;
  gyBias = sy / N;
  digitalWrite(LED_PIN, LOW);
  Serial.print("Gyro bias X: "); Serial.print(gxBias);
  Serial.print("  Y: "); Serial.println(gyBias);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Servos: center first so the spoon starts level
  rollServo.setPeriodHertz(50);
  pitchServo.setPeriodHertz(50);
  rollServo.attach(ROLL_PIN, PULSE_MIN, PULSE_MAX);
  pitchServo.attach(PITCH_PIN, PULSE_MIN, PULSE_MAX);
  writeServoAngle(rollServo, 90);
  writeServoAngle(pitchServo, 90);

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

  delay(1500);           // let you settle the handle
  calibrateGyro();

  // Initial angles from accelerometer
  float ax, ay, az, gx, gy;
  if (readMPU(ax, ay, az, gx, gy)) {
    rollAngle  = atan2(ay, az) * RAD_TO_DEG;
    pitchAngle = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  }

  lastMicros = micros();
  digitalWrite(LED_PIN, HIGH);  // LED on = stabilizing active
  Serial.println("Stabilizing...");
}

void loop() {
  float ax, ay, az, gx, gy;
  if (!readMPU(ax, ay, az, gx, gy)) return;

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;
  if (dt <= 0 || dt > 0.1) return;  // skip bad timing

  gx -= gxBias;
  gy -= gyBias;

  // Angles from accelerometer (gravity direction)
  float accRoll  = atan2(ay, az) * RAD_TO_DEG;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Complementary filter: gyro for fast motion, accel to stop drift
  rollAngle  = ALPHA * (rollAngle  + gx * dt) + (1.0 - ALPHA) * accRoll;
  pitchAngle = ALPHA * (pitchAngle + gy * dt) + (1.0 - ALPHA) * accPitch;

  // Counter-rotate servos to keep spoon level
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
