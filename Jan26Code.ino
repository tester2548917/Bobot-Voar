#include <Wire.h>
#include <math.h>

// ===================== MOTOR PINS =====================
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 9

// ===================== ENCODER PINS =====================
#define ENCODER_RIGHT_C1 2
#define ENCODER_RIGHT_C2 4
#define ENCODER_LEFT_C1 3
#define ENCODER_LEFT_C2 11

// ===================== MPU6050 =====================
#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_INT_ENABLE   0x38
#define REG_ACCEL_XOUT_H 0x3B

// ===================== TUNING =====================
#define CAL_SAMPLES 3000
#define GYRO_LSB_PER_DPS 131.0f
#define YAW_SIGN (-1)

// Turning (KEEPING YOUR “PERFECT TURNING” BASE)
#define TURN_PWM 120
#define TURN_TOL_DEG 0.75f
#define TURN_STABLE_MS 80
#define TURN_SCALE 0.97f //0.96 //.958

#define USE_3_STAGE_TURN true
#define TURN_STAGE2_DEG 20.0f
#define TURN_STAGE3_DEG 6.0f
#define TURN_PWM_MED   85
#define TURN_PWM_SLOW  45
#define TURN_MIN_PWM   40
#define TURN_TIMEOUT_MS 4000
#define TURN_STOP_IN_WINDOW true

// Driving distance -> encoder conversion
#define COUNTS_PER_50CM 4010.0f
#define COUNTS_PER_CM (COUNTS_PER_50CM / 50.0f)

// Driving PWM staging
#define USE_3_STAGE_DRIVE true
#define DRIVE_PWM_FAST 140
#define DRIVE_PWM_MED  100
#define DRIVE_PWM_SLOW 70
#define DRIVE_PWM_MIN  50

#define DRIVE_STAGE2_CM 10.0f
#define DRIVE_STAGE3_CM 3.0f

#define DRIVE_PWM_MAX 255

// ===================== STRAIGHT-LINE HEADING PID (from your 2nd program, adapted) =====================
// NOTE: This PID runs on heading error in DEGREES (not radians) but otherwise follows the same structure.
#define DRIVE_STEER_SIGN (1)      // matches your “STEER_SIGN” idea
#define DRIVE_PID_HZ 200           // 200 Hz loop like your second program
#define DRIVE_PID_DT_US (1000000UL / DRIVE_PID_HZ)

// Start with your second-program values; you can tweak.
// (Because we're in degrees, these are "deg-based" gains. If it over/under-corrects, adjust Kp first.)
float DRIVE_KP_PID = 3.50f;
float DRIVE_KI_PID = 3.50f;
float DRIVE_KD_PID = 1.70f;

float drive_err_i = 0.0f;
float drive_last_err = 0.0f;

// Clamp integral and correction (like your second code)
#define DRIVE_I_CLAMP 80.0f        // integral clamp in "deg*s" units (tune if needed)
#define DRIVE_CORR_CLAMP 120.0f    // correction clamp in PWM units (tune if needed)

// Optional mild deadband around 0 deg (helps jitter)
#define DRIVE_ERR_DEADBAND_DEG 0.25f

// ===================== STATE =====================
bool ranSequence = false;
float gyroBiasZ = 0.0f;
float headingDeg = 0.0f;
uint32_t lastMicros = 0;

// ===================== ENCODER STATE =====================
volatile long rightCount = 0;
volatile long leftCount  = 0;

// ===================== I2C HELPERS =====================
inline void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

inline bool mpuReadGyroZ(int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;

  for (uint8_t i = 0; i < 12; i++) Wire.read();
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

// ===================== MPU INIT =====================
void initMPU6050() {
  mpuWrite(REG_PWR_MGMT_1, 0x01);
  mpuWrite(REG_GYRO_CONFIG, 0x00);
  mpuWrite(REG_CONFIG, 0x05);        // DLPF 10Hz
  mpuWrite(REG_SMPLRT_DIV, 0x04);    // ~200Hz
  mpuWrite(REG_INT_ENABLE, 0x00);
}

// ===================== CALIBRATION =====================
void calibrateGyroZ() {
  delay(2000);  // warm-up

  int64_t sum = 0;
  uint16_t count = 0;

  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    int16_t gz;
    if (mpuReadGyroZ(gz)) {
      sum += gz;
      count++;
    }
    delayMicroseconds(1000);
  }

  gyroBiasZ = (count == 0) ? 0.0f : (float)sum / (float)count;
  headingDeg = 0.0f;
  lastMicros = micros();
}

// ===================== MATH =====================
inline float wrap180(float a) {
  while (a <= -180.0f) a += 360.0f;
  while (a >  180.0f)  a -= 360.0f;
  return a;
}

inline float angleError(float target, float current) {
  return wrap180(target - current);
}

// ===================== YAW UPDATE =====================
void updateYaw() {
  int16_t gz;
  if (!mpuReadGyroZ(gz)) return;

  uint32_t now = micros();
  uint32_t dtMicros = now - lastMicros;
  lastMicros = now;

  if (dtMicros == 0 || dtMicros > 200000) return;

  float dt = dtMicros * 1e-6f;
  float rateDps = ((gz - gyroBiasZ) / GYRO_LSB_PER_DPS) * (float)YAW_SIGN;
  headingDeg = wrap180(headingDeg + rateDps * dt);
}

// ===================== MOTOR CONTROL =====================
inline void motorsStandby(bool enable) {
  digitalWrite(PIN_Motor_STBY, enable ? HIGH : LOW);
}

inline void motorsStop() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}

inline void setRightMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(PIN_Motor_AIN_1, (pwm >= 0) ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMA, abs(pwm));
}

inline void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(PIN_Motor_BIN_1, (pwm >= 0) ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMB, abs(pwm));
}

void spinInPlace(int pwm) {
  setLeftMotor(+pwm);
  setRightMotor(-pwm);
}

// ===================== ENCODER HELPERS =====================
inline void resetEncoders() {
  noInterrupts();
  rightCount = 0;
  leftCount = 0;
  interrupts();
}

inline long getRightCount() {
  noInterrupts();
  long v = rightCount;
  interrupts();
  return v;
}

inline long getLeftCount() {
  noInterrupts();
  long v = leftCount;
  interrupts();
  return v;
}

// ===================== TURN FUNCTIONS (UNCHANGED) =====================
void turnBy(float deltaDeg) {
  updateYaw();
  float target = wrap180(headingDeg + deltaDeg * TURN_SCALE);

  uint32_t stableStart = 0;
  uint32_t startMs = millis();

  while (true) {
    updateYaw();

    if (millis() - startMs > TURN_TIMEOUT_MS) {
      motorsStop();
      break;
    }

    float err = angleError(target, headingDeg);
    float aerr = fabs(err);

    if (aerr <= TURN_TOL_DEG) {
      if (TURN_STOP_IN_WINDOW) motorsStop();

      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart >= TURN_STABLE_MS) break;

      if (TURN_STOP_IN_WINDOW) continue;
    } else {
      stableStart = 0;
    }

    int dir = (err >= 0) ? +1 : -1;
    int pwm = TURN_PWM;

    if (USE_3_STAGE_TURN) {
      if (aerr <= TURN_STAGE3_DEG) pwm = TURN_PWM_SLOW;
      else if (aerr <= TURN_STAGE2_DEG) pwm = TURN_PWM_MED;
      else pwm = TURN_PWM;
    } else {
      pwm = (aerr <= TURN_STAGE2_DEG) ? TURN_PWM_MED : TURN_PWM;
    }

    if (pwm < TURN_MIN_PWM) pwm = TURN_MIN_PWM;
    spinInPlace(dir * pwm);
  }

  motorsStop();
  resetEncoders();
  delay(30);
  resetEncoders();
}

void turnRight(float deg) { turnBy(+fabs(deg)); }
void turnLeft(float deg)  { turnBy(-fabs(deg)); }

// ===================== DRIVE (ENCODERS + “PERFECT STRAIGHT” PID) =====================
void drive(float distanceCm) {
  int dir = (distanceCm >= 0) ? +1 : -1;
  float distAbs = fabs(distanceCm);
  long targetCounts = (long)(distAbs * COUNTS_PER_CM + 0.5f);

  updateYaw();
  float targetHeading = headingDeg;

  resetEncoders();
  delay(30);
  resetEncoders();

  // Reset the straight-line PID state each drive
  drive_err_i = 0.0f;
  drive_last_err = 0.0f;

  uint32_t lastPidUs = micros();
  uint32_t lastPrint = 0;

  Serial.println("---- DRIVE START ----");
  Serial.print("cmdCm=");
  Serial.print(distanceCm, 2);
  Serial.print(" targetCounts=");
  Serial.println(targetCounts);

  while (true) {
    updateYaw();

    long r = getRightCount();
    long l = getLeftCount();

    long progress = (abs(r) + abs(l)) / 2;
    long remainingCounts = targetCounts - progress;
    float remainingCm = remainingCounts / COUNTS_PER_CM;

    // base PWM staging (same as your first program)
    int basePWM;
    if (USE_3_STAGE_DRIVE) {
      if (remainingCm <= DRIVE_STAGE3_CM) basePWM = DRIVE_PWM_SLOW;
      else if (remainingCm <= DRIVE_STAGE2_CM) basePWM = DRIVE_PWM_MED;
      else basePWM = DRIVE_PWM_FAST;
    } else {
      basePWM = (remainingCm <= DRIVE_STAGE2_CM) ? DRIVE_PWM_MED : DRIVE_PWM_FAST;
    }

    // Run heading PID at fixed rate (200 Hz), like your second program
    uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - lastPidUs;
    if (dtUs >= DRIVE_PID_DT_US) {
      lastPidUs = nowUs;
      float dt = dtUs / 1e6f;

      // Error in degrees (wrapped)
      float err = angleError(targetHeading, headingDeg);

      // deadband to reduce buzzing
      if (fabs(err) < DRIVE_ERR_DEADBAND_DEG) err = 0.0f;

      // PID terms
      float P = DRIVE_KP_PID * err;

      drive_err_i += err * dt;
      drive_err_i = constrain(drive_err_i, -DRIVE_I_CLAMP, DRIVE_I_CLAMP);
      float I = DRIVE_KI_PID * drive_err_i;

      float derr = (dt > 0) ? ((err - drive_last_err) / dt) : 0.0f;
      float D = DRIVE_KD_PID * derr;
      drive_last_err = err;

      // Convert PID output into PWM steer correction (same idea as your second code)
      float corr = (float)DRIVE_STEER_SIGN * (P + I + D);
      corr = constrain(corr, -(float)DRIVE_CORR_CLAMP, (float)DRIVE_CORR_CLAMP);

      int steer = (int)lroundf(corr);

      int leftPWM  = dir * basePWM + steer;
      int rightPWM = dir * basePWM - steer;

      // Minimum effective PWM (same behavior as your first program)
      if (abs(leftPWM) < DRIVE_PWM_MIN && leftPWM != 0) {
        leftPWM = (leftPWM > 0) ? DRIVE_PWM_MIN : -DRIVE_PWM_MIN;
      }
      if (abs(rightPWM) < DRIVE_PWM_MIN && rightPWM != 0) {
        rightPWM = (rightPWM > 0) ? DRIVE_PWM_MIN : -DRIVE_PWM_MIN;
      }

      leftPWM  = constrain(leftPWM,  -DRIVE_PWM_MAX, DRIVE_PWM_MAX);
      rightPWM = constrain(rightPWM, -DRIVE_PWM_MAX, DRIVE_PWM_MAX);

      setLeftMotor(leftPWM);
      setRightMotor(rightPWM);

      // Debug print ~20 Hz
      if (millis() - lastPrint >= 50) {
        lastPrint = millis();
        Serial.print("R=");
        Serial.print(r);
        Serial.print(" L=");
        Serial.print(l);
        Serial.print(" prog=");
        Serial.print(progress);
        Serial.print("/");
        Serial.print(targetCounts);
        Serial.print(" remCm=");
        Serial.print(remainingCm, 1);

        Serial.print(" basePWM=");
        Serial.print(basePWM);
        Serial.print(" head=");
        Serial.print(headingDeg, 2);
        Serial.print(" tgtHead=");
        Serial.print(targetHeading, 2);

        Serial.print(" errDeg=");
        Serial.print(err, 2);
        Serial.print(" I=");
        Serial.print(drive_err_i, 2);

        Serial.print(" steer=");
        Serial.print(steer);
        Serial.print(" pwmL=");
        Serial.print(leftPWM);
        Serial.print(" pwmR=");
        Serial.println(rightPWM);
      }
    }

    if (progress >= targetCounts) break;
  }

  motorsStop();

  long rEnd = getRightCount();
  long lEnd = getLeftCount();
  long progEnd = (abs(rEnd) + abs(lEnd)) / 2;

  Serial.println(">>> DRIVE END");
  Serial.print("Rend=");
  Serial.print(rEnd);
  Serial.print(" Lend=");
  Serial.print(lEnd);
  Serial.print(" progEnd=");
  Serial.print(progEnd);
  Serial.print("/");
  Serial.println(targetCounts);
  Serial.println("---- DRIVE STOP ----");
}

// ===================== ENCODER ISRs =====================
// NOTE: same as your first program (counts only; direction handled by abs() in drive)
void isrRight() { rightCount++; }
void isrLeft()  { leftCount++;  }

// ===================== ARDUINO =====================
void setup() {
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  motorsStandby(true);
  motorsStop();

  Serial.begin(115200);
  delay(500);

  pinMode(ENCODER_RIGHT_C1, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_C2, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_C1, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_C1), isrRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_C1),  isrLeft,  RISING);

  Wire.begin();
  Wire.setClock(400000);

  initMPU6050();
  calibrateGyroZ();
}

void loop() {
  updateYaw();

  if (!ranSequence) {
    drive(200);
    turnRight(90);
    drive(20);
    turnRight(90);
    drive(200);
    turnRight(90);
    drive(20);
    turnRight(90);

    ranSequence = true;
  }

  motorsStop();
}
