#include <Wire.h>
#include <math.h>

#define PIN_START_BUTTON 12

// ===================== MOTOR PINS =====================
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 9

// ===================== ENCODER PINS =====================
// NOTE: For full quadrature (C1+C2) on Arduino Uno/Nano,
// only pins 2 & 3 support external interrupts. This code
// attaches interrupts to C1 (pins 2 and 3) and reads C2
// inside the ISR to determine direction (proper quadrature).
#define ENCODER_RIGHT_C1 2   // interrupt-capable
#define ENCODER_RIGHT_C2 4
#define ENCODER_LEFT_C1  3   // interrupt-capable
#define ENCODER_LEFT_C2  11

// ===================== MPU6050 =====================
#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_INT_ENABLE   0x38
#define REG_ACCEL_XOUT_H 0x3B

// ===================== TUNING =====================
#define CAL_SAMPLES 2000
#define GYRO_LSB_PER_DPS 16.4f
#define YAW_SIGN (-1)

// Turning
#define TURN_PWM 255 //200
#define TURN_TOL_DEG 0.75f
#define TURN_STABLE_MS 20
#define TURN_SCALE 0.96f  //0.96

#define USE_3_STAGE_TURN true
#define TURN_STAGE2_DEG 20.0f //20
#define TURN_STAGE3_DEG 10.0f //6
#define TURN_PWM_MED   100 //140
#define TURN_PWM_SLOW  45 //30
#define TURN_MIN_PWM   30 //20
#define TURN_TIMEOUT_MS 10000
#define TURN_STOP_IN_WINDOW true

// Driving distance -> encoder conversion
#define COUNTS_PER_50CM 3350.0f //4375 3575
#define COUNTS_PER_CM (COUNTS_PER_50CM / 50.0f)

// Driving PWM staging
#define USE_3_STAGE_DRIVE true
#define DRIVE_PWM_FAST 255 //200
#define DRIVE_PWM_MED  255 //160
#define DRIVE_PWM_SLOW 180 //140
#define DRIVE_PWM_MIN  0

#define DRIVE_STAGE2_CM 10.0f
#define DRIVE_STAGE3_CM 3.0f

#define DRIVE_PWM_MAX 255

// ===================== STRAIGHT-LINE HEADING PID =====================
#define DRIVE_STEER_SIGN (1)
#define DRIVE_PID_HZ 200
#define DRIVE_PID_DT_US (1000000UL / DRIVE_PID_HZ)

float DRIVE_KP_PID = 10.70f;
float DRIVE_KI_PID = 2.08f;
float DRIVE_KD_PID = 1.00f;

float drive_err_i = 0.0f;
float drive_last_err = 0.0f;

#define DRIVE_I_CLAMP 15.0f
#define DRIVE_CORR_CLAMP 120.0f
#define DRIVE_ERR_DEADBAND_DEG 0.0f

// ===================== STATE =====================
bool ranSequence = false;
float gyroBiasZ = 0.0f;
float headingDeg = 0.0f;
uint32_t lastMicros = 0;

// ===================== ENCODER STATE =====================
// Signed counts with true direction (quadrature decode using C2).
volatile long rightCount = 0;
volatile long leftCount  = 0;

// ===================== I2C LOCKUP HARDENING =====================
#define I2C_WIRE_TIMEOUT_US 2500UL
#define I2C_FAIL_STOP_COUNT 3
static uint8_t i2cFailCount = 0;

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
  mpuWrite(REG_GYRO_CONFIG, 0x18);
  mpuWrite(REG_CONFIG, 0x05);        // DLPF 10Hz
  mpuWrite(REG_SMPLRT_DIV, 0x04);    // ~200Hz
  mpuWrite(REG_INT_ENABLE, 0x00);
}

// ===================== CALIBRATION =====================
void calibrateGyroZ() {
  delay(2000); // warm-up

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
  i2cFailCount = 0;
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

// ===================== YAW UPDATE =====================
void updateYaw() {
  int16_t gz;
  if (!mpuReadGyroZ(gz)) {
    if (++i2cFailCount >= I2C_FAIL_STOP_COUNT) {
      motorsStop();
      initMPU6050();
      i2cFailCount = 0;
      lastMicros = micros();
    }
    return;
  }

  i2cFailCount = 0;

  uint32_t now = micros();
  uint32_t dtMicros = now - lastMicros;
  lastMicros = now;

  if (dtMicros == 0 || dtMicros > 200000) return;

  float dt = dtMicros * 1e-6f;
  float rateDps = ((gz - gyroBiasZ) / GYRO_LSB_PER_DPS) * (float)YAW_SIGN;
  headingDeg = wrap180(headingDeg + rateDps * dt);
}

// ===================== TURN FUNCTIONS =====================
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
  // Encoder resets after turns are optional; kept minimal:
  resetEncoders();
}

void turnRight(float deg) { turnBy(+fabs(deg)); }
void turnLeft(float deg)  { turnBy(-fabs(deg)); }

// ===================== DRIVE =====================
void drive(float distanceCm) {
  int dir = (distanceCm >= 0) ? +1 : -1;
  float distAbs = fabs(distanceCm);
  long targetCounts = (long)(distAbs * COUNTS_PER_CM + 0.5f);

  updateYaw();
  float targetHeading = headingDeg;

  resetEncoders();

  drive_err_i = 0.0f;
  drive_last_err = 0.0f;

  uint32_t lastPidUs = micros();

  while (true) {
    updateYaw();

    long r = getRightCount();
    long l = getLeftCount();

    // Use signed counts with direction; progress uses magnitude toward target.
    // long progress = (labs(r) + labs(l)) / 2;
    long progress = (r + l) / 2;
    long remainingCounts = targetCounts - labs(progress);
    float remainingCm = remainingCounts / COUNTS_PER_CM;

    int basePWM;
    if (USE_3_STAGE_DRIVE) {
      if (remainingCm <= DRIVE_STAGE3_CM) basePWM = DRIVE_PWM_SLOW;
      else if (remainingCm <= DRIVE_STAGE2_CM) basePWM = DRIVE_PWM_MED;
      else basePWM = DRIVE_PWM_FAST;
    } else {
      basePWM = (remainingCm <= DRIVE_STAGE2_CM) ? DRIVE_PWM_MED : DRIVE_PWM_FAST;
    }

    uint32_t nowUs = micros();
    uint32_t dtUs = nowUs - lastPidUs;
    if (dtUs >= DRIVE_PID_DT_US) {
      lastPidUs = nowUs;
      float dt = dtUs / 1e6f;

      float err = angleError(targetHeading, headingDeg);
      if (fabs(err) < DRIVE_ERR_DEADBAND_DEG) err = 0.0f;

      float P = DRIVE_KP_PID * err;

      drive_err_i += err * dt;
      drive_err_i = constrain(drive_err_i, -DRIVE_I_CLAMP, DRIVE_I_CLAMP);
      float I = DRIVE_KI_PID * drive_err_i;

      float derr = (dt > 0) ? ((err - drive_last_err) / dt) : 0.0f;
      float D = DRIVE_KD_PID * derr;
      drive_last_err = err;

      float corr = (float)DRIVE_STEER_SIGN * (P + I + D);
      corr = constrain(corr, -(float)DRIVE_CORR_CLAMP, (float)DRIVE_CORR_CLAMP);

      int steer = (int)lroundf(corr);

      int leftPWM  = dir * basePWM + steer;
      int rightPWM = dir * basePWM - steer;

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
    }

    // if (progress >= targetCounts) break;
    if (dir == 1  && progress >= targetCounts) break;
    if (dir == -1 && progress <= -targetCounts) break;
  }

  motorsStop();
}

// ===================== QUADRATURE ENCODER ISRs =====================
// Proper direction decode using C2 state.
//
// Assumption (standard quadrature):
// - When channel A (C1) changes, channel B (C2) indicates direction.
// If your counts go the wrong direction, flip the +/- in each ISR.
void isrRightC1() {
  bool b = digitalRead(ENCODER_RIGHT_C2);
  // On A rising: B==HIGH often means one direction, LOW the other.
  // If direction is reversed on your build, swap ++ and --.
  if (b) rightCount--;
  else   rightCount++;
}

void isrLeftC1() {
  bool b = digitalRead(ENCODER_LEFT_C2);
  if (b) leftCount++;
  else   leftCount--;
}

// ===================== ARDUINO =====================
void setup() {
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  motorsStandby(true);
  motorsStop();

  // No Serial at all (per request)

  pinMode(ENCODER_RIGHT_C1, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_C2, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_C1,  INPUT_PULLUP);
  pinMode(ENCODER_LEFT_C2,  INPUT_PULLUP);

  // Interrupt on C1, read C2 for direction
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_C1), isrRightC1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_C1),  isrLeftC1,  RISING);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(I2C_WIRE_TIMEOUT_US, true);

  initMPU6050();
  // calibrateGyroZ();
}

void loop() {
  updateYaw();

  if (!ranSequence && digitalRead(PIN_START_BUTTON) == LOW) {
    delay(50); //debounce
      if (digitalRead(PIN_START_BUTTON) == LOW) {
        calibrateGyroZ();
    // TRACK CODE
    drive(25);
    turnLeft(90);
    drive(50);
    turnRight(90);
    drive(150);
    turnLeft(90);
    drive(50);
    drive(-12.5);
    turnLeft(90);
    drive(150);
    turnLeft(90);
    drive(50);
    turnLeft(90);
    drive(100);
    turnRight(90);
    drive(50);
    turnLeft(90);
    drive(50);
    turnRight(90);
    drive(50);
    turnRight(90);
    drive(150);
    turnRight(180);
    drive(150);
    turnLeft(90);
    drive(100);
    turnLeft(90);
    drive(150);
    turnLeft(90);
    drive(50);
    // drive(250);

    ranSequence = true;
  }
  }
  motorsStop();
}
