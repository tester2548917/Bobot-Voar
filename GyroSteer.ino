#include <Wire.h>

// PINS
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY 9

// MPU REGS
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_ZOUT_H 0x47

// TUNING
float base_speed = 1.0;
float Kp = 2.2;
float Ki = 0.30;
float Kd = 0.45;

// *** CHANGE THIS IF IT CORRECTS THE WRONG WAY ***
const int STEER_SIGN = -1;

// STATE
float yaw_angle = 0.0;
float target_yaw = 0.0;
float gyro_bias = 0.0;
float gyro_scale = 131.0;

float error_integral = 0.0;
float last_error = 0.0;

// MOTOR
void setMotors(float left_speed, float right_speed) {
  left_speed  = constrain(left_speed,  -1.0, 1.0);
  right_speed = constrain(right_speed, -1.0, 1.0);

  digitalWrite(PIN_Motor_AIN_1, (right_speed >= 0) ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMA, (int)(fabs(right_speed) * 255));

  digitalWrite(PIN_Motor_BIN_1, (left_speed >= 0) ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMB, (int)(fabs(left_speed) * 255));
}

// MPU
void initMPU6050() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  delay(50);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);
}

float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_ZOUT_H);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(MPU6050_ADDR, 2, true) < 2) return 0;

  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  float degPerSec = raw / gyro_scale;
  float radPerSec = degPerSec * PI / 180.0;

  return radPerSec - gyro_bias;
}

void calibrateGyro() {
  delay(500);
  float sum = 0;
  int count = 0;

  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_ZOUT_H);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050_ADDR, 2, true) >= 2) {
      int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
      float degPerSec = raw / gyro_scale;
      sum += degPerSec * PI / 180.0;
      count++;
    }
    delay(4);
  }

  gyro_bias = (count > 0) ? (sum / count) : 0.0;
}

void setup() {
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);

  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);

  delay(300);
  digitalWrite(PIN_Motor_STBY, HIGH);

  initMPU6050();
  calibrateGyro();

  yaw_angle = 0.0;
  target_yaw = 0.0;
  error_integral = 0.0;
  last_error = 0.0;
}

void loop() {
  static unsigned long last_loop_time = 0;
  unsigned long now = micros();
  unsigned long dt_us = now - last_loop_time;

  if (dt_us >= 5000) {  // every 5 ms (200 Hz)
    last_loop_time = now;
    float dt = dt_us / 1e6;

    // Yaw integration
    yaw_angle += readGyroZ() * dt;

    // PID
    float error = target_yaw - yaw_angle;
    float P = Kp * error;

    error_integral += error * dt;
    error_integral = constrain(error_integral, -2.0, 2.0);
    float I = Ki * error_integral;

    float error_rate = (error - last_error) / dt;
    float D = Kd * error_rate;
    last_error = error;

    float correction = STEER_SIGN * (P + I + D);
    correction = constrain(correction, -0.6, 0.6);

    float left_speed  = base_speed + correction;
    float right_speed = base_speed - correction;

    setMotors(left_speed, right_speed);
  }
}
