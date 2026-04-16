#include <Wire.h>
#include <ESP32Servo.h>

const uint8_t MPU_ADDR = 0x68;
const float RAD_TO_DEG = 57.2957795f;
const float dt = 0.004f;

// IMU raw and converted values
int16_t accXRaw, accYRaw, accZRaw;
int16_t gyroXRaw, gyroYRaw, gyroZRaw;

float accX = 0.0f, accY = 0.0f, accZ = 0.0f;
float rateRoll = 0.0f, ratePitch = 0.0f, rateYaw = 0.0f;

// paste calibration values here
float rateRollOffset = 0.0f;
float ratePitchOffset = 0.0f;
float rateYawOffset = 0.0f;
float accXOffset = 0.0f;
float accYOffset = 0.0f;
float accZOffset = 0.0f;

float angleRoll = 0.0f;
float anglePitch = 0.0f;
float compAngleRoll = 0.0f;
float compAnglePitch = 0.0f;

// motors
Servo motor1, motor2, motor3, motor4;

const int motor1Pin = 13;
const int motor2Pin = 12;
const int motor3Pin = 14;
const int motor4Pin = 27;

const int escFrequency = 500;
const int throttleIdle = 1170;
const int throttleCutoff = 1000;

float motor1Out = 1000.0f;
float motor2Out = 1000.0f;
float motor3Out = 1000.0f;
float motor4Out = 1000.0f;

// receiver
volatile int receiverValue[6] = {1500, 1500, 1000, 1500, 1500, 1500};

const int ch1Pin = 34;
const int ch2Pin = 35;
const int ch3Pin = 32;
const int ch4Pin = 33;
const int ch5Pin = 25;
const int ch6Pin = 26;

volatile uint32_t currentTime = 0;
volatile uint32_t lastRise1 = 0, lastRise2 = 0, lastRise3 = 0, lastRise4 = 0, lastRise5 = 0, lastRise6 = 0;
volatile bool lastState1 = false, lastState2 = false, lastState3 = false, lastState4 = false, lastState5 = false, lastState6 = false;

// pid gains
float kpAngleRoll = 2.0f;
float kiAngleRoll = 0.5f;
float kdAngleRoll = 0.007f;

float kpAnglePitch = 2.0f;
float kiAnglePitch = 0.5f;
float kdAnglePitch = 0.007f;

float kpRateRoll = 0.625f;
float kiRateRoll = 2.1f;
float kdRateRoll = 0.0088f;

float kpRatePitch = 0.625f;
float kiRatePitch = 2.1f;
float kdRatePitch = 0.0088f;

float kpRateYaw = 4.0f;
float kiRateYaw = 3.0f;
float kdRateYaw = 0.0f;

// pid state
float desiredAngleRoll = 0.0f;
float desiredAnglePitch = 0.0f;
float desiredRateYaw = 0.0f;

float desiredRateRoll = 0.0f;
float desiredRatePitch = 0.0f;

float errorAngleRoll = 0.0f;
float errorAnglePitch = 0.0f;
float prevErrorAngleRoll = 0.0f;
float prevErrorAnglePitch = 0.0f;
float iTermAngleRoll = 0.0f;
float iTermAnglePitch = 0.0f;

float errorRateRoll = 0.0f;
float errorRatePitch = 0.0f;
float errorRateYaw = 0.0f;
float prevErrorRateRoll = 0.0f;
float prevErrorRatePitch = 0.0f;
float prevErrorRateYaw = 0.0f;
float iTermRateRoll = 0.0f;
float iTermRatePitch = 0.0f;
float iTermRateYaw = 0.0f;

float outRoll = 0.0f;
float outPitch = 0.0f;
float outYaw = 0.0f;

uint32_t loopTimer = 0;

float clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float pidStep(float error, float kp, float ki, float kd, float &integral, float &prevError) {
  integral += ki * error * dt;
  integral = clampf(integral, -400.0f, 400.0f);

  float derivative = kd * (error - prevError) / dt;
  float output = kp * error + integral + derivative;

  prevError = error;
  return clampf(output, -400.0f, 400.0f);
}

void setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return;

  accXRaw = (Wire.read() << 8) | Wire.read();
  accYRaw = (Wire.read() << 8) | Wire.read();
  accZRaw = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return;

  gyroXRaw = (Wire.read() << 8) | Wire.read();
  gyroYRaw = (Wire.read() << 8) | Wire.read();
  gyroZRaw = (Wire.read() << 8) | Wire.read();

  accX = (float)accXRaw / 4096.0f;
  accY = (float)accYRaw / 4096.0f;
  accZ = (float)accZRaw / 4096.0f;

  rateRoll = (float)gyroXRaw / 65.5f;
  ratePitch = (float)gyroYRaw / 65.5f;
  rateYaw = (float)gyroZRaw / 65.5f;

  rateRoll -= rateRollOffset;
  ratePitch -= ratePitchOffset;
  rateYaw -= rateYawOffset;

  accX -= accXOffset;
  accY -= accYOffset;
  accZ -= accZOffset;

  angleRoll = atan2f(accY, sqrtf(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  anglePitch = -atan2f(accX, sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG;
}

void IRAM_ATTR readReceiver() {
  currentTime = micros();

  bool s1 = digitalRead(ch1Pin);
  if (s1 && !lastState1) {
    lastState1 = true;
    lastRise1 = currentTime;
  } else if (!s1 && lastState1) {
    lastState1 = false;
    receiverValue[0] = currentTime - lastRise1;
  }

  bool s2 = digitalRead(ch2Pin);
  if (s2 && !lastState2) {
    lastState2 = true;
    lastRise2 = currentTime;
  } else if (!s2 && lastState2) {
    lastState2 = false;
    receiverValue[1] = currentTime - lastRise2;
  }

  bool s3 = digitalRead(ch3Pin);
  if (s3 && !lastState3) {
    lastState3 = true;
    lastRise3 = currentTime;
  } else if (!s3 && lastState3) {
    lastState3 = false;
    receiverValue[2] = currentTime - lastRise3;
  }

  bool s4 = digitalRead(ch4Pin);
  if (s4 && !lastState4) {
    lastState4 = true;
    lastRise4 = currentTime;
  } else if (!s4 && lastState4) {
    lastState4 = false;
    receiverValue[3] = currentTime - lastRise4;
  }

  bool s5 = digitalRead(ch5Pin);
  if (s5 && !lastState5) {
    lastState5 = true;
    lastRise5 = currentTime;
  } else if (!s5 && lastState5) {
    lastState5 = false;
    receiverValue[4] = currentTime - lastRise5;
  }

  bool s6 = digitalRead(ch6Pin);
  if (s6 && !lastState6) {
    lastState6 = true;
    lastRise6 = currentTime;
  } else if (!s6 && lastState6) {
    lastState6 = false;
    receiverValue[5] = currentTime - lastRise6;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(ch3Pin, INPUT);
  pinMode(ch4Pin, INPUT);
  pinMode(ch5Pin, INPUT);
  pinMode(ch6Pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(ch1Pin), readReceiver, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), readReceiver, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch3Pin), readReceiver, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch4Pin), readReceiver, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch5Pin), readReceiver, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch6Pin), readReceiver, CHANGE);

  Wire.begin();
  Wire.setClock(400000);
  setupMPU();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motor1.setPeriodHertz(escFrequency);
  motor2.setPeriodHertz(escFrequency);
  motor3.setPeriodHertz(escFrequency);
  motor4.setPeriodHertz(escFrequency);

  motor1.attach(motor1Pin, 1000, 2000);
  motor2.attach(motor2Pin, 1000, 2000);
  motor3.attach(motor3Pin, 1000, 2000);
  motor4.attach(motor4Pin, 1000, 2000);

  motor1.writeMicroseconds(throttleCutoff);
  motor2.writeMicroseconds(throttleCutoff);
  motor3.writeMicroseconds(throttleCutoff);
  motor4.writeMicroseconds(throttleCutoff);

  delay(2000);
  loopTimer = micros();
}

void loop() {
  readMPU();

  compAngleRoll = 0.98f * (compAngleRoll + rateRoll * dt) + 0.02f * angleRoll;
  compAnglePitch = 0.98f * (compAnglePitch + ratePitch * dt) + 0.02f * anglePitch;

  compAngleRoll = clampf(compAngleRoll, -20.0f, 20.0f);
  compAnglePitch = clampf(compAnglePitch, -20.0f, 20.0f);

  desiredAngleRoll = (receiverValue[0] - 1500) * 0.1f;
  desiredAnglePitch = (receiverValue[1] - 1500) * 0.1f;
  desiredRateYaw = (receiverValue[3] - 1500) * 0.15f;

  float throttle = (float)receiverValue[2];

  if (throttle < 1030.0f) {
    motor1Out = throttleCutoff;
    motor2Out = throttleCutoff;
    motor3Out = throttleCutoff;
    motor4Out = throttleCutoff;

    iTermAngleRoll = 0.0f;
    iTermAnglePitch = 0.0f;
    iTermRateRoll = 0.0f;
    iTermRatePitch = 0.0f;
    iTermRateYaw = 0.0f;

    prevErrorAngleRoll = 0.0f;
    prevErrorAnglePitch = 0.0f;
    prevErrorRateRoll = 0.0f;
    prevErrorRatePitch = 0.0f;
    prevErrorRateYaw = 0.0f;

    motor1.writeMicroseconds((int)motor1Out);
    motor2.writeMicroseconds((int)motor2Out);
    motor3.writeMicroseconds((int)motor3Out);
    motor4.writeMicroseconds((int)motor4Out);

    while (micros() - loopTimer < (uint32_t)(dt * 1000000.0f)) {}
    loopTimer = micros();
    return;
  }

  throttle = clampf(throttle, (float)throttleCutoff, 1800.0f);

  errorAngleRoll = desiredAngleRoll - compAngleRoll;
  errorAnglePitch = desiredAnglePitch - compAnglePitch;

  desiredRateRoll = pidStep(errorAngleRoll, kpAngleRoll, kiAngleRoll, kdAngleRoll, iTermAngleRoll, prevErrorAngleRoll);
  desiredRatePitch = pidStep(errorAnglePitch, kpAnglePitch, kiAnglePitch, kdAnglePitch, iTermAnglePitch, prevErrorAnglePitch);

  errorRateRoll = desiredRateRoll - rateRoll;
  errorRatePitch = desiredRatePitch - ratePitch;
  errorRateYaw = desiredRateYaw - rateYaw;

  outRoll = pidStep(errorRateRoll, kpRateRoll, kiRateRoll, kdRateRoll, iTermRateRoll, prevErrorRateRoll);
  outPitch = pidStep(errorRatePitch, kpRatePitch, kiRatePitch, kdRatePitch, iTermRatePitch, prevErrorRatePitch);
  outYaw = pidStep(errorRateYaw, kpRateYaw, kiRateYaw, kdRateYaw, iTermRateYaw, prevErrorRateYaw);

  motor1Out = throttle - outRoll - outPitch - outYaw;
  motor2Out = throttle - outRoll + outPitch + outYaw;
  motor3Out = throttle + outRoll + outPitch - outYaw;
  motor4Out = throttle + outRoll - outPitch + outYaw;

  motor1Out = clampf(motor1Out, throttleCutoff, 2000.0f);
  motor2Out = clampf(motor2Out, throttleCutoff, 2000.0f);
  motor3Out = clampf(motor3Out, throttleCutoff, 2000.0f);
  motor4Out = clampf(motor4Out, throttleCutoff, 2000.0f);

  if (motor1Out < throttleIdle) motor1Out = throttleIdle;
  if (motor2Out < throttleIdle) motor2Out = throttleIdle;
  if (motor3Out < throttleIdle) motor3Out = throttleIdle;
  if (motor4Out < throttleIdle) motor4Out = throttleIdle;

  motor1.writeMicroseconds((int)motor1Out);
  motor2.writeMicroseconds((int)motor2Out);
  motor3.writeMicroseconds((int)motor3Out);
  motor4.writeMicroseconds((int)motor4Out);

  while (micros() - loopTimer < (uint32_t)(dt * 1000000.0f)) {}
  loopTimer = micros();
}