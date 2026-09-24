/*
  Self-Balancing Robot - cascade PID structure (ported from CoppeliaSim design)
  Hardware: Arduino Nano, MPU6050 (I2C), TB6612FNG, 2x N20 (no encoders)

  Wiring:
    MPU6050  VCC->5V, GND->GND, SDA->A4, SCL->A5
    TB6612   VM->battery+ (7.4V), VCC->5V (buck), GND->common GND
    TB6612   STBY->D3, PWMA->D5, AIN1->D4, AIN2->D2, PWMB->D6, BIN1->D7, BIN2->D8

  Tuning order:
    1. Leave ENABLE_OUTER_LOOP = false. Tune Kp, then Kd, then Ki for the
       inner loop until it balances in place reasonably well.
    2. Only then set ENABLE_OUTER_LOOP = true and tune KpVel/KiVel from
       very small values to trim out slow drift.
*/
// change the kp kd and ki values based on your bot , first change your kp , then kd and then ki 

#include <Wire.h>

// ---------------- MPU6050 ----------------
const int MPU_ADDR = 0x68;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double angle = 0;
double angleOffset = 0;
unsigned long lastTime;

// ---------------- Motor driver pins ----------------
const int STBY = 3;
const int AIN1 = 4;
const int AIN2 = 2;
const int PWMA = 5;   // Left motor
const int BIN1 = 7;
const int BIN2 = 8;
const int PWMB = 6;   // Right motor

// ---------------- Inner loop (angle) ----------------
double Kp = 22.0;
double Ki = 140.0;
double Kd = 0.9;

double error, previousError = 0;
double integral = 0;
double dFiltered = 0;
const double maxIntegral = 300;
const int MAX_PWM = 255;

// ---------------- Outer loop (drift trim, no encoders) ----------------
bool ENABLE_OUTER_LOOP = false;   // leave off until inner loop is stable

double KpVel = 0.02;    // start tiny - this is a proxy loop, not real velocity control
double KiVel = 0.005;
double outputFiltered = 0;
double velIntegral = 0;
const double maxTiltCorrection = 4.0;   // degrees - clamp so outer loop can't destabilize inner loop

double targetAngle = 0;
const double MAX_TILT = 40.0;   // safety cutoff, degrees

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);

  calibrateAngleOffset();
  lastTime = micros();
}

void loop() {
  readMPU();

  unsigned long now = micros();
  double dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  if (dt <= 0) return;

  updateAngle(dt);
  double currentAngle = angle - angleOffset;

  if (abs(currentAngle) > MAX_TILT) {
    stopMotors();
    integral = 0;
    velIntegral = 0;
    outputFiltered = 0;
    return;
  }

  // ---- outer loop: drift trim via filtered motor-command proxy ----
  if (ENABLE_OUTER_LOOP) {
    outputFiltered = outputFiltered + 0.02 * (lastMotorCommand - outputFiltered);
    velIntegral += outputFiltered * dt;
    double velCorrection = KpVel * outputFiltered + KiVel * velIntegral;
    velCorrection = constrain(velCorrection, -maxTiltCorrection, maxTiltCorrection);
    targetAngle = velCorrection;
  } else {
    targetAngle = 0;
  }

  // ---- inner loop: angle PID ----
  error = targetAngle - currentAngle;

  integral += error * dt;
  integral = constrain(integral, -maxIntegral, maxIntegral);

  double d = (error - previousError) / dt;
  dFiltered = dFiltered + 0.3 * (d - dFiltered);   // same low-pass idea as your sim code
  previousError = error;

  double motorCommand = Kp * error + Ki * integral + Kd * dFiltered;
  motorCommand = constrain(motorCommand, -MAX_PWM, MAX_PWM);
  lastMotorCommand = motorCommand;

  driveMotors(motorCommand);

  // Serial.print("angle: "); Serial.print(currentAngle);
  // Serial.print("  cmd: "); Serial.println(motorCommand);
}

double lastMotorCommand = 0;

// ---------------- Sensor reading ----------------
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void updateAngle(double dt) {
  double accAngle = atan2((double)accX, (double)accZ) * 180.0 / PI;
  double gyroRate = gyroY / 131.0;
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;
}

void calibrateAngleOffset() {
  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    readMPU();
    double accAngle = atan2((double)accX, (double)accZ) * 180.0 / PI;
    sum += accAngle;
    delay(2);
  }
  angleOffset = (double)sum / samples;
  angle = angleOffset;
}

// ---------------- Motor control ----------------
void driveMotors(double speed) {
  int pwm = (int)abs(speed);
  pwm = constrain(pwm, 0, MAX_PWM);
  bool forward = speed >= 0;
  setMotor(AIN1, AIN2, PWMA, forward, pwm);
  setMotor(BIN1, BIN2, PWMB, forward, pwm);
}

void setMotor(int in1, int in2, int pwmPin, bool forward, int pwm) {
  if (forward) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
