#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int belakang_kanan_pin[] = {16, 13, 12};
const int belakang_kiri_pin[] = {17, 23, 25};
const int depan_kanan_pin[] = {18, 19, 26};
const int depan_kiri_pin[] = {4, 2, 14};

float kp = 1.7, ki = 0.1, kd = 0.1; 
float prevError = 0, integral = 0;

float targetAngle = 0;

int baseSpeed = 100; 
float gyroDrift = 0.0;

unsigned long lastTime = 0;

float angle = 0;

const float smoothFactor = 0.1; 

float frontLeftSpeed = 0, frontRightSpeed = 0;
float rearLeftSpeed = 0, rearRightSpeed = 0;

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  setupMotors();
  calibrateGyro();
  lastTime = millis();
}

void loop() {
  float currentAngle = getYawAngle();

  float correction = calculatePID(targetAngle, currentAngle);

  float newFrontLeftSpeed = baseSpeed - correction;
  float newFrontRightSpeed = baseSpeed + correction;
  float newRearLeftSpeed = baseSpeed - correction;
  float newRearRightSpeed = baseSpeed + correction;

  frontLeftSpeed += (newFrontLeftSpeed - frontLeftSpeed) * smoothFactor;
  frontRightSpeed += (newFrontRightSpeed - frontRightSpeed) * smoothFactor;
  rearLeftSpeed += (newRearLeftSpeed - rearLeftSpeed) * smoothFactor;
  rearRightSpeed += (newRearRightSpeed - rearRightSpeed) * smoothFactor;

  int finalFrontLeftSpeed = constrain(frontLeftSpeed, -150, 150);
  int finalFrontRightSpeed = constrain(frontRightSpeed, -150, 150);
  int finalRearLeftSpeed = constrain(rearLeftSpeed, -150, 150);
  int finalRearRightSpeed = constrain(rearRightSpeed, -150, 150);

  Serial.print("Current Angle (degrees): ");
  Serial.print(currentAngle);
  Serial.print(" | FL: ");
  Serial.print(finalFrontLeftSpeed);
  Serial.print(" | FR: ");
  Serial.print(finalFrontRightSpeed);
  Serial.print(" | RL: ");
  Serial.print(finalRearLeftSpeed);
  Serial.print(" | RR: ");
  Serial.print(finalRearRightSpeed);
  Serial.print(" | Correction: ");
  Serial.println(correction);

  setMotorSpeed(depan_kiri_pin, finalFrontLeftSpeed);
  setMotorSpeed(depan_kanan_pin, finalFrontRightSpeed);
  setMotorSpeed(belakang_kiri_pin, finalRearLeftSpeed);
  setMotorSpeed(belakang_kanan_pin, finalRearRightSpeed);

  delay(50); 
}

void setupMotors() {
  for (int i = 0; i < 3; i++) {
    pinMode(depan_kiri_pin[i], OUTPUT);
    pinMode(depan_kanan_pin[i], OUTPUT);
    pinMode(belakang_kiri_pin[i], OUTPUT);
    pinMode(belakang_kanan_pin[i], OUTPUT);
  }
}

float calculatePID(float setpoint, float measured) {
  float error = setpoint - measured;
  integral += error * 0.05; 
  float derivative = (error - prevError) / 0.05; 
  prevError = error;

  return kp * error + ki * integral + kd * derivative;
}

void calibrateGyro() {
  Serial.println("Calibrating gyroscope...");
  float sum = 0;
  const int numSamples = 500;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    sum += gyro.gyro.z;
    delay(5); 
  }

  gyroDrift = sum / numSamples; 
  Serial.print("Gyro drift (rad/s): ");
  Serial.println(gyroDrift, 6);
}

float getYawAngle() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; 
  lastTime = currentTime;

  float angularVelocity = gyro.gyro.z - gyroDrift; 
  const float radToDeg = 180.0 / 3.14159265359;    

  angle += angularVelocity * radToDeg * deltaTime;
  return angle;
}

void setMotorSpeed(const int motorPins[], int speed) {
  if (speed > 0) {
    analogWrite(motorPins[0], speed); 
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins[2], LOW);
  } else {
    analogWrite(motorPins[0], -speed); 
    digitalWrite(motorPins[1], LOW);
    digitalWrite(motorPins[2], HIGH);
  }
}
