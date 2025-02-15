#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Motor 1
const byte encA1 = 4;
const byte encB1 = 23;
const byte en1 = 19;
volatile int pos1 = 0;  // Position in encoder ticks
int lastPos1 = 0;  // Last Position in encoder ticks
const byte in11 = 18;
const byte in12 = 17;
const int ticksPerRevolution1 = 210;  // Adjust based on encoder specs

// Motor 2
const byte encA2 = 34;
const byte encB2 = 35;
const byte en2 = 32;
volatile int pos2 = 0;  // Position in encoder ticks
int lastPos2 = 0;  // Last Position in encoder ticks
const byte in21 = 25;
const byte in22 = 33;
const int ticksPerRevolution2 = 210;  // Adjust based on encoder specs

// Robot parameters
const float WHEEL_CIRCUMFERENCE = PI * 0.045; // Diameter = 0.045m
const float WHEEL_BASE = 0.092; // Distance between wheels in meters

// Yaw variables
float yawImu = 0.0; // IMU-based yaw
float lastYawImu = 0;
float yawEncoder = 0.0; // Encoder-based yaw
float fusedYaw = 0.0; // Fused yaw estimate
unsigned long lastTime = 0;
float gyroZOffset = 0.0; // Calibration offset

// Complementary filter weight
const float ALPHA = 0.9; // Weight for IMU (adjust based on testing)

void setup() {
  Serial.begin(115200);

  // Initialize motor/encoder pins
  pinMode(encA1, INPUT_PULLUP);
  pinMode(encB1, INPUT_PULLUP);
  pinMode(encA2, INPUT_PULLUP);
  pinMode(encB2, INPUT_PULLUP);
  pinMode(in11, OUTPUT);
  pinMode(in12, OUTPUT);
  pinMode(in21, OUTPUT);
  pinMode(in22, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encA1), readEncoderChannelA1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), readEncoderChannelA2, RISING);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate gyro (robot MUST be stationary here!)
  calibrateGyro();
  lastTime = millis();
}

void loop() {
  // Update IMU yaw
  updateImuYaw();

  // Update encoder yaw
  float deltaYaw = getEncoderYawAngle();
  yawEncoder += deltaYaw;

  // Fuse yaw estimates
  fusedYaw = ALPHA * yawImu + (1 - ALPHA) * yawEncoder;

  // Print results
  Serial.print("IMU Yaw: ");
  Serial.print(yawImu);
  Serial.print("° | Encoder Yaw: ");
  Serial.print(yawEncoder);
  Serial.print("° | Fused Yaw: ");
  Serial.print(fusedYaw);
  Serial.println("°");

  delay(50); // Adjust loop frequency
}

//-------------------------------------------------------
// Encoder ISRs
//-------------------------------------------------------
void readEncoderChannelA1() {
  int b = digitalRead(encB1);
  pos1 += (b == 0) ? 1 : -1;
}

void readEncoderChannelA2() {
  int b = digitalRead(encB2);
  pos2 += (b == 0) ? 1 : -1;
}

//-------------------------------------------------------
// Yaw Calculations
//-------------------------------------------------------
float getEncoderYawAngle() {
  // Calculate wheel displacements
  float delta_s1 = (pos1 - lastPos1) * WHEEL_CIRCUMFERENCE / ticksPerRevolution1;
  float delta_s2 = (pos2 - lastPos2) * WHEEL_CIRCUMFERENCE / ticksPerRevolution2;

  // Update last positions
  lastPos1 = pos1;
  lastPos2 = pos2;

  // Return delta yaw (degrees)
  return (delta_s2 - delta_s1) / WHEEL_BASE * (180.0 / PI);
}

void updateImuYaw() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Seconds
  lastTime = currentTime;

  // Integrate gyro (subtract bias first)
  float deltaYaw = (g.gyro.z - gyroZOffset) * dt * (180.0 / M_PI); // Rad/s to °/s

  // Check if the change in yaw is significant
  if (fabs(deltaYaw) > 0.1) {
    yawImu += deltaYaw;
  }
}

//-------------------------------------------------------
// Gyro Calibration
//-------------------------------------------------------
void calibrateGyro() {
  Serial.println("Calibrating gyro...");
  delay(1000); // Ensure robot is stationary

  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZOffset += g.gyro.z;
    delay(10);
  }
  gyroZOffset /= samples; // Average bias
  Serial.print("Calibrated gyro bias: ");
  Serial.println(gyroZOffset);
}