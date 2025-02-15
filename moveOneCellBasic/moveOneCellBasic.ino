#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>

// Create sensor objects
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();  //right
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();  //left
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();  //front
Adafruit_MPU6050 mpu;
int rightReading, leftReading, frontReading;
// Define XSHUT pins
#define XSHUT1 13
#define XSHUT2 14
#define XSHUT3 27

// Robot parameters
const float WHEEL_CIRCUMFERENCE = PI * 0.045;  // Diameter = 0.045m
const float WHEEL_BASE = 0.092;                // Distance between wheels in meters
// PID constants
float Kp2 = 0.5;   // Proportional gain
float Ki2 = 0.2;   // Integral gain
float Kd2 = 0.05;  // Derivative gain

float Kp1 = 0.5;   // Proportional gain
float Ki1 = 0.2;   // Integral gain
float Kd1 = 0.05;  // Derivative gain
// PID variables for Motor 1
float previousError1 = 0;
float integral1 = 0;
float previousError1Rotate = 0;
float integral1Rotate = 0;

// PID variables for Motor 2
float previousError2 = 0;
float integral2 = 0;
float previousError2Rotate = 0;
float integral2Rotate = 0;

long previousTimeForward = 0;
long previousTimeRotate = 0;

const int motorRPMCalculationInterval = 50;  // Reduced interval for better responsiveness (ms)

int speed1 = 0;        // Initialize speed1
int speed2 = 0;        // Initialize speed2
int speed1Rotate = 0;  // Initialize speed1
int speed2Rotate = 0;  // Initialize speed2
// Yaw variables
float yawImu = 0.0;  // IMU-based yaw
float lastYawImu = 0;
float yawEncoder = 0.0;  // Encoder-based yaw
float yawEncoderBias = 0;
float yawOffset = 0;
float fusedYaw = 0.0;  // Fused yaw estimate
unsigned long lastTime = 0;
float gyroZOffset = 0.0;  // Calibration offset

// Complementary filter weight
const float ALPHA = 0.92;  // Weight for IMU (adjust based on testing)
int desiredRPM = 50;
// Motor 1
const byte encA1 = 4;
const byte encB1 = 23;
const byte en1 = 19;
volatile int pos1 = 0;  // Position in encoder ticks
int lastPos1 = 0;       // Last Position in encoder ticks
const byte in11 = 18;
const byte in12 = 17;
float angle1 = 0;
float rpm1 = 0;
const int ticksPerRevolution1 = 210;  // Adjust based on encoder specs

// Motor 2
const byte encA2 = 34;
const byte encB2 = 35;
const byte en2 = 32;
volatile int pos2 = 0;  // Position in encoder ticks
int lastPos2 = 0;       // Last Position in encoder ticks
const byte in21 = 25;
const byte in22 = 33;
float angle2 = 0;
float rpm2 = 0;
const int ticksPerRevolution2 = 210;  // Adjust based on encoder specs

float turningRPM = 30;


// Function declarations
float getCurrentAngle();
void calculateRPM();
void controlMotor(int, int, int);
void updatePID(int, int, int, int, int);
void readEncoderChannelA1();
void readEncoderChannelA2();
void resetMPU6050();
void brake();
void reset();

void setup() {
  Serial.begin(115200);

  // Initialize pins
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
  attachInterrupt(encA1, readEncoderChannelA1, RISING);
  attachInterrupt(encA2, readEncoderChannelA2, RISING);



  Serial.println("Motors initialized.");

  Wire.begin();

  // Set XSHUT pins as outputs
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);

  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);

  delay(100);


  // Initialize the sensors one by one

  // Sensor 1
  digitalWrite(XSHUT1, HIGH);  // Power on sensor 1
  delay(150);                  // Increased delay
  if (!sensor1.begin(0x30)) {  // Change to unique address
    Serial.println("Failed to initialize sensor 1");
    while (1)
      ;
  }


  // Sensor 2
  digitalWrite(XSHUT2, HIGH);  // Power on sensor 2
  delay(150);                  // Wait for sensor to initialize
  if (!sensor2.begin(0x31)) {  // Change to unique address
    Serial.println("Failed to initialize sensor 2");
    while (1)
      ;
  }



  // Sensor 3
  digitalWrite(XSHUT3, HIGH);  // Power on sensor 3
  delay(150);                  // Wait for sensor to initialize
  if (!sensor3.begin(0x32)) {  // Change to unique address
    Serial.println("Failed to initialize sensor 3");
    while (1)
      ;
  }



  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate gyro (robot MUST be stationary here!)
  calibrateGyro();
  lastTime = millis();

  // Set initial motor directions
  digitalWrite(in11, LOW);
  digitalWrite(in12, HIGH);

  digitalWrite(in21, LOW);
  digitalWrite(in22, HIGH);

  sensor1.startRangeContinuous();
  sensor2.startRangeContinuous();
  sensor3.startRangeContinuous();
}

void loop() {
  Serial.println("Entered iteration");
  moveOneCell();
  brake();
  Serial.println("Finished iteration. Waiting 2 seconds for next time.");
  delay(2000);
}

void updatePIDForward() {
  //This is the default one
  updatePID(desiredRPM, desiredRPM, 1, 1, 0);
}
 
int moveOneCell() {
  // Initial encoder positions at the start
  int initialPos1 = pos1;
  int initialPos2 = pos2;

  int wallTooCloseThreshold = 70;
  int distanceToMove = 200;
  volatile int position1, position2;  //For checkPosition

  // Read initial front distance.  Use a loop.
  int initialFrontReading = 0;
  while (!sensor3.isRangeComplete()) {
    Serial.println("Stuck in initial Front Read");
  }
  initialFrontReading = sensor3.readRangeResult();
  sensor3.startRangeContinuous();

  Serial.print("Initial Front Value: ");
  Serial.print(initialFrontReading);
  int frontReading = 0;

  updatePIDForward();
  Serial.println("Called Move Forward");
  while (true) {  // Loop for continuous movement and checking

    if (sensor3.isRangeComplete()) {
      frontReading = sensor3.readRangeResult();
      sensor3.startRangeContinuous();  // Restart immediately

      Serial.print("Current Front Reading: ");
      if (frontReading > 0)
        Serial.println(frontReading);
      else
        Serial.println("NULL");

      if (abs(initialFrontReading - frontReading) >= distanceToMove || frontReading <= wallTooCloseThreshold) {
        // Check for 20cm difference OR wall too close
        // Stop and return
        Serial.println("Stop condition reached from lidar");
        if (frontReading <= wallTooCloseThreshold)
          Serial.println("Wall too close threshold was reached");
        return 1;
      }
    } else {
      // Front sensor is NOT ready
      // Check position with encoder
      position1 = (abs(pos1 - initialPos1) / (float)ticksPerRevolution1) * (WHEEL_CIRCUMFERENCE * 10);  //Converted to mm
      position2 = (abs(pos2 - initialPos2) / (float)ticksPerRevolution2) * (WHEEL_CIRCUMFERENCE * 10);

      Serial.print("Encoder minimum pos: ");
      Serial.println(min(position1, position2));
      // takes smaller of the two
      if (min(position1, position2) >= distanceToMove) {  //Also check for 20cm (200mm) using encoders
        Serial.println("Stop condition reached from encoder");
        return 1;
      }
    }

    unsigned long currentTime = millis();
    if (currentTime - previousTimeForward >= motorRPMCalculationInterval) {
      previousTimeForward = currentTime;
      calculateRPM();

      updatePIDForward();
      //Wall follow
      if (sensor1.isRangeComplete() && sensor2.isRangeComplete()) {
        int rightReading = sensor1.readRangeResult();
        int leftReading = sensor2.readRangeResult();
        sensor1.startRangeContinuous();
        sensor2.startRangeContinuous();
        int diff = rightReading - leftReading;  // positive means left is closer
        const float diffThreshold = 15.0;
        float recoveryGain = 0.015;

        if (abs(diff) > diffThreshold && abs(diff) < 150.0) {
          if (diff > 0) {
            // Left is closer
            float scale = 1.0 + recoveryGain * diff;
            int rpm1_adj = constrain((int)(desiredRPM / scale), 30, desiredRPM + 10);
            int rpm2_adj = constrain((int)(desiredRPM * scale - 5), 40, desiredRPM + 20);
            updatePID(rpm1_adj, rpm2_adj, 1, 1, 0);  // Adjusted RPMs
          } else {
            // Right is closer
            float scale = 1.0 + recoveryGain * (-diff);
            int rpm1_adj = constrain((int)(desiredRPM * scale + 10), 40, desiredRPM + 20);
            int rpm2_adj = constrain((int)(desiredRPM / scale), 40, desiredRPM + 10);
            updatePID(rpm1_adj, rpm2_adj, 1, 1, 0);  // Adjusted RPMs
          }
        } else {
          // If within threshold, use regular PID.
          updatePIDForward();
        }
      } else {
        // Regular PID
        updatePIDForward();
      }
    }
  }

  return 0;  // Should not get here
}

void updatePID(int desiredRPM1, int desiredRPM2, int sameDirection, int forward, int right) {
  // PID for Motor 1
  float error1 = (desiredRPM1 - abs(rpm1));  // Error for Motor 1
  if (sameDirection) {
    integral1 += error1 * (motorRPMCalculationInterval / 1000.0);  // Integral term
    integral1 = constrain(integral1, -1000, 1000);                 // Integral windup protection
    // Reset integral for small errors
    if (abs(error1) < 5) integral1 = 0;

  } else {
    integral1Rotate += error1 * (motorRPMCalculationInterval / 1000.0);  // Integral term
    integral1Rotate = constrain(integral1Rotate, -1000, 1000);           // Integral windup protection
    // Reset integral for small errors
    if (abs(error1) < 5) integral1Rotate = 0;
  }

  float derivative1 = (error1 - previousError1) / (motorRPMCalculationInterval / 1000.0);  // Derivative term
  float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;                      // PID output
  output1 = constrain(output1, -50, 50);                                                   // Limit PID output

  //speed1 = constrain(speed1 + output1, 50, 255);  // Adjust speed within bounds

  // PID for Motor 2
  float error2 = desiredRPM2 - abs(rpm2);  // Error for Motor 2
  if (sameDirection) {
    integral2 += error2 * (motorRPMCalculationInterval / 1000.0);  // Integral term
    integral2 = constrain(integral2, -1000, 1000);                 // Integral windup protection

    // Reset integral for small errors
    if (abs(error2) < 5) integral2 = 0;
  } else {
    integral2Rotate += error2 * (motorRPMCalculationInterval / 1000.0);  // Integral term
    integral2Rotate = constrain(integral2Rotate, -1000, 1000);           // Integral windup protection

    // Reset integral for small errors
    if (abs(error2) < 5) integral2Rotate = 0;
  }
  float derivative2 = (error2 - previousError2) / (motorRPMCalculationInterval / 1000.0);  // Derivative term
  float output2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;                      // PID output
  output2 = constrain(output2, -50, 50);                                                   // Limit PID output

  //speed2 = constrain(speed2 + output2, 50, 255);  // Adjust speed within bounds

  if (sameDirection) {
    speed1 = constrain(speed1 + output1, 50, 255);  // Adjust speed within bounds
    speed2 = constrain(speed2 + output2, 50, 255);  // Adjust speed within bounds
    if (forward) {
      controlMotor(0, speed1, 1);  // Control Motor 1
      controlMotor(1, speed2, 1);  // Control Motor 2
    } else {
      controlMotor(0, speed1, 0);  // Control Motor 1
      controlMotor(1, speed2, 0);  // Control Motor 2
    }
    previousError1 = error1;  // Update previous error
    previousError2 = error2;  // Update previous error
  } else {
    speed1Rotate = constrain(speed1Rotate + output1, 50, 255);  // Adjust speed within bounds
    speed2Rotate = constrain(speed2Rotate + output2, 50, 255);  // Adjust speed within bounds
    if (right) {
      controlMotor(0, speed1Rotate, 0);  // Control Motor 1
      controlMotor(1, speed2Rotate, 1);  // Control Motor 2
    } else {
      controlMotor(0, speed1Rotate, 1);  // Control Motor 1
      controlMotor(1, speed2Rotate, 0);  // Control Motor 2
    }
    previousError1Rotate = error1;  // Update previous error
    previousError2Rotate = error2;  // Update previous error
  }
}
void readEncoderChannelA1() {
  int b = digitalRead(encB1);
  if (b == 0) {
    pos1 += 1;
  } else {
    pos1 -= 1;
  }
}

void readEncoderChannelA2() {
  int b = digitalRead(encB2);
  if (b == 0) {
    pos2 += 1;
  } else {
    pos2 -= 1;
  }
}

void calibrateGyro() {
  Serial.println("Calibrating gyro...");
  delay(1000);  // Ensure robot is stationary

  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZOffset += g.gyro.z;
    delay(10);
  }
  gyroZOffset /= samples;  // Average bias
  Serial.print("Calibrated gyro bias: ");
  Serial.println(gyroZOffset);
}
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
  float dt = (currentTime - lastTime) / 1000.0;  // Seconds
  lastTime = currentTime;

  // Integrate gyro (subtract bias first)
  float deltaYaw = (g.gyro.z - gyroZOffset) * dt * (180.0 / M_PI);  // Rad/s to °/s

  // Check if the change in yaw is significant
  if (fabs(deltaYaw) > 0.1) {
    yawImu += deltaYaw;
  }
}

void resetMPU6050() {
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(10);
}
void brake() {
  digitalWrite(in11, LOW);
  digitalWrite(in12, LOW);
  digitalWrite(in21, LOW);
  digitalWrite(in22, LOW);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
}
void calculateRPM() {
  int currentPosition1 = pos1;
  int currentPosition2 = pos2;

  int deltaPos1 = currentPosition1 - lastPos1;  // Calculate ticks since last check
  int deltaPos2 = currentPosition2 - lastPos2;  // Calculate ticks since last check
  lastPos1 = pos1;                              // Update last position
  lastPos2 = pos2;                              // Update last position

  // Calculate RPM
  float timeInterval = motorRPMCalculationInterval / 1000.0;  // Time interval in seconds

  rpm1 = (deltaPos1 / (float)ticksPerRevolution1) / timeInterval * 60.0;
  rpm2 = (deltaPos2 / (float)ticksPerRevolution2) / timeInterval * 60.0;
}

void controlMotor(int motorIndex, int speed, int dir) {
  if (motorIndex == 0) {
    if (dir == 0) {
      digitalWrite(in11, HIGH);
      digitalWrite(in12, LOW);
    } else {
      digitalWrite(in11, LOW);
      digitalWrite(in12, HIGH);
    }
    analogWrite(en1, speed);
  } else {
    if (dir == 0) {
      digitalWrite(in21, HIGH);
      digitalWrite(in22, LOW);
    } else {
      digitalWrite(in21, LOW);
      digitalWrite(in22, HIGH);
    }
    analogWrite(en2, speed);
  }
}
void reset() {
  yawImu = 0;
  yawEncoderBias = yawEncoder;
  fusedYaw = 0;
  speed1 = 0;
  speed1Rotate = 0;
  speed2 = 0;
  speed2Rotate = 0;
  integral1 = 0;
  integral1Rotate = 0;
  integral2 = 0;
  integral2Rotate = 0;
  previousError1 = 0;
  previousError1Rotate = 0;
  previousError2 = 0;
  previousError2Rotate = 0;
}
float getCurrentAngle() {
  // Update IMU yaw
  updateImuYaw();

  // Update encoder yaw
  float deltaYaw = getEncoderYawAngle();
  yawEncoder += deltaYaw;

  // Fuse yaw estimates
  fusedYaw = ALPHA * yawImu + (1 - ALPHA) * yawEncoder;
  return fusedYaw;
}
