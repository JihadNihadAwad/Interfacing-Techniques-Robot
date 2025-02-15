#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>

// Create sensor objects
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();  // right sensor
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();  // left sensor
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();  // front sensor
Adafruit_MPU6050 mpu;

// Global sensor readings
int rightReading, leftReading, frontReading;

// Define XSHUT pins
#define XSHUT1 13
#define XSHUT2 14
#define XSHUT3 27

// Robot parameters
const float WHEEL_CIRCUMFERENCE = PI * 0.045;  // Diameter = 0.045m
const float WHEEL_BASE = 0.092;                // Distance between wheels in meters

// PID constants (applied for both routines; tune if needed separately)
float Kp1 = 0.5;   // Proportional gain for Motor 1
float Ki1 = 0.2;   // Integral gain for Motor 1
float Kd1 = 0.05;  // Derivative gain for Motor 1

float Kp2 = 0.5;   // Proportional gain for Motor 2
float Ki2 = 0.2;   // Integral gain for Motor 2
float Kd2 = 0.05;  // Derivative gain for Motor 2

// PID variables for Motor 1 (forward movement)
float previousError1 = 0;
float integral1 = 0;
// PID variables for Motor 2 (forward movement)
float previousError2 = 0;
float integral2 = 0;

// PID variables for turning (rotational)
float previousError1Rotate = 0;
float integral1Rotate = 0;
float previousError2Rotate = 0;
float integral2Rotate = 0;

// Timing variables
long previousTimeForward = 0;
long previousTimeRotate = 0;
const int motorRPMCalculationInterval = 50;  // in milliseconds

// Speed variables (for forward and turn motions)
int speed1 = 0;
int speed2 = 0;
int speed1Rotate = 0;
int speed2Rotate = 0;

// Yaw and IMU variables
float yawImu = 0.0;         // IMU-based yaw
float yawEncoder = 0.0;     // Encoder-based yaw
float yawEncoderBias = 0;
float fusedYaw = 0.0;       // Fused yaw estimate
unsigned long lastTime = 0;
float gyroZOffset = 0.0;    // Gyro calibration offset
const float ALPHA = 0.92;   // Complementary filter weight

// For turn functions
float mpuAngleBeforeTurning;
float turningRPM = 30;

// For forward (moveOneCell)
int desiredRPM = 50;

// Motor 1 configuration
const byte encA1 = 4;
const byte encB1 = 23;
const byte en1 = 19;
volatile int pos1 = 0;  // encoder ticks
int lastPos1 = 0;       // last recorded ticks
const byte in11 = 18;
const byte in12 = 17;
float rpm1 = 0;
const int ticksPerRevolution1 = 210;

// Motor 2 configuration
const byte encA2 = 34;
const byte encB2 = 35;
const byte en2 = 32;
volatile int pos2 = 0;  // encoder ticks
int lastPos2 = 0;       // last recorded ticks
const byte in21 = 25;
const byte in22 = 33;
float rpm2 = 0;
const int ticksPerRevolution2 = 210;

// Function declarations
void calculateRPM();
void controlMotor(int motorIndex, int speed, int dir);
void updatePID_turn(int desiredRPM, int sameDirection, int forward, int right);
void updatePID_forward(int desiredRPM1, int desiredRPM2, int sameDirection, int forward, int right);
void readEncoderChannelA1();
void readEncoderChannelA2();
void calibrateGyro();
void updateImuYaw();
float getEncoderYawAngle();
float getCurrentAngle();
void resetMPU6050();
void brake();
void resetVars();
int moveOneCell();
int hasFinishedTurning();
void turnRight();

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
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

  Serial.println("Motors initialized.");

  Wire.begin();

  // Set XSHUT pins as outputs and turn sensors off initially
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(100);

  // Initialize the sensors one by one

  // Sensor 1
  digitalWrite(XSHUT1, HIGH);
  delay(150);
  if (!sensor1.begin(0x30)) {
    Serial.println("Failed to initialize sensor 1");
    while (1);
  }

  // Sensor 2
  digitalWrite(XSHUT2, HIGH);
  delay(150);
  if (!sensor2.begin(0x31)) {
    Serial.println("Failed to initialize sensor 2");
    while (1);
  }

  // Sensor 3
  digitalWrite(XSHUT3, HIGH);
  delay(150);
  if (!sensor3.begin(0x32)) {
    Serial.println("Failed to initialize sensor 3");
    while (1);
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate gyro (robot MUST be stationary)
  calibrateGyro();
  lastTime = millis();

  // Set initial motor directions
  digitalWrite(in11, LOW);
  digitalWrite(in12, HIGH);
  digitalWrite(in21, LOW);
  digitalWrite(in22, HIGH);

  // Start sensors in continuous mode (for distance reading)
  sensor1.startRangeContinuous();
  sensor2.startRangeContinuous();
  sensor3.startRangeContinuous();

  Serial.println("Setup complete.");
}

void loop() {
  Serial.println("Starting moveOneCell 1");
  moveOneCell();
  Serial.println("Finished moveOneCell 1");

  Serial.println("Starting moveOneCell 2");
  moveOneCell();
  Serial.println("Finished moveOneCell 2");

  Serial.println("Starting moveOneCell 3");
  //moveOneCell();
  Serial.println("Finished moveOneCell 3");

  Serial.println("Turning right");
  turnRight();
  Serial.println("Finished turning right");

  Serial.println("Moving Forward Once");
  moveOneCell();
  Serial.println("Finished moving Forward");

  

  Serial.println("Sequence complete. Waiting 5 seconds before repeating.");
  delay(5000);
}

// PID routine for turning movements (uses one desired RPM)
void updatePID_turn(int desiredRPM, int sameDirection, int forward, int right) {
  // PID for Motor 1
  float error1 = (desiredRPM - abs(rpm1));
  if (sameDirection) {
    integral1 += error1 * (motorRPMCalculationInterval / 1000.0);
    integral1 = constrain(integral1, -1000, 1000);
    if (abs(error1) < 5)
      integral1 = 0;
  } else {
    integral1Rotate += error1 * (motorRPMCalculationInterval / 1000.0);
    integral1Rotate = constrain(integral1Rotate, -1000, 1000);
    if (abs(error1) < 5)
      integral1Rotate = 0;
  }
  float derivative1 = (error1 - previousError1) / (motorRPMCalculationInterval / 1000.0);
  float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
  output1 = constrain(output1, -50, 50);

  // PID for Motor 2
  float error2 = desiredRPM - abs(rpm2);
  if (sameDirection) {
    integral2 += error2 * (motorRPMCalculationInterval / 1000.0);
    integral2 = constrain(integral2, -1000, 1000);
    if (abs(error2) < 5)
      integral2 = 0;
  } else {
    integral2Rotate += error2 * (motorRPMCalculationInterval / 1000.0);
    integral2Rotate = constrain(integral2Rotate, -1000, 1000);
    if (abs(error2) < 5)
      integral2Rotate = 0;
  }
  float derivative2 = (error2 - previousError2) / (motorRPMCalculationInterval / 1000.0);
  float output2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;
  output2 = constrain(output2, -50, 50);

  if (sameDirection) {
    speed1 = constrain(speed1 + output1, 50, 255);
    speed2 = constrain(speed2 + output2, 50, 255);
    if (forward) {
      controlMotor(0, speed1, 1);
      controlMotor(1, speed2, 1);
    } else {
      controlMotor(0, speed1, 0);
      controlMotor(1, speed2, 0);
    }
    previousError1 = error1;
    previousError2 = error2;
  } else {
    speed1Rotate = constrain(speed1Rotate + output1, 50, 255);
    speed2Rotate = constrain(speed2Rotate + output2, 50, 255);
    if (right) {
      controlMotor(0, speed1Rotate, 0);
      controlMotor(1, speed2Rotate, 1);
    } else {
      controlMotor(0, speed1Rotate, 1);
      controlMotor(1, speed2Rotate, 0);
    }
    previousError1Rotate = error1;
    previousError2Rotate = error2;
  }
}

// PID routine for forward movement (allows two different desired speeds)
void updatePID_forward(int desiredRPM1, int desiredRPM2, int sameDirection, int forward, int right) {
  // PID for Motor 1
  float error1 = (desiredRPM1 - abs(rpm1));
  if (sameDirection) {
    integral1 += error1 * (motorRPMCalculationInterval / 1000.0);
    integral1 = constrain(integral1, -1000, 1000);
    if (abs(error1) < 5)
      integral1 = 0;
  } else {
    integral1Rotate += error1 * (motorRPMCalculationInterval / 1000.0);
    integral1Rotate = constrain(integral1Rotate, -1000, 1000);
    if (abs(error1) < 5)
      integral1Rotate = 0;
  }
  float derivative1 = (error1 - previousError1) / (motorRPMCalculationInterval / 1000.0);
  float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
  output1 = constrain(output1, -50, 50);

  // PID for Motor 2
  float error2 = (desiredRPM2 - abs(rpm2));
  if (sameDirection) {
    integral2 += error2 * (motorRPMCalculationInterval / 1000.0);
    integral2 = constrain(integral2, -1000, 1000);
    if (abs(error2) < 5)
      integral2 = 0;
  } else {
    integral2Rotate += error2 * (motorRPMCalculationInterval / 1000.0);
    integral2Rotate = constrain(integral2Rotate, -1000, 1000);
    if (abs(error2) < 5)
      integral2Rotate = 0;
  }
  float derivative2 = (error2 - previousError2) / (motorRPMCalculationInterval / 1000.0);
  float output2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;
  output2 = constrain(output2, -50, 50);

  if (sameDirection) {
    speed1 = constrain(speed1 + output1, 50, 255);
    speed2 = constrain(speed2 + output2, 50, 255);
    if (forward) {
      controlMotor(0, speed1, 1);
      controlMotor(1, speed2, 1);
    } else {
      controlMotor(0, speed1, 0);
      controlMotor(1, speed2, 0);
    }
    previousError1 = error1;
    previousError2 = error2;
  } else {
    speed1Rotate = constrain(speed1Rotate + output1, 50, 255);
    speed2Rotate = constrain(speed2Rotate + output2, 50, 255);
    if (right) {
      controlMotor(0, speed1Rotate, 0);
      controlMotor(1, speed2Rotate, 1);
    } else {
      controlMotor(0, speed1Rotate, 1);
      controlMotor(1, speed2Rotate, 0);
    }
    previousError1Rotate = error1;
    previousError2Rotate = error2;
  }
}

// Encoder interrupt routines
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

// Calculate RPM based on encoder tick difference
void calculateRPM() {
  int currentPosition1 = pos1;
  int currentPosition2 = pos2;
  int deltaPos1 = currentPosition1 - lastPos1;
  int deltaPos2 = currentPosition2 - lastPos2;
  lastPos1 = pos1;
  lastPos2 = pos2;
  float timeInterval = motorRPMCalculationInterval / 1000.0;
  rpm1 = (deltaPos1 / (float)ticksPerRevolution1) / timeInterval * 60.0;
  rpm2 = (deltaPos2 / (float)ticksPerRevolution2) / timeInterval * 60.0;
}

// Function to control a motor (motorIndex: 0 for motor1, 1 for motor2)
// dir: 1 for one direction, 0 for the opposite
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

// Update IMU yaw by integrating gyro data
void updateImuYaw() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  float deltaYaw = (g.gyro.z - gyroZOffset) * dt * (180.0 / PI);
  if (fabs(deltaYaw) > 0.1) {
    yawImu += deltaYaw;
  }
}

// Compute change in yaw based on encoder readings
float getEncoderYawAngle() {
  float delta_s1 = (pos1 - lastPos1) * WHEEL_CIRCUMFERENCE / ticksPerRevolution1;
  float delta_s2 = (pos2 - lastPos2) * WHEEL_CIRCUMFERENCE / ticksPerRevolution2;
  lastPos1 = pos1;
  lastPos2 = pos2;
  return (delta_s2 - delta_s1) / WHEEL_BASE * (180.0 / PI);
}

// Get the current fused yaw based on IMU and encoder feedback
float getCurrentAngle() {
  updateImuYaw();
  float deltaYaw = getEncoderYawAngle();
  yawEncoder += deltaYaw;
  fusedYaw = ALPHA * yawImu + (1 - ALPHA) * yawEncoder;
  return fusedYaw;
}

// Reset globals used for PID and yaw fusion
void resetVars() {
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

// Function to calibrate the gyro; robot must be stationary during this process
void calibrateGyro() {
  Serial.println("Calibrating gyro...");
  delay(1000);
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZOffset += g.gyro.z;
    delay(10);
  }
  gyroZOffset /= samples;
  Serial.print("Calibrated gyro bias: ");
  Serial.println(gyroZOffset);
}

// Reset MPU6050 settings (used before turning)
void resetMPU6050() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(10);
}

// Brake both motors
void brake() {
  digitalWrite(in11, LOW);
  digitalWrite(in12, LOW);
  digitalWrite(in21, LOW);
  digitalWrite(in22, LOW);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
}

// Return 1 if the turning (90°) has finished; 0 otherwise
int hasFinishedTurning() {
  float currentAngle = getCurrentAngle();
  float difference = abs(mpuAngleBeforeTurning - currentAngle);
  if (difference > 180) {
    difference = abs(difference - 360);
  }
  // Add error tolerance (e.g. 0° extra rotation)
  if (difference < 90)
    return 0;
  return 1;
}

// Turn right by approximately 90 degrees
void turnRight() {
  brake();
  resetVars();
  delay(20);
  resetMPU6050();
  mpuAngleBeforeTurning = getCurrentAngle();
  // Start turning right: in our configuration, we set motors to spin in opposite directions.
  updatePID_turn(turningRPM, 0, 1, 1);
  unsigned long currentTime = millis();
  while (!hasFinishedTurning()) {
    currentTime = millis();
    if (currentTime - previousTimeRotate >= motorRPMCalculationInterval) {
      previousTimeRotate = currentTime;
      calculateRPM();
      updatePID_turn(turningRPM, 0, 1, 1);
    }
  }
  brake();
  resetVars();
}

// Move one cell forward until either a distance threshold or a wall is detected.
// Returns 1 when the movement stops.
int moveOneCell() {
  int initialPos1 = pos1;
  int initialPos2 = pos2;
  int wallTooCloseThreshold = 70;
  int distanceToMove = 200;  // in mm
  
  int initialFrontReading = 0;
  while (!sensor3.isRangeComplete()) {
    Serial.println("Stuck in initial Front Read");
  }
  initialFrontReading = sensor3.readRangeResult();
  sensor3.startRangeContinuous();
  Serial.print("Initial Front Value: ");
  Serial.println(initialFrontReading);
  
  // Start moving forward using the forward PID (with possibly different speeds for each wheel)
  updatePID_forward(desiredRPM, desiredRPM, 1, 1, 0);
  Serial.println("Called moveOneCell: moving forward...");
  
  int frontReading = 0;
  while (true) {
    if (sensor3.isRangeComplete()) {
      frontReading = sensor3.readRangeResult();
      sensor3.startRangeContinuous();
      Serial.print("Current Front Reading: ");
      if (frontReading > 0)
        Serial.println(frontReading);
      else
        Serial.println("NULL");
      
      if (abs(initialFrontReading - frontReading) >= distanceToMove || frontReading <= wallTooCloseThreshold) {
        Serial.println("Stop condition reached from lidar");
        if (frontReading <= wallTooCloseThreshold)
          Serial.println("Wall too close threshold was reached");
        brake();
        return 1;
      }
    } else {
      // Fallback: check encoder distance (converted to mm)
      int position1 = (abs(pos1 - initialPos1) / (float)ticksPerRevolution1) * (WHEEL_CIRCUMFERENCE * 1000);
      int position2 = (abs(pos2 - initialPos2) / (float)ticksPerRevolution2) * (WHEEL_CIRCUMFERENCE * 1000);
      //Serial.print("Encoder minimum distance (mm): ");
      Serial.println(min(position1, position2));
      if (min(position1, position2) >= distanceToMove) {
        Serial.println("Stop condition reached from encoder");
        brake();
        return 1;
      }
    }
    
    unsigned long currentTime = millis();
    if (currentTime - previousTimeForward >= motorRPMCalculationInterval) {
      previousTimeForward = currentTime;
      calculateRPM();
      // Wall follow (using side sensors) adjustment
      if (sensor1.isRangeComplete() && sensor2.isRangeComplete()) {
        int rightReading = sensor1.readRangeResult();
        int leftReading = sensor2.readRangeResult();
        sensor1.startRangeContinuous();
        sensor2.startRangeContinuous();
        int diff = rightReading - leftReading;
        const float diffThreshold = 15.0;
        float recoveryGain = 0.015;
        if (abs(diff) > diffThreshold && abs(diff) < 150.0) {
          if (diff > 0) {
            float scale = 1.0 + recoveryGain * diff;
            int rpm1_adj = constrain((int)(desiredRPM / scale), 30, desiredRPM + 10);
            int rpm2_adj = constrain((int)(desiredRPM * scale - 5), 40, desiredRPM + 20);
            updatePID_forward(rpm1_adj, rpm2_adj, 1, 1, 0);
          } else {
            float scale = 1.0 + recoveryGain * (-diff);
            int rpm1_adj = constrain((int)(desiredRPM * scale + 10), 40, desiredRPM + 20);
            int rpm2_adj = constrain((int)(desiredRPM / scale), 40, desiredRPM + 10);
            updatePID_forward(rpm1_adj, rpm2_adj, 1, 1, 0);
          }
        } else {
          updatePID_forward(desiredRPM, desiredRPM, 1, 1, 0);
        }
      } else {
        updatePID_forward(desiredRPM, desiredRPM, 1, 1, 0);
      }
    }
  }
  return 0; // Should not reach here
}