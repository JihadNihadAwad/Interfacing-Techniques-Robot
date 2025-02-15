#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>

// Create sensor objects
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();  // right
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();  // left
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();  // front
Adafruit_MPU6050 mpu;

int rightReading, leftReading, frontReading;

// XSHUT pins for VL53L0X sensors
#define XSHUT1 13
#define XSHUT2 14
#define XSHUT3 27

// Robot parameters
const float WHEEL_CIRCUMFERENCE = PI * 0.045;  // in meters (D = 0.045 m)
const float WHEEL_BASE = 0.092;                // in meters

// PID constants and variables for forward motion
float Kp2 = 0.5;
float Ki2 = 0.2;
float Kd2 = 0.05;

float Kp1 = 0.5;
float Ki1 = 0.2;
float Kd1 = 0.05;

float previousError1 = 0;
float integral1 = 0;
float previousError1Rotate = 0;
float integral1Rotate = 0;

float previousError2 = 0;
float integral2 = 0;
float previousError2Rotate = 0;
float integral2Rotate = 0;

unsigned long previousTimeForward = 0;
unsigned long previousTimeRotate = 0;

const int motorRPMCalculationInterval = 50;  // in ms

int speed1 = 0;
int speed2 = 0;
int speed1Rotate = 0;
int speed2Rotate = 0;

// Yaw and IMU variables
float yawImu = 0.0;
float yawEncoder = 0.0;
float yawEncoderBias = 0;
float yawOffset = 0;
float fusedYaw = 0.0;
unsigned long lastTime = 0;
float gyroZOffset = 0.0;

// Complementary filter weight
const float ALPHA = 0.92;
int desiredRPM = 50;

// Motor 1 pins and encoder variables
const byte encA1 = 4;
const byte encB1 = 23;
const byte en1 = 19;
volatile int pos1 = 0;
int lastPos1 = 0;
const byte in11 = 18;
const byte in12 = 17;
float rpm1 = 0;
const int ticksPerRevolution1 = 210;

// Motor 2 pins and encoder variables
const byte encA2 = 34;
const byte encB2 = 35;
const byte en2 = 32;
volatile int pos2 = 0;
int lastPos2 = 0;
const byte in21 = 25;
const byte in22 = 33;
float rpm2 = 0;
const int ticksPerRevolution2 = 210;

float turningRPM = 50;

// Variables for checking motion and sensor readings
unsigned long lastStuckCheckTime = 0;
int lastPos1Check = 0;
int lastPos2Check = 0;
unsigned long stuckCheckInterval = 500;
int lastFrontLidarReading = 0;

// Global variable for desired angle; if negative, nearest 90° multiple is selected
float desiredAngle = 0;
float lastMpuAngle = 0;

// Function declarations
float getCurrentAngle();
void calculateRPM();
void controlMotor(int, int, int);
void updatePID(int, int, int, int, int);
void updatePIDRotate(int desiredRPM, int sameDirection, int forward, int right);
void readEncoderChannelA1();
void readEncoderChannelA2();
void resetMPU6050();
void brake();
void reset();
void calibrateGyro();

bool isRobotStuck();
bool isRobotTilted();
float getAngleDiff(float, float);
void alignRobotTo90();
void correctWallHit();
void turnLeftRight(bool, int);
void updatePIDForward();
int moveOneCell();


// -------------------
// setup and loop (non-blocking checks maintained)
// -------------------
void setup() {
  Serial.begin(115200);

  // Initialize motor and encoder pins
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

  // Initialize VL53L0X sensors: power on sequentially
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(100);

  digitalWrite(XSHUT1, HIGH);
  delay(150);
  if (!sensor1.begin(0x30)) {
    Serial.println("Failed to initialize sensor 1");
    while (1)
      ;
  }

  digitalWrite(XSHUT2, HIGH);
  delay(150);
  if (!sensor2.begin(0x31)) {
    Serial.println("Failed to initialize sensor 2");
    while (1)
      ;
  }

  digitalWrite(XSHUT3, HIGH);
  delay(150);
  if (!sensor3.begin(0x32)) {
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

  calibrateGyro();
  lastTime = millis();

  // Set initial motor directions (forward)
  digitalWrite(in11, LOW);
  digitalWrite(in12, HIGH);
  digitalWrite(in21, LOW);
  digitalWrite(in22, HIGH);

  // Start front sensor continuous range reading
  sensor3.startRangeContinuous();
}

void loop() {
  // Non-blocking main loop where each function returns whenever a stop condition is met.
  // Run cell movement
  moveOneCell();
  brake();
  delay(2000);
}


// -------------------
// Primary movement function (non-blocking condition checking)
// -------------------
int moveOneCell() {
  int initialPos1 = pos1;
  int initialPos2 = pos2;
  int wallTooCloseThreshold = 70;  // in mm
  int distanceToMove = 200;        // mm target travel distance
  int frontReading = 0;

  int correctedFlag = 0;

  // Wait until first valid front sensor reading (non-blocking)
  while (!sensor3.isRangeComplete()) {
  }
  int initialFrontReading = sensor3.readRangeResult();
  lastFrontLidarReading = initialFrontReading;
  sensor3.startRangeContinuous();

  Serial.print("Initial Front Value: ");
  Serial.println(initialFrontReading);

  // Use non-blocking loop to check both sensor and encoder conditions
  if (lastPos1Check < 99999) {
    lastPos1Check = 99999;
    lastPos2Check = 99999;
  } else {
    lastPos1Check = 0;
    lastPos2Check = 0;
  }
  lastMpuAngle = getCurrentAngle();
  while (true) {
    // Check stop conditions immediately
    if (isRobotStuck()) {
      Serial.println("Robot appears to be stuck!");
      correctWallHit();
      correctedFlag = 1;
    }
    if (isRobotTilted()) {
      Serial.println("Robot appears too tilted (front sensor may be reading a side wall).");
      alignRobotTo90();
      correctedFlag = 1;
    }

    if (sensor3.isRangeComplete()) {

      frontReading = sensor3.readRangeResult();
      sensor3.startRangeContinuous();
      if (frontReading > 0) {
        Serial.print("Front Reading: ");
        Serial.println(frontReading);
        if (correctedFlag) {
          // Modify initial position to account for current distance left
          // Because we moved earlier
          int movedDist = abs(initialFrontReading - frontReading);
          initialPos1 = pos1 - (movedDist * ticksPerRevolution1) / (WHEEL_CIRCUMFERENCE * 10);
          initialPos2 = initialPos1;
          correctedFlag = 0;
        }
      } else
        Serial.println("Front Reading is NULL");

      // Check if the front distance has changed sufficiently or the wall is too close
      
    } else if (!correctedFlag) {
      // If front sensor is not ready, check encoder positions continuously
      int position1 = (abs(pos1 - initialPos1) * (WHEEL_CIRCUMFERENCE * 10)) / ticksPerRevolution1;
      int position2 = (abs(pos2 - initialPos2) * (WHEEL_CIRCUMFERENCE * 10)) / ticksPerRevolution2;
      
    }

    unsigned long currentTime = millis();
    if (currentTime - previousTimeForward >= motorRPMCalculationInterval) {
      previousTimeForward = currentTime;
      calculateRPM();
      updatePIDForward();

      // Apply wall-follow correction if side sensors are available
      if (sensor1.isRangeComplete() && sensor2.isRangeComplete()) {
        int rightReading = sensor1.readRangeResult();
        int leftReading = sensor2.readRangeResult();
        sensor1.startRangeContinuous();
        sensor2.startRangeContinuous();
        int diff = rightReading - leftReading;
        const float diffThreshold = 15.0;
        float recoveryGain = 0.015;
        if (abs(diff) > diffThreshold && abs(diff) < 150) {
          if (diff > 0) {
            float scale = 1.0 + recoveryGain * diff;
            int rpm1_adj = constrain((int)(desiredRPM / scale), 30, desiredRPM + 10);
            int rpm2_adj = constrain((int)(desiredRPM * scale - 5), 40, desiredRPM + 20);
            updatePID(rpm1_adj, rpm2_adj, 1, 1, 0);
          } else {
            float scale = 1.0 + recoveryGain * (-diff);
            int rpm1_adj = constrain((int)(desiredRPM * scale + 10), 40, desiredRPM + 20);
            int rpm2_adj = constrain((int)(desiredRPM / scale), 40, desiredRPM + 10);
            updatePID(rpm1_adj, rpm2_adj, 1, 1, 0);
          }
        } else {
          updatePIDForward();
        }
      } else {
        updatePIDForward();
      }
    }
  }
  return 0;
}


// -------------------
// Turning function
// -------------------
void turnLeftRight(bool turnLeft, int degrees) {
  float currentAngle = getCurrentAngle();
  float targetAngle = turnLeft ? currentAngle - degrees : currentAngle + degrees;

  // Normalize targetAngle if necessary (assuming getAngleDiff handles wrap-around)
  const float TURN_TOLERANCE = 2.0;  // degrees

  while (fabs(getAngleDiff(getCurrentAngle(), targetAngle)) > TURN_TOLERANCE) {
    // Apply PID control for turning.
    if (turnLeft) {
      updatePIDRotate(turningRPM, 0, 1, 0);
    } else {
      updatePIDRotate(turningRPM, 0, 0, 1);
    }
    calculateRPM();
  }
  brake();
  // Update encoder baselines after the turn.
  lastPos1 = pos1;
  lastPos2 = pos2;
}


// -------------------
// Condition Checking Functions
// -------------------
bool isRobotStuck() {
  return false;
  int stuckThresholdEncoder = 2;  // 2cm
  int stuckThresholdLidar = 20;   // 2cm
  unsigned long currentTime = millis();
  if (currentTime - lastStuckCheckTime >= stuckCheckInterval) {
    int deltaPos1 = abs(pos1 - lastPos1Check);
    int deltaPos2 = abs(pos2 - lastPos2Check);
    int minDelta = (deltaPos1 < deltaPos2) ? deltaPos1 : deltaPos2;
    int currentLidar = sensor3.isRangeComplete() ? sensor3.readRangeResult() : lastFrontLidarReading;
    int deltaLidar = abs(currentLidar - lastFrontLidarReading);
    lastStuckCheckTime = currentTime;
    lastPos1Check = pos1;
    lastPos2Check = pos2;
    lastFrontLidarReading = currentLidar;
    if (minDelta < stuckThresholdEncoder && deltaLidar < stuckThresholdLidar)
      return true;
  }
  return false;
}

bool isRobotTilted() {
  float currentAngle = getCurrentAngle();
  float angleDifference = fabs(currentAngle - lastMpuAngle);
  lastMpuAngle = currentAngle;
  const float TILT_THRESHOLD = 30.0;  // Threshold in degrees for detecting a sudden tilt.
  return (angleDifference > TILT_THRESHOLD);
}

// Compute smallest signed angle difference in degrees
float getAngleDiff(float current, float target) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

// Align robot to a specific target angle via non-blocking PID updates
void alignRobotTo90() {
  float currentAngle = getCurrentAngle();
  float targetAngle = desiredAngle;
  if (targetAngle < 0) {
    int multiples[4] = { 0, 90, 180, 270 };
    float minDiff = 360;
    float best = 0;
    for (int i = 0; i < 4; i++) {
      float diff = fabs(currentAngle - multiples[i]);
      if (diff > 180) diff = 360 - diff;
      if (diff < minDiff) {
        minDiff = diff;
        best = multiples[i];
      }
    }
    targetAngle = best;
  }
  Serial.print("Aligning from ");
  Serial.print(currentAngle);
  Serial.print(" to ");
  Serial.println(targetAngle);

  unsigned long lastUpdateTime = millis();
  while (fabs(getAngleDiff(getCurrentAngle(), targetAngle)) > 10) {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= motorRPMCalculationInterval) {
      lastUpdateTime = currentTime;
      calculateRPM();
   
      if (getAngleDiff(getCurrentAngle(), targetAngle) > 0)
        updatePIDRotate(turningRPM, 0, 0, 1);
      else
        updatePIDRotate(turningRPM, 0, 1, 0);
    }
  }
  brake();
  reset();
  Serial.println("Alignment complete.");
}

void correctWallHit() {
  int leftVal = sensor2.isRangeComplete() ? sensor2.readRangeResult() : 0;
  int rightVal = sensor1.isRangeComplete() ? sensor1.readRangeResult() : 0;
  float hitAngle = getCurrentAngle();
  Serial.print("Wall hit at angle: ");
  Serial.println(hitAngle);

  // Back up a short distance
  int initialPos1 = pos1;
  int initialPos2 = pos2;
  int distanceToBack = 40;  // Backup distance in mm.
  while (true) {
    int dist1 = (abs(pos1 - initialPos1) * (WHEEL_CIRCUMFERENCE * 10)) / ticksPerRevolution1;
    int dist2 = (abs(pos2 - initialPos2) * (WHEEL_CIRCUMFERENCE * 10)) / ticksPerRevolution2;
    if (min(dist1, dist2) >= distanceToBack)
      break;
    controlMotor(0, 100, 0);
    controlMotor(1, 100, 0);
  }
  brake();
  Serial.println("Backup complete.");

  // Determine the correction needed.
  int correctionDegrees = 0;
  // If one of the side walls is close (within 10cm), turn away from that wall.
  if (leftVal < rightVal && leftVal < 100) {
    correctionDegrees = 10;  // Turn right by 10 degrees.
    Serial.println("Left wall too close. Correcting: turning right.");
    turnLeftRight(false, correctionDegrees);
  } else if (rightVal < leftVal && rightVal < 100) {
    correctionDegrees = 10;  // Turn left by 10 degrees.
    Serial.println("Right wall too close. Correcting: turning left.");
    turnLeftRight(true, correctionDegrees);
  } else {
    // Both sides are far; scan for gaps by rotating to both sides.
    Serial.println("Both sides far. Scanning for gap...");

    // Save original angle.
    float originalAngle = getCurrentAngle();
    float bestLeftAngle = originalAngle;
    int bestLeftDistance = 0;
    float bestRightAngle = originalAngle;
    int bestRightDistance = 0;

    // Scan left: turn left 30 degrees.
    turnLeftRight(true, 30);
    float currentScanAngle = getCurrentAngle();
    int currentDistance = sensor3.isRangeComplete() ? sensor3.readRangeResult() : 0;
    Serial.print("Left scan at angle ");
    Serial.print(currentScanAngle);
    Serial.print(": distance = ");
    Serial.println(currentDistance);
    if (currentDistance > bestLeftDistance) {
      bestLeftDistance = currentDistance;
      bestLeftAngle = currentScanAngle;
    }

    // Return to original orientation (rotate right by 30°)
    turnLeftRight(false, 30);

    // Scan right: turn right 30 degrees.
    turnLeftRight(false, 30);
    currentScanAngle = getCurrentAngle();
    currentDistance = sensor3.isRangeComplete() ? sensor3.readRangeResult() : 0;
    Serial.print("Right scan at angle ");
    Serial.print(currentScanAngle);
    Serial.print(": distance = ");
    Serial.println(currentDistance);
    if (currentDistance > bestRightDistance) {
      bestRightDistance = currentDistance;
      bestRightAngle = currentScanAngle;
    }

    // Return to original orientation if not already there.
    turnLeftRight(true, 30);

    // Determine which side had the best gap.
    if (bestLeftDistance >= bestRightDistance) {
      Serial.print("Gap found on left side at angle ");
      Serial.print(bestLeftAngle);
      Serial.print(" with distance ");
      Serial.println(bestLeftDistance);
      float angleToTurn = getAngleDiff(getCurrentAngle(), bestLeftAngle);
      if (angleToTurn < 0)
        turnLeftRight(true, abs((int)angleToTurn));
      else
        turnLeftRight(false, (int)angleToTurn);
    } else {
      Serial.print("Gap found on right side at angle ");
      Serial.print(bestRightAngle);
      Serial.print(" with distance ");
      Serial.println(bestRightDistance);
      float angleToTurn = getAngleDiff(getCurrentAngle(), bestRightAngle);
      if (angleToTurn < 0)
        turnLeftRight(true, abs((int)angleToTurn));
      else
        turnLeftRight(false, (int)angleToTurn);
    }
  }

  // Move forward 4cm (40mm)
  initialPos1 = pos1;
  initialPos2 = pos2;
  int distanceToMove = 40;  // Forward distance in mm.
  while (true) {
    int dist1 = (abs(pos1 - initialPos1) * (WHEEL_CIRCUMFERENCE * 10)) / ticksPerRevolution1;
    int dist2 = (abs(pos2 - initialPos2) * (WHEEL_CIRCUMFERENCE * 10)) / ticksPerRevolution2;
    if (min(dist1, dist2) >= distanceToMove)
      break;
    controlMotor(0, 100, 1);
    controlMotor(1, 100, 1);
  }
  brake();
  Serial.println("Forward movement complete.");

  // Realign to the original 90 degrees using desiredAngle
  Serial.print("Realigning to ");
  Serial.print(desiredAngle);
  Serial.println(" degrees.");
  alignRobotTo90();
}


// -------------------
// Other supporting functions
// -------------------
void readEncoderChannelA1() {
  int b = digitalRead(encB1);
  if (b == 0)
    pos1 += 1;
  else
    pos1 -= 1;
}

void readEncoderChannelA2() {
  int b = digitalRead(encB2);
  if (b == 0)
    pos2 += 1;
  else
    pos2 -= 1;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro...");
  delay(1000);  // Minimal delay required during initial gyro calibration
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZOffset += g.gyro.z;
  }
  gyroZOffset /= samples;
  Serial.print("Calibrated gyro bias: ");
  Serial.println(gyroZOffset);
}

float getEncoderYawAngle() {
  float delta_s1 = (pos1 - lastPos1) * WHEEL_CIRCUMFERENCE / ticksPerRevolution1;
  float delta_s2 = (pos2 - lastPos2) * WHEEL_CIRCUMFERENCE / ticksPerRevolution2;
  lastPos1 = pos1;
  lastPos2 = pos2;
  return (delta_s2 - delta_s1) / WHEEL_BASE * (180.0 / PI);
}

void updateImuYaw() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  float deltaYaw = (g.gyro.z - gyroZOffset) * dt * (180.0 / M_PI);
  if (fabs(deltaYaw) > 0.1)
    yawImu += deltaYaw;
}

float getCurrentAngle() {
  updateImuYaw();
  float deltaYaw = getEncoderYawAngle();
  yawEncoder += deltaYaw;
  fusedYaw = ALPHA * yawImu + (1 - ALPHA) * yawEncoder;
  return fusedYaw;
}

void resetMPU6050() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(10);  // minimal delay for sensor re-init
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
  int deltaPos1 = currentPosition1 - lastPos1;
  int deltaPos2 = currentPosition2 - lastPos2;
  lastPos1 = pos1;
  lastPos2 = pos2;
  float timeInterval = motorRPMCalculationInterval / 1000.0;
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

void updatePID(int desiredRPM1, int desiredRPM2, int sameDirection, int forward, int right) {
  float error1 = (desiredRPM1 - abs(rpm1));
  if (sameDirection) {
    integral1 += error1 * (motorRPMCalculationInterval / 1000.0);
    integral1 = constrain(integral1, -1000, 1000);
    if (abs(error1) < 5) integral1 = 0;
  } else {
    integral1Rotate += error1 * (motorRPMCalculationInterval / 1000.0);
    integral1Rotate = constrain(integral1Rotate, -1000, 1000);
    if (abs(error1) < 5) integral1Rotate = 0;
  }
  float derivative1 = (error1 - previousError1) / (motorRPMCalculationInterval / 1000.0);
  float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
  output1 = constrain(output1, -50, 50);

  float error2 = desiredRPM2 - abs(rpm2);
  if (sameDirection) {
    integral2 += error2 * (motorRPMCalculationInterval / 1000.0);
    integral2 = constrain(integral2, -1000, 1000);
    if (abs(error2) < 5) integral2 = 0;
  } else {
    integral2Rotate += error2 * (motorRPMCalculationInterval / 1000.0);
    integral2Rotate = constrain(integral2Rotate, -1000, 1000);
    if (abs(error2) < 5) integral2Rotate = 0;
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

void updatePIDForward() {
  updatePID(desiredRPM, desiredRPM, 1, 1, 0);
}

void updatePIDRotate(int desiredRPM, int sameDirection, int forward, int right) {
  // PID for Motor 1
  float error1 = (desiredRPM - abs(rpm1));  // Error for Motor 1
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
  float error2 = desiredRPM - abs(rpm2);  // Error for Motor 2
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