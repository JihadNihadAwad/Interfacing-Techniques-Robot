#include <cstdlib>
#include <climits>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>

#define MAX_DIM 16  // MAX dimension for maze size

//////////////////////////////////////
// FloodFill
//////////////////////////////////////
// Direction vectors for movement (N, E, S, W)
enum Direction { NORTH,
                 EAST,
                 SOUTH,
                 WEST };

// Track current position and direction
int currentRow = 0;
int currentColumn = 0;
int currentDirection = NORTH;

constexpr int dRow[] = { 1, 0, -1, 0 };
constexpr int dCol[] = { 0, 1, 0, -1 };

// Global variables for maze and walls
int maze[MAX_DIM][MAX_DIM] = { 0 };
int walls[MAX_DIM][MAX_DIM][4] = { 0 };

// NOTE: Keep these here since order matters
class Cell {
public:
  int row;
  int col;
  Cell(int row, int col)
    : row(row), col(col) {}
};

// Global goal cells
Cell* goal_cells[4] = {};

// Functions
bool isValid(int row, int col);
void floodFill(Cell* goals[], int num_goal_cells);
void initializeMazeWalls();
void logWalls();
bool moveRobot();
void startExploration(Cell* goals[], int num_goal_cells);
void startRun();

//////////////////////////////////////
// API
// MPU, Lidar and Motor APIs
//////////////////////////////////////

float mpuAngleBeforeTurning = 0;
float lastDesiredReading = 0;
// Functions
int isWallFront();
int isWallRight();
int isWallLeft();

int moveOneCell();
void turnRight();
void turnLeft();
void brake();

float getCurrentAngle();
int hasFinishedTurning();
void reset();
//////////////////////////////////////
// Motor Control
//////////////////////////////////////
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

// PID constants
float Kp = 1.25;  // Proportional gain
float Ki = 0.5;   // Integral gain
float Kd = 0.25;  // Derivative gain

// PID constants for forward/backward motion (Motor 1 and Motor 2 treated similarly for forward/backward)
float Kp1 = 0.5;   // Proportional gain for forward/backward (adjust as needed)
float Ki1 = 0.2;   // Integral gain for forward/backward (adjust as needed)
float Kd1 = 0.05;  // Derivative gain for forward/backward (adjust as needed)

// PID constants for rotation (turning) (Motor 1 and Motor 2 treated differently for rotation)
float Kp2 = 0.5;   // Proportional gain for rotation (adjust as needed)
float Ki2 = 0.2;   // Integral gain for rotation (adjust as needed)
float Kd2 = 0.05;  // Derivative gain for rotation (adjust as needed)

// PID variables for rotation - these are *separate* from the forward/backward integrals and errors
float integral1Rotate = 0;       // Integral term for Motor 1 during rotation
float integral2Rotate = 0;       // Integral term for Motor 2 during rotation
float previousError1Rotate = 0;  // Previous error for Motor 1 during rotation
float previousError2Rotate = 0;  // Previous error for Motor 2 during rotation

// Desired RPM for both motors
float desiredRPM = 40.0;  // Set your desired RPM here
float turningRPM = 0.1;

// PID variables for Motor 1
float previousError1 = 0;
float integral1 = 0;

// PID variables for Motor 2
float previousError2 = 0;
float integral2 = 0;

int speed1 = 0;        // Initialize speed1
int speed2 = 0;        // Initialize speed2
int speed1Rotate = 0;  // Initialize speed1
int speed2Rotate = 0;  // Initialize speed2

long previousTime = 0;
const int motorRPMCalculationInterval = 50;  // Interval for RPM calculation (ms)
int motorSpeed = 255;
// Functions
void calculateRPM();
void controlMotor(int, int, int);
void updatePIDForward();
void updatePID(int desiredRPM1, int desiredRPM2, int sameDirection, int forward, int right);
void readEncoderChannelA1();
void readEncoderChannelA2();
void initialiseMotorPins();

//////////////////////////////////////
// MPU
//////////////////////////////////////
// Functions
void initialiseMPU();
void calibrateGyro();
void resetMPU6050();
void updateImuYaw();
float getEncoderYawAngle();

// Yaw variables
float yawImu = 0.0;  // IMU-based yaw
float lastYawImu = 0;
float yawEncoder = 0.0;  // Encoder-based yaw
float yawEncoderBias = 0.0;
float yawOffset = 0;
float fusedYaw = 0.0;  // Fused yaw estimate
unsigned long lastTime = 0;
float gyroZOffset = 0.0;  // Calibration offset
// Complementary filter weight
const float ALPHA = 0.98;  // Weight for IMU (adjust based on testing)
Adafruit_MPU6050 mpu;

//////////////////////////////////////
// Lidars
//////////////////////////////////////
// Functions
void initialiseLidars();
void testRecoveryForward();
//void readLiders(); // No longer needed

// Create sensor objects
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();  //right
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();  //left
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();  //front
//int rightReading, leftReading, frontReading; // No longer needed, readings are done directly in API functions
// Define XSHUT pins
#define XSHUT1 13
#define XSHUT2 14
#define XSHUT3 27

// Robot parameters
const float WHEEL_CIRCUMFERENCE = PI * 0.045;  // Diameter = 0.045m
const float WHEEL_BASE = 0.09;                 // Distance between wheels in meters
//////////////////////////////////////
// Main
//////////////////////////////////////
void setup() {
  Serial.begin(115200);

  initialiseMotorPins();

  initialiseLidars();

  initialiseMPU();

  initializeMazeWalls();
}

void loop() {
  // start_run starts the algorithm, but note that it has its own loop, so once it exits, our run would be over
  // start_run();
  // while(true) {};

  // API Movement Tests
  // turnRight();
  // turnLeft();
  // moveOneCell();

  // Moves 20cm, stops for 200ms (might not be able to see that), moves 20cm
  startRun();
}

//////////////////////////////////////
// API
//////////////////////////////////////
int isWallFront() {
  while (!sensor3.isRangeComplete()) {};
  int frontReading = sensor3.readRangeResult();
  sensor3.startRangeContinuous();  // Immediately restart for next reading
  if (frontReading <= 60)
    return 1;

  return 0;
}

int isWallRight() {
  while (!sensor1.isRangeComplete()) {};
  int rightReading = sensor1.readRangeResult();
  sensor1.startRangeContinuous();  // Immediately restart for next reading
  if (rightReading <= 60)
    return 1;

  return 0;
}

int isWallLeft() {
  while (!sensor2.isRangeComplete()) {};
  int leftReading = sensor2.readRangeResult();
  sensor2.startRangeContinuous();  // Immediately restart for next reading
  if (leftReading <= 60)
    return 1;

  return 0;
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
  int frontReading = initialFrontReading;

  if (lastDesiredReading != 0) {
    if (lastDesiredReading > initialFrontReading) {
      distanceToMove = distanceToMove - abs(lastDesiredReading - initialFrontReading);
    } else
      distanceToMove = distanceToMove + abs(lastDesiredReading - initialFrontReading);

    lastDesiredReading = initialFrontReading - distanceToMove;
  }
  updatePIDForward();
  Serial.println("Called Move Forward");
  while (true) {  // Loop for continuous movement and checking

    if (sensor3.isRangeComplete()) {
      frontReading = sensor3.readRangeResult();
      sensor3.startRangeContinuous();  // Restart immediately

      Serial.print("Current Front Reading: ");
      Serial.println(frontReading);
      if (abs(initialFrontReading - frontReading) >= distanceToMove || frontReading <= wallTooCloseThreshold) {
        // Check for 20cm difference OR wall too close
        // Stop and return
        Serial.println("Stop condition reached from lidar");
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
    if (currentTime - previousTime >= motorRPMCalculationInterval) {
      previousTime = currentTime;
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
        float recoveryGain = 0.02;

        if (abs(diff) > diffThreshold && abs(diff) < 40.0) {
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

void turnRight() {
  brake();
  delay(400);  // Small delay for stability

  mpuAngleBeforeTurning = getCurrentAngle();  // Record initial angle

  controlMotor(0, 100, 0);  // Control Motor 1, forward
  controlMotor(1, 70, 1);  // Control Motor 2, backward

  unsigned long currentTime = millis();
  while (!hasFinishedTurning()) {
  }
  brake();
  reset();  // Reset again after turning
  delay(400);
}

void turnLeft() {
  brake();
  delay(400);
  mpuAngleBeforeTurning = getCurrentAngle();  //Store the angle before turning

   // Start turning left (opposite directions)
  controlMotor(0, 100, 1);  // Control Motor 1, forward
  controlMotor(1, 70, 0);  // Control Motor 2, backward
  unsigned long currentTime = millis();
  while (!hasFinishedTurning()) {
    
  }
  brake();
  reset();
  delay(400);
}

void brake() {
  controlMotor(0, 0, 1);
  controlMotor(1, 0, 0);
}
float getCurrentAngle() {
  // Update IMU yaw
  updateImuYaw();

  // Update encoder yaw
  float deltaYaw = getEncoderYawAngle();
  yawEncoder += deltaYaw;

  // Fuse yaw estimates
  fusedYaw = ALPHA * yawImu + (1 - ALPHA) * (yawEncoder - yawEncoderBias);
  return fusedYaw;
}

// Checks if the difference between previous angle and current angle is less than 90 degrees
// return false if it is, true if not, direction doesn't matter
int hasFinishedTurning() {
  float currentAngleWhileTurning = getCurrentAngle();
  float difference = abs(mpuAngleBeforeTurning - currentAngleWhileTurning);
  if (difference > 180) {
    // Only case where this happens is if angle was near 0 or 360
    // Example: prev_angle = 45, next angle = 315 or vice versa
    difference = abs(difference - 360);
  }
  // Error includes:
  // 1) MPU reading inaccuracy
  // 2) Extra rotation before robot stops when called to brake, could be a slight delay
  float error = 5;
  if (difference + error < 90.0) {
    return 0;  // false
  }
  return 1;
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

//////////////////////////////////////
// MPU
//////////////////////////////////////
// Functions
void initialiseMPU() {
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
//////////////////////////////////////
// Lidars
//////////////////////////////////////
// Functions
void initialiseLidars() {
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

  sensor1.setMeasurementTimingBudgetMicroSeconds(20000);
  sensor1.setVcselPulsePeriod(0, 12);  // Pre-range pulse period
  sensor1.setVcselPulsePeriod(1, 8);   // Final-range pulse period


  // Sensor 2
  digitalWrite(XSHUT2, HIGH);  // Power on sensor 2
  delay(150);                  // Wait for sensor to initialize
  if (!sensor2.begin(0x31)) {  // Change to unique address
    Serial.println("Failed to initialize sensor 2");
    while (1)
      ;
  }

  sensor2.setMeasurementTimingBudgetMicroSeconds(20000);
  sensor2.setVcselPulsePeriod(0, 12);  // Pre-range pulse period
  sensor2.setVcselPulsePeriod(1, 8);   // Final-range pulse period

  // Sensor 3
  digitalWrite(XSHUT3, HIGH);  // Power on sensor 3
  delay(150);                  // Wait for sensor to initialize
  if (!sensor3.begin(0x32)) {  // Change to unique address
    Serial.println("Failed to initialize sensor 3");
    while (1)
      ;
  }
  sensor3.setMeasurementTimingBudgetMicroSeconds(20000);
  sensor3.setVcselPulsePeriod(0, 12);  // Pre-range pulse period
  sensor3.setVcselPulsePeriod(1, 8);   // Final-range pulse period


  sensor1.startRangeContinuous();
  sensor2.startRangeContinuous();
  sensor3.startRangeContinuous();
}

void testRecoveryForward() {
  while (true) {
    moveOneCell();
    brake();
    Serial.println("Next Iteration will start in 2 seconds");
    delay(2000);  // Short delay between cell moves for manual intervention
  }
}

//////////////////////////////////////
// Motor Control
//////////////////////////////////////
void initialiseMotorPins() {
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

  digitalWrite(in11, LOW);
  digitalWrite(in12, HIGH);

  digitalWrite(in21, LOW);
  digitalWrite(in22, HIGH);

  Serial.println("Motors initialized.");
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

  angle1 = (currentPosition1 % ticksPerRevolution1) * 360.0 / ticksPerRevolution1;
  angle2 = (currentPosition2 % ticksPerRevolution2) * 360.0 / ticksPerRevolution2;
}

inline void setPin(uint8_t pin) {
  if (pin < 32) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1UL << pin));
  } else {
    WRITE_PERI_REG(GPIO_OUT1_W1TS_REG, (1UL << (pin - 32)));
  }
}

inline void clearPin(uint8_t pin) {
  if (pin < 32) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1UL << pin));
  } else {
    WRITE_PERI_REG(GPIO_OUT1_W1TC_REG, (1UL << (pin - 32)));
  }
}

void controlMotor(int motorIndex, int speed, int dir) {
  if (motorIndex == 0) {
    if (dir == 0) {
      // Set motor 1 forward: in11 HIGH, in12 LOW
      setPin(in11);
      clearPin(in12);
    } else {
      // Reverse motor 1: in11 LOW, in12 HIGH
      clearPin(in11);
      setPin(in12);
    }
    analogWrite(en1, speed);
  } else {
    if (dir == 0) {
      // Set motor 2 forward: in21 HIGH, in22 LOW
      setPin(in21);
      clearPin(in22);
    } else {
      // Reverse motor 2: in21 LOW, in22 HIGH
      clearPin(in21);
      setPin(in22);
    }
    analogWrite(en2, speed);
  }
}

// void controlMotor(int motorIndex, int speed, int dir) {
//   if (motorIndex == 0) {
//     if (dir == 0) {
//       digitalWrite(in11, HIGH);
//       digitalWrite(in12, LOW);
//     } else {
//       digitalWrite(in11, LOW);
//       digitalWrite(in12, HIGH);
//     }
//     analogWrite(en1, speed);
//   } else {
//     if (dir == 0) {
//       digitalWrite(in21, HIGH);
//       digitalWrite(in22, LOW);
//     } else {
//       digitalWrite(in21, LOW);
//       digitalWrite(in22, HIGH);
//     }
//     analogWrite(en2, speed);
//   }
// }

void updatePIDForward() {
  //This is the default one
  updatePID(desiredRPM, desiredRPM, 1, 1, 0);
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
    integral1 += error1 * (motorRPMCalculationInterval / 1000.0);  // Integral term. Using integral1 for both cases
    integral1 = constrain(integral1, -1000, 1000);           // Integral windup protection
    // Reset integral for small errors
    if (abs(error1) < 5) integral1 = 0;
  }

  float derivative1 = (error1 - previousError1) / (motorRPMCalculationInterval / 1000.0);  // Derivative term
  float output1 = Kp * error1 + Ki * integral1 + Kd * derivative1;                      // PID output
  output1 = constrain(output1, -50, 50);                                                   // Limit PID output

  // PID for Motor 2
  float error2 = desiredRPM2 - abs(rpm2);  // Error for Motor 2
  if (sameDirection) {
    integral2 += error2 * (motorRPMCalculationInterval / 1000.0);  // Integral term
    integral2 = constrain(integral2, -1000, 1000);                 // Integral windup protection

    // Reset integral for small errors
    if (abs(error2) < 5) integral2 = 0;
  } else {
    integral2 += error2 * (motorRPMCalculationInterval / 1000.0);  // Integral term. Using integral2
    integral2 = constrain(integral2, -1000, 1000);           // Integral windup protection

    // Reset integral for small errors
    if (abs(error2) < 5) integral2 = 0;
  }
  float derivative2 = (error2 - previousError2) / (motorRPMCalculationInterval / 1000.0);  // Derivative term
  float output2 = Kp * error2 + Ki * integral2 + Kd * derivative2;                      // PID output
  output2 = constrain(output2, -50, 50);                                                   // Limit PID output


  if (sameDirection) {
    speed1 = constrain(speed1 + output1, 50, 255);  // Adjust speed within bounds
    speed2 = constrain(speed2 + output2, 50, 255);  // Adjust speed within bounds
    if (forward) {
      controlMotor(0, speed1, 1);  // Control Motor 1, forward
      controlMotor(1, speed2, 1);  // Control Motor 2, forward
    } else {
      controlMotor(0, speed1, 0);  // Control Motor 1, backward
      controlMotor(1, speed2, 0);  // Control Motor 2, backward
    }
    previousError1 = error1;  // Update previous error
    previousError2 = error2;  // Update previous error
  } else {
    //For turning
    int speed1_turn = constrain(speed1 + output1, 50, 255);
    int speed2_turn = constrain(speed2 + output2, 50, 255);

    if (right) {
      controlMotor(0, speed1_turn, 0);  // Control Motor 1, forward
      controlMotor(1, speed2_turn, 0);  // Control Motor 2, backward
    } else {
      controlMotor(0, speed1_turn, 1);  // Control Motor 1, backward
      controlMotor(1, speed2_turn, 1);  // Control Motor 2, forward
    }
    previousError1 = error1;  // Update previous error
    previousError2 = error2;  // Update previous error
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

//////////////////////////////////////
// FloodFill
//////////////////////////////////////
// Queue implementation for BFS
class Queue {
private:
  Cell* data[MAX_DIM * MAX_DIM * 2];
  int front = 0;
  int rear = 0;

public:
  void enqueue(Cell* cell) {
    if (rear < MAX_DIM * MAX_DIM * 2) {
      data[rear++] = cell;
    }
  }

  Cell* dequeue() {
    return isEmpty() ? nullptr : data[front++];
  }

  bool isEmpty() const {
    return front == rear;
  }
};

bool isValid(int row, int col) {
  return row >= 0 && row < MAX_DIM && col >= 0 && col < MAX_DIM;
}

void floodFill(Cell* goals[], int num_goal_cells) {
  int processed[MAX_DIM][MAX_DIM] = {};
  int distance[MAX_DIM][MAX_DIM] = {};

  for (int i = 0; i < MAX_DIM; i++) {
    for (int j = 0; j < MAX_DIM; j++) {
      distance[i][j] = 1;
    }
  }

  for (int i = 0; i < num_goal_cells; i++) {
    distance[goals[i]->row][goals[i]->col] = 0;
    processed[goals[i]->row][goals[i]->col] = 1;
  }

  Queue q;
  for (int i = 0; i < num_goal_cells; i++) {
    q.enqueue(goals[i]);
  }

  int count = 0;
  while (!q.isEmpty()) {
    Cell* current = q.dequeue();
    int current_dist = distance[current->row][current->col];
    count++;

    for (int dir = 0; dir < 4; dir++) {
      int newRow = current->row + dRow[dir];
      int newCol = current->col + dCol[dir];

      if (isValid(newRow, newCol) && walls[current->row][current->col][dir] == 0 && !processed[newRow][newCol]) {

        processed[newRow][newCol] = 1;
        distance[newRow][newCol] = current_dist + 1;
        q.enqueue(new Cell(newRow, newCol));
      }
    }

    if (count > 4) {
      delete current;
    }
  }

  // Update maze distances
  for (int i = 0; i < MAX_DIM; i++) {
    for (int j = 0; j < MAX_DIM; j++) {
      maze[i][j] = distance[i][j];
    }
  }
}

void initializeMazeWalls() {
  // Set NORTH walls for top row
  for (int col = 0; col < MAX_DIM; col++) {
    walls[MAX_DIM - 1][col][NORTH] = 1;
  }

  // Set EAST walls for rightmost column
  for (int row = 0; row < MAX_DIM; row++) {
    walls[row][MAX_DIM - 1][EAST] = 1;
  }

  // Set SOUTH walls for bottom row
  for (int col = 0; col < MAX_DIM; col++) {
    walls[0][col][SOUTH] = 1;
  }

  // Set WEST walls for leftmost column
  for (int row = 0; row < MAX_DIM; row++) {
    walls[row][0][WEST] = 1;
  }
}

void logWalls() {
  if (isWallFront()) {
    walls[currentRow][currentColumn][currentDirection] = 1;
    int opposite = (currentDirection + 2) % 4;
    int nr = currentRow + dRow[currentDirection];
    int nc = currentColumn + dCol[currentDirection];
    if (isValid(nr, nc)) {
      walls[nr][nc][opposite] = 1;
    }
  }

  if (isWallRight()) {
    int rightDir = (currentDirection + 1) % 4;
    walls[currentRow][currentColumn][rightDir] = 1;
    int opposite = (rightDir + 2) % 4;
    int nr = currentRow + dRow[rightDir];
    int nc = currentColumn + dCol[rightDir];
    if (isValid(nr, nc)) {
      walls[nr][nc][opposite] = 1;
    }
  }

  if (isWallLeft()) {
    int leftDir = (currentDirection + 3) % 4;
    walls[currentRow][currentColumn][leftDir] = 1;
    int opposite = (leftDir + 2) % 4;
    int nr = currentRow + dRow[leftDir];
    int nc = currentColumn + dCol[leftDir];
    if (isValid(nr, nc)) {
      walls[nr][nc][opposite] = 1;
    }
  }
}

bool moveRobot() {
  if (moveOneCell()) {
    currentRow += dRow[currentDirection];
    currentColumn += dCol[currentDirection];
    return true;
  }
  return false;
}

void startExploration(Cell* goals[], int num_goal_cells) {
  floodFill(goals, num_goal_cells);

  while (true) {
    logWalls();

    // Check if at goal
    for (int i = 0; i < num_goal_cells; i++) {
      if (currentRow == goals[i]->row && currentColumn == goals[i]->col) {
        return;
      }
    }

    // Movement logic
    int frontWall = walls[currentRow][currentColumn][currentDirection];
    int rightWall = walls[currentRow][currentColumn][(currentDirection + 1) % 4];
    int leftWall = walls[currentRow][currentColumn][(currentDirection + 3) % 4];

    if (frontWall && rightWall && leftWall) {
      turnRight();
      turnRight();
      currentDirection = (currentDirection + 2) % 4;
      floodFill(goals, num_goal_cells);
      continue;
    }

    // Determine best direction
    int bestDir = -1;
    int minDist = INT_MAX;
    const int dirs[] = { currentDirection, (currentDirection + 1) % 4, (currentDirection + 3) % 4 };

    for (int dir : dirs) {
      if (!walls[currentRow][currentColumn][dir]) {
        int nr = currentRow + dRow[dir];
        int nc = currentColumn + dCol[dir];
        if (isValid(nr, nc) && maze[nr][nc] < minDist) {
          minDist = maze[nr][nc];
          bestDir = dir;
        }
      }
    }

    // Re-flood if needed
    if (bestDir == -1) {
      floodFill(goals, num_goal_cells);
      continue;
    }

    // Rotate to direction
    while (currentDirection != bestDir) {
      turnRight();
      currentDirection = (currentDirection + 1) % 4;
    }

    moveRobot();
  }
}

void startRun() {
  currentRow = currentColumn = 0;
  currentDirection = NORTH;
  int center = MAX_DIM / 2 - 1;
  goal_cells[0] = new Cell(center, center);
  goal_cells[1] = new Cell(center, center + 1);
  goal_cells[2] = new Cell(center + 1, center);
  goal_cells[3] = new Cell(center + 1, center + 1);

  startExploration(goal_cells, 4);

  // Cleanup
  for (Cell* cell : goal_cells) {
    delete cell;
  }
}