#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>

// Create sensor objects
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();  //right
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();  //left
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();  //front
// Define XSHUT pins
#define XSHUT1 13
#define XSHUT2 14
#define XSHUT3 27
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
float Kp2 = 0.5;   // Proportional gain
float Ki2 = 0.2;   // Integral gain
float Kd2 = 0.05;  // Derivative gain

float Kp1 = 0.5;   // Proportional gain
float Ki1 = 0.2;   // Integral gain
float Kd1 = 0.05;  // Derivative gain

float desiredRPM = 50;

// PID variables for Motor 1
float previousError1 = 0;
float integral1 = 0;

// PID variables for Motor 2
float previousError2 = 0;
float integral2 = 0;

long previousTime = 0;
const int motorRPMCalculationInterval = 50;  // Reduced interval for better responsiveness (ms)
int motorSpeed = 100;

int speed1 = motorSpeed;  // Initialize speed1
int speed2 = motorSpeed;  // Initialize speed2
// Robot parameters
const float WHEEL_CIRCUMFERENCE = PI * 4.5;  // Diameter = 0.045m
const float WHEEL_BASE = 0.092;              // Distance between wheels in meters
volatile int position1, position2;
volatile int stop;

int initialFrontReading = 0;
int frontReading = 0;
int distanceMoved = 0;
// Function declarations
void calculateRPM();
void controlMotor(int, int, int);
void updatePID();
void readEncoderChannelA1();
void readEncoderChannelA2();
void checkPosition();
void brake();
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

  // Set initial motor directions
  // digitalWrite(in11, LOW);
  // digitalWrite(in12, HIGH);

  // digitalWrite(in21, LOW);
  // digitalWrite(in22, HIGH);

  // // Set initial motor speeds
  // analogWrite(en1, motorSpeed);
  // analogWrite(en2, motorSpeed);

  // Attach encoder interrupts with debouncing
  attachInterrupt(encA1, readEncoderChannelA1, RISING);
  attachInterrupt(encA2, readEncoderChannelA2, RISING);

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


  sensor1.startRangeContinuous();
  sensor2.startRangeContinuous();
  sensor3.startRangeContinuous();

  while (!sensor3.isRangeComplete()) {
    Serial.print("Initial Reading: ");
    initialFrontReading = sensor3.readRangeResult();
    Serial.println(initialFrontReading);
  }
  sensor3.startRangeContinuous();

  Serial.println("Motors initialized.");
  delay(2000);
}

void loop() {
  // Serial.println(position1);
  //   Serial.println(position1);

  // if (stop == 1) {
  //    brake();
  //   while (true){
  //     ;
  //   }
  // }

  if (sensor3.isRangeComplete()) {
    frontReading = sensor3.readRangeResult();

    Serial.print("Front Reading: ");
    Serial.println(frontReading);
    if (abs(initialFrontReading - frontReading) >= 200) {
      brake();
      Serial.println("Lider Stop");
      while (true) {};
    }
    sensor3.startRangeContinuous();
  } else {
    checkPosition();
    if (min(position1, position2) >= 20) {
      brake();
      while (true) {};
    }
  }
  unsigned long currentTime = millis();
  float timeDifference = currentTime - previousTime;

  // Calculate and display RPM
  if (timeDifference >= motorRPMCalculationInterval) {
    previousTime = currentTime;

    calculateRPM();

    // Print to Serial Monitor

    Serial.print(rpm1);
    Serial.print(", ");
    Serial.println(rpm2);

    // Update PID controllers for both motors
    updatePID();
  }
}

void calculateRPM() {
  int currentPosition1 = pos1;
  int currentPosition2 = pos2;

  int deltaPos1 = currentPosition1 - lastPos1;  // Calculate ticks since last check
  int deltaPos2 = currentPosition2 - lastPos2;  // Calculate ticks since last check

  // Ignore large jumps (noise)
  // if (abs(deltaPos1) > 100) deltaPos1 = 0;
  // if (abs(deltaPos2) > 100) deltaPos2 = 0;

  lastPos1 = pos1;  // Update last position
  lastPos2 = pos2;  // Update last position

  // Calculate RPM
  float timeInterval = motorRPMCalculationInterval / 1000.0;  // Time interval in seconds

  rpm1 = (deltaPos1 / (float)ticksPerRevolution1) / timeInterval * 60.0;
  rpm2 = (deltaPos2 / (float)ticksPerRevolution2) / timeInterval * 60.0;

  angle1 = (currentPosition1 % ticksPerRevolution1) * 360.0 / ticksPerRevolution1;
  angle2 = (currentPosition2 % ticksPerRevolution2) * 360.0 / ticksPerRevolution2;
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

void updatePID() {
  // PID for Motor 1
  float error1 = (desiredRPM - abs(rpm1));                       // Error for Motor 1
  integral1 += error1 * (motorRPMCalculationInterval / 1000.0);  // Integral term
  integral1 = constrain(integral1, -1000, 1000);                 // Integral windup protection

  // Reset integral for small errors
  if (abs(error1) < 5) integral1 = 0;

  float derivative1 = (error1 - previousError1) / (motorRPMCalculationInterval / 1000.0);  // Derivative term
  float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;                      // PID output
  output1 = constrain(output1, -50, 50);                                                   // Limit PID output

  speed1 = constrain(speed1 + output1, 50, 255);  // Adjust speed within bounds

  controlMotor(0, speed1, 1);  // Control Motor 1
  previousError1 = error1;     // Update previous error


  // PID for Motor 2
  float error2 = abs(rpm1) - abs(rpm2);                          // Error for Motor 2
  integral2 += error2 * (motorRPMCalculationInterval / 1000.0);  // Integral term
  integral2 = constrain(integral2, -1000, 1000);                 // Integral windup protection

  // Reset integral for small errors
  if (abs(error2) < 5) integral2 = 0;

  float derivative2 = (error2 - previousError2) / (motorRPMCalculationInterval / 1000.0);  // Derivative term
  float output2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;                      // PID output
  output2 = constrain(output2, -50, 50);                                                   // Limit PID output

  speed2 = constrain(speed2 + output2, 50, 255);  // Adjust speed within bounds

  controlMotor(1, speed2, 1);  // Control Motor 2
  previousError2 = error2;     // Update previous error
}

void readEncoderChannelA1() {
  int b = digitalRead(encB1);
  if (b == 0) {
    pos1 += 1;
  } else {
    pos1 -= 1;
  }
  // position1 = (abs(pos1) / (float)ticksPerRevolution1) * (WHEEL_CIRCUMFERENCE);
  // if (position1 >= 18) {
  //   stop = 1;
  // }
}

void readEncoderChannelA2() {
  int b = digitalRead(encB2);
  if (b == 0) {
    pos2 += 1;
  } else {
    pos2 -= 1;
  }
  // position2 = (abs(pos2) / (float)ticksPerRevolution2) * (WHEEL_CIRCUMFERENCE);
  // if (position2 >= 18)
  //   stop = 1;
}
void checkPosition() {
  position1 = (abs(pos1) / (float)ticksPerRevolution1) * (WHEEL_CIRCUMFERENCE);
  position2 = (abs(pos2) / (float)ticksPerRevolution2) * (WHEEL_CIRCUMFERENCE);
  Serial.print("Position1: ");
  Serial.println(abs(position1));
  Serial.print("Position2: ");
  Serial.println(abs(position2));
}
void brake() {
  digitalWrite(in11, LOW);
  digitalWrite(in12, LOW);
  digitalWrite(in21, LOW);
  digitalWrite(in22, LOW);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
}