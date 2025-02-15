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

void setup() {
  Serial.begin(115200);
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

  // sensor1.setMeasurementTimingBudgetMicroSeconds(20000);
  // sensor1.setVcselPulsePeriod(0, 12);  // Pre-range pulse period
  // sensor1.setVcselPulsePeriod(1, 8);   // Final-range pulse period
  // Sensor 2
  digitalWrite(XSHUT2, HIGH);  // Power on sensor 2
  delay(150);                  // Wait for sensor to initialize
  if (!sensor2.begin(0x31)) {  // Change to unique address
    Serial.println("Failed to initialize sensor 2");
    while (1)
      ;
  }

  // sensor2.setMeasurementTimingBudgetMicroSeconds(20000);
  // sensor2.setVcselPulsePeriod(0, 12);  // Pre-range pulse period
  // sensor2.setVcselPulsePeriod(1, 8);   // Final-range pulse period

  // Sensor 3
  digitalWrite(XSHUT3, HIGH);  // Power on sensor 3
  delay(150);                  // Wait for sensor to initialize
  if (!sensor3.begin(0x32)) {  // Change to unique address
    Serial.println("Failed to initialize sensor 3");
    while (1)
      ;
  }

  // sensor3.setMeasurementTimingBudgetMicroSeconds(20000);
  // sensor3.setVcselPulsePeriod(0, 12);  // Pre-range pulse period
  // sensor3.setVcselPulsePeriod(1, 8);   // Final-range pulse period


  sensor1.startRangeContinuous();
  sensor2.startRangeContinuous();
  sensor3.startRangeContinuous();

  // // Try to initialize!
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }

  Serial.println("All sensors initialized with unique addresses.");
}

//All prints will be commented out
void loop() {
  // Read data from sensor 1
  if (sensor1.isRangeComplete()) {
    Serial.print("Sensor1: ");
    rightReading = sensor1.readRangeResult();
    //if (rightReading <= 70)
      //Serial.println("Wall on the right");
    Serial.println(rightReading);

    sensor1.startRangeContinuous();
  }


  if (sensor2.isRangeComplete()) {
    Serial.print("Sensor2: ");
    leftReading = sensor2.readRangeResult();
    //if (leftReading <= 70)
      //Serial.println("Wall on the left");
    Serial.println(leftReading);


    sensor2.startRangeContinuous();
  }
  //while(!sensor3.isRangeComplete()){Serial.println("Stuck");}
  if (sensor3.isRangeComplete()) {
    Serial.print("Sensor3: ");
    frontReading = sensor3.readRangeResult();
    //if (frontReading <= 120)
      //Serial.println("Wall in front");
    Serial.println(frontReading);

    sensor3.startRangeContinuous();
  }
  // VL53L0X_RangingMeasurementData_t measure;

  // // Take a single measurement
  // sensor3.rangingTest(&measure, false);  // `false` = non-blocking

  // if (measure.RangeStatus != 4) {  // Check if measurement is valid
  //   Serial.print("Front Distance (mm): ");
  //   Serial.println(measure.RangeMilliMeter);
  // } else {
  //   Serial.println("Out of range");
  // }
  // sensor2.rangingTest(&measure, false);  // `false` = non-blocking

  // if (measure.RangeStatus != 4) {  // Check if measurement is valid
  //   Serial.print("Right Distance (mm): ");
  //   Serial.println(measure.RangeMilliMeter);
  // } else {
  //   Serial.println("Out of range");
  // }
  // sensor1.rangingTest(&measure, false);  // `false` = non-blocking

  // if (measure.RangeStatus != 4) {  // Check if measurement is valid
  //   Serial.print("Left Distance (mm): ");
  //   Serial.println(measure.RangeMilliMeter);
  // } else {
  //   Serial.println("Out of range");
  // }
  delay(100);  // Add a small delay
}
