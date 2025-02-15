
// Motor 1
const byte encA1 = 4;
const byte encB1 = 23;
const byte en1 = 18;
volatile int pos1 = 0;  // Position in encoder ticks
const byte in11 = 19;
const byte in12 = 16;
const int ticksPerRevolution1 = 200;  // Adjust based on encoder specs

// Motor 2
const byte encA2 = 34;
const byte encB2 = 35;
const byte en2 = 32;
volatile int pos2 = 0;  // Position in encoder ticks
const byte in21 = 25;
const byte in22 = 33;
const int ticksPerRevolution2 = 215;  // Adjust based on encoder specs


void setup() {
  Serial.begin(9600);
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
  digitalWrite(in11, LOW);
  digitalWrite(in12, HIGH);

  digitalWrite(in21, LOW);
  digitalWrite(in22, HIGH);

  // Set initial motor speeds
  analogWrite(en1, 255);
  analogWrite(en2, 255);

  // Attach encoder interrupts
  attachInterrupt(encA1, readEncoderChannelA1, RISING);
  attachInterrupt(encA2, readEncoderChannelA2, RISING);
}

void loop() {
  
  // Serial.print(pos1);
  // Serial.print(", ");
  // Serial.println(pos2);

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


/*
#include <Encoder.h>

// Encoder Pins
const byte encA1 = 2;
const byte encB1 = 4;
Encoder myEnc(encA1, encB1);  // Initialize encoder object

// Motor Pins
const byte in1 = 6;
const byte in2 = 5;

// Potentiometer Pin
const byte potPin = A0;

// Motor Variables
const int threshold = 512;
int potValue = 0;
float motorSpeed1 = 170;
byte motorDirection1 = 0;  // 0->CW, 1->CCW

// Encoder and RPM Calculation
volatile int pos1 = 0;  // Position in encoder ticks
float rpm1 = 0;
long previousTime1 = 0;
const int motorRPMCalculationInterval = 1000;  // ms interval for RPM calculation
const int ticksPerRevolution = 920;           // Encoder ticks per revolution

// Function Declarations
void calculateRPM();
void controlMotor();

void setup() {
  Serial.begin(9600);
  
  // Initialize motor control pins
  pinMode(potPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Start motor with initial speed
  analogWrite(in1, motorSpeed1);
  analogWrite(in2, 0);

  // Reset encoder position
  myEnc.write(0);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime1 >= motorRPMCalculationInterval) {
    // Calculate RPM every motorRPMCalculationInterval
    calculateRPM();
    previousTime1 = currentTime;

    // Display RPM
    Serial.print("RPM: ");
    Serial.println(rpm1);
  }
}

void calculateRPM() {
  int currentPosition = myEnc.read();
  int ticks = currentPosition - pos1;  // Calculate ticks since last check
  pos1 = currentPosition;             // Update last position

  // Calculate RPM
  rpm1 = (ticks * 60.0) / (ticksPerRevolution * (motorRPMCalculationInterval / 1000.0));
}

*/

/*
#include <Encoder.h>

// Encoder Pins
const byte encA1 = 2;
const byte encB1 = 4;
Encoder myEnc(encA1, encB1);  // Initialize encoder object

// Motor Pins
const byte in1 = 6;
const byte in2 = 5;

// Potentiometer Pin
const byte potPin = A0;

// Motor Variables
const int threshold = 512;
int potValue = 0;
float motorSpeed1 = 170;
byte motorDirection1 = 0;  // 0->CW, 1->CCW

// Encoder and RPM Calculation
volatile int pos1 = 0;  // Position in encoder ticks
float rpm1 = 0;
long previousTime1 = 0;
const int motorRPMCalculationInterval = 1000;  // ms interval for RPM calculation
const int ticksPerRevolution = 920;           // Encoder ticks per revolution

// Function Declarations
void calculateRPM();
void controlMotor();

void setup() {
  Serial.begin(9600);
  
  // Initialize motor control pins
  pinMode(potPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Start motor with initial speed
  analogWrite(in1, motorSpeed1);
  analogWrite(in2, 0);

  // Reset encoder position
  myEnc.write(0);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime1 >= motorRPMCalculationInterval) {
    // Calculate RPM every motorRPMCalculationInterval
    calculateRPM();
    previousTime1 = currentTime;

    // Display RPM
    Serial.print("RPM: ");
    Serial.println(rpm1);
  }
}

void calculateRPM() {
  int currentPosition = myEnc.read();
  int ticks = currentPosition - pos1;  // Calculate ticks since last check
  pos1 = currentPosition;             // Update last position

  // Calculate RPM
  rpm1 = (ticks * 60.0) / (ticksPerRevolution * (motorRPMCalculationInterval / 1000.0));
}


/*#include <Encoder.h>

// Encoder Pins
const byte encA1 = 2;
const byte encB1 = 4;
Encoder myEnc(encA1, encB1);  // Initialize encoder object

// Motor Pins
const byte in1 = 6;
const byte in2 = 5;

// Potentiometer Pin
const byte potPin = A0;

// Motor Variables
const int threshold = 512;
int potValue = 0;
float motorSpeed1 = 10;
byte motorDirection1 = 0;  // 0->CW, 1->CCW

// Encoder and RPM Calculation
volatile int pos1 = 0;  // Position in encoder ticks
float rpm1 = 0;
long previousTime1 = 0;
const int motorRPMCalculationInterval = 100;  // ms interval for RPM calculation
const float pulsesPerRevolution = 4;          // Adjust based on encoder specs
volatile int pulses_count = 0;
volatile bool lastStateENCA1;    // Last known state of channel A1
volatile bool lastStateENCB1;    // Last known state of channel B1

// Function Declarations
void calculateRPM(int deltapos, float timeInterval);
void controlMotor();

void setup() {
  Serial.begin(9600);
  // Initialize pins
  pinMode(potPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Reset encoder position
  myEnc.write(0);
  analogWrite(in1, motorSpeed1);
  analogWrite(in2, 0);

  attachInterrupt(digitalPinToInterrupt(encA1), readEncoderChannelA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB1), readEncoderChannelB1, CHANGE);
}

void loop() {
  if(abs(pos1) >= 450){
      Serial.println(abs(pos1));
  	  pos1 = 0;
  }
}

void readEncoderChannelA1() {
  int a = digitalRead(encA1);
  int b = digitalRead(encB1);
  
   
    if (a != b) {
      pos1 += 1;
    } else {
      pos1 -= 1;
    }
    
  
}
void readEncoderChannelB1() {
  int a = digitalRead(encA1);
  int b = digitalRead(encB1);
  
    if (a == b) {
      pos1 += 1;
    } else {
      pos1 -= 1;
    }
    
  
}
*/
/*
#include <Encoder.h>

// Encoder Pins
const byte encA1 = 2;
const byte encB1 = 4;
Encoder myEnc(encA1, encB1); // Initialize encoder object

// Motor Pins
const byte in1 = 6;
const byte in2 = 5;

// Potentiometer Pin
const byte potPin = A0;

// Motor Variables
const int threshold = 512;
int potValue = 0;
float motorSpeed1 = 10;
byte motorDirection1 = 0; // 0->CW, 1->CCW
bool motorStatus1 = true;

// Encoder and RPM Calculation
volatile int pos1 = 0; // Position in encoder ticks
float rpm1 = 0;
long previousTime1 = 0;
const int motorRPMCalculationInterval = 100; // ms interval for RPM calculation
const float pulsesPerRevolution = 4; // Adjust based on encoder specs

// Function Declarations
void calculateRPM(int deltapos, float timeInterval);
void controlMotor();

void setup() {
  Serial.begin(9600);
    // Initialize pins
    pinMode(potPin, INPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // Reset encoder position
    myEnc.write(0);
  	analogWrite(in1, motorSpeed1);
    analogWrite(in2, 0);
}

void loop() {
      // Calculate RPM based on encoder ticks
  int new_pos = myEnc.read();
       //Serial.println((int)abs(new_pos));

  if(abs(new_pos) >= 920){
    Serial.println((int)abs(new_pos));
    myEnc.write(0);
    
    
  }
          
    
}

*/
