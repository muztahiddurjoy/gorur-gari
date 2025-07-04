#include <Arduino.h>

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

#define US1_TRIG 4
#define US1_ECHO 5

#define US2_TRIG 6
#define US2_ECHO 7

#define US3_TRIG 8
#define US3_ECHO 9

#define US4_TRIG 10
#define US4_ECHO 11

#define MOTOR_LEFT 10
#define MOTOR_RIGHT 11




long d1,d2,d3,d4;
int D1,D2,D3,D4;

// Constants for JGA25-370 motor
const float WHEEL_RADIUS = 0.0325;       // Wheel radius in meters (65mm diameter)
const float COUNTS_PER_REV = 44.0;       // 11 PPR × 4 (quadrature decoding)
const float METERS_PER_COUNT = (2 * PI * WHEEL_RADIUS) / COUNTS_PER_REV;

// Odometry variables
volatile long encoderCount = 0;
long lastEncoderCount = 0;
float distance = 0.0;            // Total distance traveled (meters)
float x = 0.0;                   // X position in meters
float y = 0.0;                   // Y position in meters
float theta = 0.0;               // Orientation in radians
double duration = 0.0;

// Motion variables
float velocity = 0.0;            // Linear velocity (m/s)
float rpm = 0.0;                 // Motor RPM
unsigned long lastTime = 0;
const int sampleTime = 100;      // Odometry update interval (ms)

// Lookup table for quadrature decoding
const int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void setup() {
  Serial.begin(115200);
  
  // Set encoder pins as inputs
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(US1_TRIG, OUTPUT); 
  pinMode(US1_ECHO, INPUT);

  pinMode(US2_TRIG,OUTPUT);
  pinMode(US2_ECHO,INPUT);

  pinMode(US3_TRIG,OUTPUT);
  pinMode(US3_ECHO,INPUT);
  
  pinMode(US4_TRIG,OUTPUT);
  pinMode(US4_ECHO,INPUT);

  pinMode(MOTOR_LEFT,OUTPUT);
  pinMode(MOTOR_RIGHT,OUTPUT);
  // Attach interrupts for encoder pulses
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);
  
  Serial.println("Time,Position,Velocity,RPM,Distance,X,Y,Theta");
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  
  printOdometryData(currentTime);
  D1 = getUSReading(US1_TRIG,US1_ECHO);
  D2 = getUSReading(US2_TRIG,US2_ECHO);
  D3 = getUSReading(US3_TRIG,US3_ECHO);
  D4 = getUSReading(US4_TRIG,US4_ECHO);
  Serial.print(D1);
  Serial.print(",");
  Serial.print(D2);
  Serial.print(",");
  Serial.println(D3);
  // controlMotor(255);
  

  if (elapsedTime >= sampleTime) {
    // Calculate linear velocity (m/s)
    long deltaCount = encoderCount - lastEncoderCount;
    // velocity = (deltaCount * METERS_PER_COUNT) / (elapsedTime / 1000.0);
    
    // Calculate RPM
    rpm = (deltaCount * 60000.0) / (COUNTS_PER_REV * elapsedTime);
    
    // Update total distance
    distance += deltaCount * METERS_PER_COUNT;
    
    // Update position (simple linear motion - replace with your kinematic model)
    updatePosition(deltaCount * METERS_PER_COUNT);
    
    // Update last values
    lastEncoderCount = encoderCount;
    lastTime = currentTime;
    
  }
  
  // Serial.print(",");
  // Serial.println(D4);
  
  
}

// Update robot position (simplified for single wheel)
void updatePosition(float deltaDistance) {
  // For a differential drive robot, you would use both wheel distances
  // This is a simplified model for demonstration with one wheel
  x += deltaDistance * cos(theta);
  y += deltaDistance * sin(theta);
  
  // For a real robot, you would update theta based on wheel differences
  // theta += (deltaRight - deltaLeft) / WHEEL_BASE;
}

// Print all odometry data
void printOdometryData(unsigned long time) {
  Serial.print(encoderCount);
  Serial.print(",");
  // Serial.print(velocity);    // 3 decimal places
  // Serial.print(",");
 
}

// Interrupt service routine for encoder
void updateEncoder() {
  static uint8_t enc_val = 0;
  
  // Shift previous state and add new state
  enc_val = (enc_val << 2) | (digitalRead(ENCODER_B) << 1) | digitalRead(ENCODER_A);
  
  // Update encoder count using the lookup table
  encoderCount += lookup_table[enc_val & 0b1111];
}

int getUSReading(int trig, int echo){
// Clears the trigPin
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo, HIGH);
  // Calculating the distance
  return duration * 0.034 / 2;
}


void controlMotor(int pwm_value){
  if(pwm_value>0){
    
    analogWrite(MOTOR_LEFT,0);
    analogWrite(MOTOR_RIGHT,pwm_value);
  }
  else if(pwm_value<0){
    analogWrite(MOTOR_LEFT,pwm_value);
    analogWrite(MOTOR_RIGHT,0);
  }
  
}