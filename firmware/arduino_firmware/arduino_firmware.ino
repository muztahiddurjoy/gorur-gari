#include <Arduino.h>
#include <Servo.h>  // Include Servo library

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

#define US1_TRIG 4
#define US1_ECHO 5

#define US2_TRIG 6
#define US2_ECHO 7

#define US3_TRIG 8
#define US3_ECHO 9

#define MOTOR_LEFT 10
#define MOTOR_RIGHT 11
#define SERVO_PIN 12  // Servo pin

#define BTN 13

// Motor direction constants
#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Servo myServo;  // Create servo object

long d1, d2, d3;
int D1, D2, D3;

// Odometry variables
volatile long encoderCount = 0;
long lastEncoderCount = 0;
double duration = 0.0;
int btn_state = 0;

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

  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);

  pinMode(US3_TRIG, OUTPUT);
  pinMode(US3_ECHO, INPUT);
  
  // Motor control pins
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);

  // Button pin
  pinMode(BTN, INPUT_PULLUP);

  // Attach servo
  myServo.attach(SERVO_PIN);
  myServo.write(90);  // Center position

  // Attach interrupts for encoder pulses
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);
  
  Serial.println("Ready to receive commands (M<pwm>,S<angle>)");
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  
  printOdometryData(currentTime);
  updateBtn();
  
  // Read ultrasonic sensors
  D1 = getUSReading(US1_TRIG, US1_ECHO);
  D2 = getUSReading(US2_TRIG, US2_ECHO);
  D3 = getUSReading(US3_TRIG, US3_ECHO);
  
  Serial.print(D1);
  Serial.print(",");
  Serial.print(D2);
  Serial.print(",");
  Serial.print(D3);
  Serial.print(",");
  Serial.println(btn_state);

  // Process serial commands
  processSerialCommands();

  if (elapsedTime >= sampleTime) {
    lastEncoderCount = encoderCount;
    lastTime = currentTime;
  }
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Motor command: M<pwm_value> (e.g., M150 or M-100)
    if (command == 'M') {
      int pwm = Serial.parseInt();
      controlMotor(pwm);
    }
    // Servo command: S<angle> (e.g., S90)
    else if (command == 'S') {
      int angle = Serial.parseInt();
      angle = constrain(angle, 0, 180);  // Limit to servo range
      myServo.write(angle);
    }
    
    // Clear any remaining characters in buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}

// Print all odometry data
void printOdometryData(unsigned long time) {
  Serial.print(encoderCount);
  Serial.print(",");
}

// Interrupt service routine for encoder
void updateEncoder() {
  static uint8_t enc_val = 0;
  
  // Shift previous state and add new state
  enc_val = (enc_val << 2) | (digitalRead(ENCODER_B) << 1) | digitalRead(ENCODER_A);
  
  // Update encoder count using the lookup table
  encoderCount += lookup_table[enc_val & 0b1111];
}

int getUSReading(int trig, int echo) {
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

void updateBtn() {
  btn_state = digitalRead(BTN);
}

void controlMotor(int pwm_value) {
  // Constrain PWM value to valid range (-255 to 255)
  pwm_value = constrain(pwm_value, -255, 255);
  
  if (pwm_value > 0) {    // Forward
    analogWrite(MOTOR_LEFT, 0);
    analogWrite(MOTOR_RIGHT, pwm_value);
  } 
  else if (pwm_value < 0) {  // Backward
    analogWrite(MOTOR_LEFT, abs(pwm_value));
    analogWrite(MOTOR_RIGHT, 0);
  }
  else {  // Stop
    analogWrite(MOTOR_LEFT, 0);
    analogWrite(MOTOR_RIGHT, 0);
  }
}