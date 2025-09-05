#include "motor_controller.h"
#include <Arduino.h>

// Initialize static members
volatile long MotorController::encoderCountStatic = 0;
const int8_t MotorController::lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Global instance
MotorController motorController;

// Static variables for ISR
static volatile uint8_t enc_val = 0;
static volatile uint8_t last_enc_state = 0;

MotorController::MotorController() {
    // Initialize odometry data
    odometry.lastEncoderCount = 0;
    odometry.lastTime = 0;
}

void MotorController::begin() {
    Serial.println("Initializing motor controller...");
    
    // Set encoder pins as inputs
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    
    // Motor control pins
    // pinMode(MOTOR_LEFT, OUTPUT);
    // pinMode(MOTOR_RIGHT, OUTPUT);
    
    // Initialize motors to stop
    digitalWrite(MOTOR_LEFT, LOW);
    digitalWrite(MOTOR_RIGHT, LOW);
    
    // Attach servo (if needed)
    // myServo.attach(SERVO_PIN);
    // myServo.write(90);  // Center position
    
    // Read initial encoder state
    last_enc_state = (digitalRead(ENCODER_B) << 1) | digitalRead(ENCODER_A);
    
    // Attach interrupts for encoder pulses - FALLING edge is safer
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoderISR, CHANGE);
    
    odometry.lastTime = millis();
    Serial.println("Motor controller initialized");
}

void MotorController::update() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - odometry.lastTime;
    
    if (elapsedTime >= odometry.sampleTime) {
        odometry.lastEncoderCount = encoderCountStatic;
        odometry.lastTime = currentTime;
    }
}

// Optimized ISR - NO digitalRead calls!
void IRAM_ATTR MotorController::updateEncoderISR() {
    // Read pins directly from GPIO register (much faster)
    uint8_t enc_a = (GPIO.in >> ENCODER_A) & 0x1;
    uint8_t enc_b = (GPIO.in >> ENCODER_B) & 0x1;
    uint8_t current_state = (enc_b << 1) | enc_a;
    
    // Update encoder state
    enc_val = (enc_val << 2) | current_state;
    
    // Update encoder count using the lookup table
    encoderCountStatic += lookup_table[enc_val & 0b1111];
}

void MotorController::controlMotor(int pwm_value) {
    // Constrain PWM value to valid range (-255 to 255)
    Serial.println("Control motor with PWM: " + String(pwm_value));
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

void MotorController::stopMotor() {
    controlMotor(0);
}

void MotorController::moveForward(int speed) {
    speed = constrain(speed, 0, 255);
    controlMotor(speed);
}

void MotorController::moveBackward(int speed) {
    speed = constrain(speed, 0, 255);
    controlMotor(-speed);
}

void MotorController::setServoAngle(int angle) {
    angle = constrain(angle, 0, 180);  // Limit to servo range
    myServo.write(angle);
}

int MotorController::getServoAngle() {
    return myServo.read();
}

long MotorController::getEncoderCount() {
    return encoderCountStatic;
}

long MotorController::getEncoderDelta() {
    long current_count = encoderCountStatic;
    long delta = current_count - odometry.lastEncoderCount;
    odometry.lastEncoderCount = current_count;
    return delta;
}

float MotorController::getDistanceTraveled(float wheelCircumference, int pulsesPerRevolution) {
    long pulses = getEncoderDelta();
    return (wheelCircumference * pulses) / pulsesPerRevolution;
}