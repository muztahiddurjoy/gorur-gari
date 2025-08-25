#include "motor_controller.h"

// Initialize static members
volatile long MotorController::encoderCountStatic = 0;
const int8_t MotorController::lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Global instance
MotorController motorController;

MotorController::MotorController() {
    // Initialize odometry data
    odometry.encoderCount = 0;
    odometry.lastEncoderCount = 0;
    odometry.lastTime = 0;
}

void MotorController::begin() {
    // Set encoder pins as inputs
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    
    // Motor control pins
    pinMode(MOTOR_LEFT, OUTPUT);
    pinMode(MOTOR_RIGHT, OUTPUT);
    
    // Attach servo
    myServo.attach(SERVO_PIN);
    myServo.write(90);  // Center position
    
    // Attach interrupts for encoder pulses
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoderISR, CHANGE);
    
    odometry.lastTime = millis();
}

void MotorController::update() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - odometry.lastTime;
    
    if (elapsedTime >= odometry.sampleTime) {
        odometry.lastEncoderCount = odometry.encoderCount;
        odometry.lastTime = currentTime;
    }
}

// Interrupt service routine for encoder
void MotorController::updateEncoderISR() {
    static uint8_t enc_val = 0;
    // Shift previous state and add new state
    enc_val = (enc_val << 2) | (digitalRead(ENCODER_B) << 1) | digitalRead(ENCODER_A);
    
    // Update encoder count using the lookup table
    encoderCountStatic += lookup_table[enc_val & 0b1111];
}

void MotorController::controlMotor(int pwm_value) {
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
    long delta = encoderCountStatic - odometry.lastEncoderCount;
    odometry.lastEncoderCount = encoderCountStatic;
    return delta;
}

float MotorController::getDistanceTraveled(float wheelCircumference, int pulsesPerRevolution) {
    long pulses = getEncoderDelta();
    return (wheelCircumference * pulses) / pulsesPerRevolution;
}