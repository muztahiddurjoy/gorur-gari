#include "motor_controller.h"
#include <Arduino.h>

// Static timer management - prevents timer conflicts with servo controllers
int MotorController::_nextMotorTimer = 1; // Start from timer 1 (timer 0 reserved for servos)

MotorController::MotorController(int pinLeft, int pinRight) 
    : _pinLeft(pinLeft), _pinRight(pinRight) {
    
    // Assign unique timer to avoid conflicts with servo controllers
    _motorTimer = _nextMotorTimer++;
    
    // Wrap around if we run out of timers (skip timer 0 for servos)
    if (_nextMotorTimer >= 8) _nextMotorTimer = 1;
}

void MotorController::begin() {
    Serial.println("Initializing motor controller...");
    
    // Setup motor control pins
    _setupMotorPWM();
    
    // Initialize motors to stop
    stopMotor();
    
    Serial.println("Motor controller initialized");
}

void MotorController::_setupMotorPWM() {
    // Configure PWM channels for motor control
    // Skip timer 0 to avoid conflicts with servo controllers
    ledcSetup(_motorTimer * 2, 1000, 8);     // Channel for left motor, 1kHz, 8-bit resolution
    ledcSetup(_motorTimer * 2 + 1, 1000, 8); // Channel for right motor, 1kHz, 8-bit resolution
    
    // Attach pins to PWM channels
    ledcAttachPin(_pinLeft, _motorTimer * 2);
    ledcAttachPin(_pinRight, _motorTimer * 2 + 1);
}

void MotorController::_cleanupMotorPWM() {
    // Detach pins from PWM channels
    ledcDetachPin(_pinLeft);
    ledcDetachPin(_pinRight);
}

void MotorController::controlMotor(int pwmValue) {
    // Constrain PWM value to valid range (-255 to 255)
    pwmValue = constrain(pwmValue, -255, 255);
    
    if (pwmValue > 0) { 
        // Forward
        ledcWrite(_motorTimer * 2, 0);        // Left motor off
        ledcWrite(_motorTimer * 2 + 1, pwmValue); // Right motor forward
    }
    else if (pwmValue < 0) { 
        // Backward
        ledcWrite(_motorTimer * 2, abs(pwmValue)); // Left motor backward
        ledcWrite(_motorTimer * 2 + 1, 0);         // Right motor off
    }
    else { 
        // Stop
        ledcWrite(_motorTimer * 2, 0);     // Left motor off
        ledcWrite(_motorTimer * 2 + 1, 0); // Right motor off
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