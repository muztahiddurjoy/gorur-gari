#include "servo_controller.h"

ServoController::ServoController(int pin) : _pin(pin), _currentAngle(0) {}

void ServoController::begin() {
    // Remove analogWrite line - not needed for servo
    _servo.attach(_pin);
    _servo.write(_currentAngle);
}

void ServoController::setAngle(int angle) {
    // Add bounds checking for servo angle (typically 0-180 degrees)
    _currentAngle = constrain(angle, 0, 180);
    _servo.write(_currentAngle);
}

int ServoController::getAngle() {
    return _currentAngle;
}