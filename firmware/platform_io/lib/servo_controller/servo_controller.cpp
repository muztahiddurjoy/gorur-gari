#include "servo_controller.h"

ServoController::ServoController(int pin) : _pin(pin), _currentAngle(0) {}

void ServoController::begin() {
    _servo.attach(_pin);
    _servo.write(_currentAngle);
}

void ServoController::setAngle(int angle) {
    _currentAngle = angle;
    _servo.write(_currentAngle);
}

int ServoController::getAngle() {
    return _currentAngle;
}
