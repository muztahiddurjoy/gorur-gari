#include "servo_controller.h"

ServoController::ServoController(int pin) : _pin(pin), _currentAngle(90) {}

void ServoController::begin() {
    // Remove analogWrite line - not needed for servo
     ESP32PWM::allocateTimer(2);
     _servo.setPeriodHertz(50);    // Standard 50hz servo
     _servo.attach(_pin, 1000, 2000);
     _servo.write(90);
}

void ServoController::setAngle(int angle) {
    // Add bounds checking for servo angle (typically 0-180 degrees)
    _currentAngle = constrain(angle, 0, 180);
    _servo.write(_currentAngle);
}

int ServoController::getAngle() {
    return _currentAngle;
}