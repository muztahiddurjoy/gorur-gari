#include "encoder_controller.h"
#include <Arduino.h>

// Static member definitions
EncoderController* EncoderController::_instance = nullptr;
const int EncoderController::_lookupTable[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

EncoderController::EncoderController(int pinA, int pinB) 
    : _pinA(pinA), _pinB(pinB), _encoderCount(0), _lastEncoderCount(0), 
      _lastTime(0), _velocity(0.0), _sampleTime(100) {
    _instance = this; // Set static instance for interrupt handling
}

void EncoderController::begin() {
    // Set encoder pins as inputs with pullup resistors
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    
    // Attach interrupts for encoder pulses
    attachInterrupt(digitalPinToInterrupt(_pinA), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), updateEncoder, CHANGE);
    
    _lastTime = millis();
    _encoderCount = 0;
    _lastEncoderCount = 0;
}

long EncoderController::getCount() {
    return _encoderCount;
}

void EncoderController::resetCount() {
    noInterrupts();
    _encoderCount = 0;
    _lastEncoderCount = 0;
    interrupts();
}

double EncoderController::getVelocity() {
    return _velocity;
}

void EncoderController::updateOdometry() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - _lastTime;
    
    if (elapsedTime >= _sampleTime) {
        // Calculate velocity (counts per second)
        long countDifference = _encoderCount - _lastEncoderCount;
        _velocity = (double)countDifference / (elapsedTime / 1000.0);
        
        _lastEncoderCount = _encoderCount;
        _lastTime = currentTime;
    }
}

// Static interrupt service routine
void EncoderController::updateEncoder() {
    if (_instance != nullptr) {
        _instance->_updateEncoderInstance();
    }
}

// Instance method for encoder update
void EncoderController::_updateEncoderInstance() {
    static uint8_t enc_val = 0;
    
    // Shift previous state and add new state
    enc_val = (enc_val << 2) | (digitalRead(_pinB) << 1) | digitalRead(_pinA);
    
    // Update encoder count using the lookup table
    _encoderCount += _lookupTable[enc_val & 0b1111];
}