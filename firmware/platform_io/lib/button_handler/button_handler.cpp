#include "button_handler.h"

ButtonHandler::ButtonHandler(int pin) 
    : _buttonPin(pin), _currentState(false), _lastState(false), 
      _clicked(false), _released(false), _lastDebounceTime(0), 
      _pressedStartTime(0) {}

void ButtonHandler::begin() {
    pinMode(_buttonPin, INPUT_PULLUP);
    _currentState = _readButton();
    _lastState = _currentState;
}

void ButtonHandler::update() {
    bool reading = _readButton();
    
    // Reset the debouncing timer if the button state changed
    if (reading != _lastState) {
        _lastDebounceTime = millis();
    }
    
    // If enough time has passed since the last state change
    if ((millis() - _lastDebounceTime) > _debounceDelay) {
        // If the button state has actually changed
        if (reading != _currentState) {
            _currentState = reading;
            
            // Button was just pressed (transition from HIGH to LOW with pullup)
            if (!_currentState && _lastState) {
                _clicked = true;
                _pressedStartTime = millis();
            }
            // Button was just released (transition from LOW to HIGH with pullup)
            else if (_currentState && !_lastState) {
                _released = true;
            }
        }
    }
    
    _lastState = reading;
}

bool ButtonHandler::isPressed() {
    return !_currentState; // Inverted because of INPUT_PULLUP
}

bool ButtonHandler::wasClicked() {
    if (_clicked) {
        _clicked = false; // Reset the flag after reading
        return true;
    }
    return false;
}

bool ButtonHandler::wasReleased() {
    if (_released) {
        _released = false; // Reset the flag after reading
        return true;
    }
    return false;
}

unsigned long ButtonHandler::getPressedTime() {
    if (isPressed() && _pressedStartTime > 0) {
        return millis() - _pressedStartTime;
    }
    return 0;
}

bool ButtonHandler::_readButton() {
    return digitalRead(_buttonPin);
}