#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

class ButtonHandler {
public:
    ButtonHandler(int pin);
    void begin();
    void update();
    bool isPressed();
    bool wasClicked();
    bool wasReleased();
    unsigned long getPressedTime();
    
private:
    int _buttonPin;
    bool _currentState;
    bool _lastState;
    bool _clicked;
    bool _released;
    unsigned long _lastDebounceTime;
    unsigned long _pressedStartTime;
    const unsigned long _debounceDelay = 50;
    
    bool _readButton();
};

#endif // BUTTON_HANDLER_H