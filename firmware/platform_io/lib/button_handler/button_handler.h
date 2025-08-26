#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>


class ButtonHandler {
private:
    int button1Pin;
    int button2Pin;
    bool button1State;
    bool button2State;
    bool button1PrevState;
    bool button2PrevState;
    unsigned long lastDebounceTime1;
    unsigned long lastDebounceTime2;
    const unsigned long debounceDelay = 50;

public:
    ButtonHandler();
    void begin(int pin1, int pin2 = -1);
    void update();
    bool isButton1Pressed();
    bool isButton2Pressed();
    bool wasButton1Clicked();
    bool wasButton2Clicked();
    int getButtonState(); // Get combined state
};

extern ButtonHandler buttonHandler;

#endif // BUTTON_HANDLER_H