#include "button_handler.h"

ButtonHandler buttonHandler;

ButtonHandler::ButtonHandler() 
    : button1Pin(-1), button2Pin(-1), 
      button1State(false), button2State(false),
      button1PrevState(false), button2PrevState(false),
      lastDebounceTime1(0), lastDebounceTime2(0) {
}

void ButtonHandler::begin(int pin1, int pin2) {
    button1Pin = pin1;
    button2Pin = pin2;
    
    pinMode(button1Pin, INPUT_PULLUP);
    if (button2Pin != -1) {
        pinMode(button2Pin, INPUT_PULLUP);
    }
}

void ButtonHandler::update() {
    unsigned long currentTime = millis();
    
    // Read button 1 with debouncing
    bool reading1 = digitalRead(button1Pin) == LOW;
    if (reading1 != button1PrevState) {
        lastDebounceTime1 = currentTime;
    }
    
    if ((currentTime - lastDebounceTime1) > debounceDelay) {
        if (reading1 != button1State) {
            button1State = reading1;
        }
    }
    button1PrevState = reading1;
    
    // Read button 2 with debouncing (if configured)
    if (button2Pin != -1) {
        bool reading2 = digitalRead(button2Pin) == LOW;
        if (reading2 != button2PrevState) {
            lastDebounceTime2 = currentTime;
        }
        
        if ((currentTime - lastDebounceTime2) > debounceDelay) {
            if (reading2 != button2State) {
                button2State = reading2;
            }
        }
        button2PrevState = reading2;
    }
}

bool ButtonHandler::isButton1Pressed() {
    return button1State;
}

bool ButtonHandler::isButton2Pressed() {
    return button2State;
}

bool ButtonHandler::wasButton1Clicked() {
    bool clicked = button1State && !button1PrevState;
    return clicked;
}

bool ButtonHandler::wasButton2Clicked() {
    bool clicked = button2State && !button2PrevState;
    return clicked;
}

int ButtonHandler::getButtonState() {
    int state = 0;
    if (button1State) state |= 0x01;
    if (button2State) state |= 0x02;
    return state;
}