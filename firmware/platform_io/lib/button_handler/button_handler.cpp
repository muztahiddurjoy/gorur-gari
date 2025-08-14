#include "button_handler.h"

int buttonState = 0;
int activeButton = -1;

void btnLoop() {
    int reading = digitalRead(activeButton);
    if (reading != buttonState) {
        buttonState = reading;
    }
}
void btnSetup(int pin) {
    pinMode(pin, INPUT_PULLUP);
    activeButton = pin;
    Serial.println("Button setup complete");
}