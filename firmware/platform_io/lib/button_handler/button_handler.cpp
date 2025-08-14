#include "button_handler.h"
#include "tft_handler.h"

int buttonState = 0;
int activeButton = -1;

void btnLoop() {
    int reading = digitalRead(activeButton);
    if (reading != buttonState) {
        buttonState = reading;
        tft.drawString("Button "+String(buttonState), 10, 10, 2);
    }
}
void btnSetup(int pin) {
    pinMode(pin, INPUT_PULLUP);
    activeButton = pin;
    Serial.println("Button setup complete");
}