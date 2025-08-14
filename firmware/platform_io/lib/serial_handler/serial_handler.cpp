#include <Arduino.h>
#include "serial_handler.h"
#include "button_handler.h"


void serialSetup(int baudRate) {
    Serial.begin(baudRate);
}

void serialLoop() {
    Serial.println(String(buttonState));
}
