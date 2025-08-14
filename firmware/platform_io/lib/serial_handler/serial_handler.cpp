#include <Arduino.h>
#include "serial_handler.h"

void serialSetup(int baudRate) {
    Serial.begin(baudRate);
}

