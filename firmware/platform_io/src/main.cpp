#include <Arduino.h>
#include "button_handler.h"
#include "serial_handler.h"
#include "tft_handler.h"

#define BUTTON_1 35
#define BUTTON_2 0


void setup() {
    serialSetup(115200);
    btnSetup(BUTTON_1);
    tftSetup();
}

void loop() {
    btnLoop();
    serialLoop();
}

