#include <Arduino.h>
#include <TFT_eSPI.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "button_handler.h"
#include "tft_handler.h"



#define BUTTON_1 35
#define BUTTON_2 0


void setup() {
    Serial.begin(115200);
    btnSetup(BUTTON_1);
    tftSetup();
}

void loop() {
    btnLoop();
}

