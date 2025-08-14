#include <Arduino.h>
#include <TFT_eSPI.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "button_handler.h"



#define BUTTON_1 35
#define BUTTON_2 0

TFT_eSPI tft = TFT_eSPI(); 

void setup() {
    Serial.begin(115200);
    btnSetup(BUTTON_1);
    btnSetup(BUTTON_2);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextSize(3);
tft.drawString("I Hate RS", 10, 10, 2);
}

void loop() {
    btnLoop();
}

