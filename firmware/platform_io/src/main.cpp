#include <Arduino.h>
#include <TFT_eSPI.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// put function declarations here:
int myFunction(int, int);
TFT_eSPI tft = TFT_eSPI(); 

void setup() {
tft.init();
tft.setRotation(1);
tft.fillScreen(TFT_BLACK);
tft.setTextColor(TFT_YELLOW, TFT_BLACK);
tft.setTextSize(3);
tft.drawString("I Hate RS", 10, 10, 2);
}

void loop() {
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}