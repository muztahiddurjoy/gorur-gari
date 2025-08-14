#include <TFT_eSPI.h>
#include "tft_handler.h"

TFT_eSPI tft = TFT_eSPI();

void tftSetup() {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
}
