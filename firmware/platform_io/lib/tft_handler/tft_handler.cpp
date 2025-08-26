#include <TFT_eSPI.h>
#include "tft_handler.h"

TFT_eSPI tft = TFT_eSPI();
unsigned long lastTftRefresh = 0;
const unsigned long TFT_REFRESH_INTERVAL = 100; // Refresh every 100ms

void tftSetup() {
    tft.init();
    tft.setRotation(1); // Try 0, 1, 2, 3 if display orientation is wrong
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextFont(2);
    
    // Show startup message
    tftShowMessage("System Starting...", TFT_GREEN);
    delay(2000);
    tftClear();
}

void tftUpdate(long encoder_count, int button_state, float velocity, float steering_angle) {
    unsigned long currentTime = millis();
    
    // Only refresh the display at a reasonable rate to avoid flickering
    if (currentTime - lastTftRefresh >= TFT_REFRESH_INTERVAL) {
        tftClear();
        
        // Display header
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.setCursor(5, 5);
        tft.println("CAR STATUS");
        tft.drawLine(0, 25, tft.width(), 25, TFT_WHITE);
        
        // Display values
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(5, 30);
        tft.print("Encoder: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println(encoder_count);
        
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(5, 50);
        tft.print("Button: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println(button_state);
        
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(5, 70);
        tft.print("Velocity: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println(velocity, 2);
        
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(5, 90);
        tft.print("Steering: ");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.println(steering_angle, 1);
        
        // Show uptime
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(5, 110);
        tft.printf("Uptime: %ds", millis() / 1000);
        
        lastTftRefresh = currentTime;
    }
}

void tftShowMessage(const String &message, uint16_t color) {
    tftClear();
    tft.setTextColor(color, TFT_BLACK);
    tft.setCursor(10, tft.height() / 2 - 10);
    tft.println(message);
}

void tftClear() {
    tft.fillScreen(TFT_BLACK);
}