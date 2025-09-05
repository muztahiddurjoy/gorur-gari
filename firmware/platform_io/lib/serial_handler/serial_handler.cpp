#include <Arduino.h>
#include "serial_handler.h"

// Remove FreeRTOS components for now to simplify
static String rxBuffer = "";
static const int RX_BUFFER_SIZE = 64;

void serialSetup(void) {
    // rxBuffer.reserve(RX_BUFFER_SIZE);
    Serial.println("Serial handler ready");
}

void serialLoop(void) {
    // Empty for now - handled in main loop
}

bool getSerialCommand(SerialCommand &cmd) {
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxBuffer.length() > 0) {
                int commaIndex = rxBuffer.indexOf(',');
                
                if (commaIndex != -1) {
                    String velStr = rxBuffer.substring(0, commaIndex);
                    String angleStr = rxBuffer.substring(commaIndex + 1);
                    
                    cmd.velocity = velStr.toFloat();
                    cmd.steering_angle = angleStr.toFloat();
                    
                    Serial.printf("Parsed command: %.2f, %.1f\n", cmd.velocity, cmd.steering_angle);
                    rxBuffer = "";
                    return true;
                }
                rxBuffer = "";
            }
        } else if (rxBuffer.length() < RX_BUFFER_SIZE - 1) {
            rxBuffer += c;
        }
    }
    return false;
}

void sendSerialData(const SerialData &data) {
    Serial.print(data.encoder_count);
    Serial.print(",");
    Serial.println(data.button_state);
}

void sendRawData(const String &data) {
    Serial.println(data);
}