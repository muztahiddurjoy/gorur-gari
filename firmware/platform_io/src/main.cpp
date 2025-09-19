#include <Arduino.h>
#include "serial_handler.h"
#include "button_handler.h"
#include "motor_controller.h"
#include "tft_handler.h"

#define BUTTON_1 35
#define BUTTON_2 0

// Global variables
float currentVelocity = 0;
float currentSteering = 0;
unsigned long lastLoopTime = 0;
unsigned long lastTftUpdate = 0;
const unsigned long TFT_UPDATE_INTERVAL = 200; // Update TFT every 200ms

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(1000); // Wait for serial to stabilize
    
    // Initialize TFT FIRST - show startup status immediately
    tftSetup();
    
    // Initialize buttons (will work even if not physically connected)
    buttonHandler.begin(BUTTON_1, BUTTON_2);
    
    // Initialize motor controller (will work in simulation mode if not connected)
    motorController.begin();
    
    // Initialize serial handler
    serialSetup();
    
    // Show system ready message on TFT
    tftShowMessage("System Ready!", TFT_GREEN);
    delay(2000); // Show for 2 seconds
    
    lastLoopTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Feed the watchdog regularly and show heartbeat
    if (currentTime - lastLoopTime > 1000) {
        
        lastLoopTime = currentTime;
    }
    
    // Always update button states (even if no buttons connected)
    buttonHandler.update();
    int buttonState = buttonHandler.getButtonState();
    if(buttonState != 0) {
        tftSetTimeStamp(currentTime);
    }
    
    // Always read encoder (even if no encoder connected - will return 0 or simulated data)
    long encoderCount = motorController.getEncoderCount();
    
    // Process incoming commands (will work even if no motors connected)
    SerialCommand cmd;
    bool newCommand = getSerialCommand(cmd);
    if (newCommand) {
        Serial.printf("Command: Vel=%.2f, Angle=%.1f\n", cmd.velocity, cmd.steering_angle);
        
        currentVelocity = cmd.velocity;
        currentSteering = cmd.steering_angle;
        
        // Apply velocity command (motor controller will handle no-connection gracefully)
        int mapped_pwm = map(cmd.velocity, -100, 100, -255, 255);
        int motorSpeed = constrain(mapped_pwm, -255, 255);
        motorController.controlMotor(motorSpeed);
    }
    
    // Always send serial data (for monitoring)
    SerialData data;
    data.button_state = buttonState;
    data.encoder_count = encoderCount;
    // sendSerialData(data);
    
    // ALWAYS update TFT regardless of motor/servo connection status
    if (currentTime - lastTftUpdate >= TFT_UPDATE_INTERVAL) {
        tftUpdate(
            encoderCount,
            buttonState,
            currentVelocity,
            currentSteering
        );
        
        // Optional: Debug output
        Serial.printf("TFT: Enc=%ld, Btn=%d, Vel=%.2f, Ang=%.1f\n", 
                     encoderCount, buttonState, currentVelocity, currentSteering);
        
        lastTftUpdate = currentTime;
    }
    
    // CRITICAL: Add delay to prevent watchdog reset
    delay(10);
    yield(); // Give time for background tasks
}