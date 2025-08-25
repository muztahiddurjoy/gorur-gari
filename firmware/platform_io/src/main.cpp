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

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(1000); // Wait for serial to stabilize
    Serial.println("System starting...");
    
    // Initialize components with error checking
    Serial.println("Initializing TFT...");
    tftSetup();
    
    Serial.println("Initializing buttons...");
    buttonHandler.begin(BUTTON_1, BUTTON_2);
    
    Serial.println("Initializing motor controller...");
    motorController.begin();
    
    Serial.println("Initializing serial handler...");
    serialSetup();
    
    Serial.println("System ready - starting main loop");
    lastLoopTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Feed the watchdog regularly
    if (currentTime - lastLoopTime > 1000) {
        Serial.println("Loop running...");
        lastLoopTime = currentTime;
    }
    
    // Update button states
    buttonHandler.update();
    
    // Process incoming commands
    SerialCommand cmd;
    if (getSerialCommand(cmd)) {
        Serial.printf("Command: Vel=%.2f, Angle=%.1f\n", cmd.velocity, cmd.steering_angle);
        
        currentVelocity = cmd.velocity;
        currentSteering = cmd.steering_angle;
        
        // Apply velocity command
        int motorSpeed = constrain(cmd.velocity * 255, -255, 255);
        motorController.controlMotor(motorSpeed);
    }
    
    // Send serial data
    SerialData data;
    data.button_state = buttonHandler.getButtonState();
    data.encoder_count = motorController.getEncoderCount();
    sendSerialData(data);
    
    // Update TFT with current values
    tftUpdate(
        data.encoder_count,
        data.button_state,
        currentVelocity,
        currentSteering
    );
    
    // CRITICAL: Add delay to prevent watchdog reset
    delay(10);
    yield(); // Give time for background tasks
}