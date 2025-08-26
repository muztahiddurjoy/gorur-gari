#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

// Pin definitions
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_LEFT 10
#define MOTOR_RIGHT 11
#define SERVO_PIN 12

class MotorController {
private:
    long encoderCount;
    uint8_t lastEncoderState;
    unsigned long lastUpdateTime;

public:
    MotorController();
    void begin();
    void update(); // Call this regularly to update encoder
    
    // Motor control
    void controlMotor(int pwm_value);
    void stopMotor();
    void moveForward(int speed);
    void moveBackward(int speed);
    
    // Servo control (optional)
    void setServoAngle(int angle);
    
    // Encoder
    long getEncoderCount();
};

extern MotorController motorController;

#endif // MOTOR_CONTROLLER_H