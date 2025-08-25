#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>  

// Pin definitions
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTOR_LEFT 10
#define MOTOR_RIGHT 11
#define SERVO_PIN 12 

// Motor direction constants
#define FORWARD 1
#define BACKWARD -1
#define STOP 0

// Odometry structure
struct OdometryData {
    volatile long encoderCount;
    long lastEncoderCount;
    unsigned long lastTime;
    const int sampleTime = 100; // ms
};

// Motor Controller class
class MotorController {
private:
    Servo myServo;
    OdometryData odometry;
    
    // Encoder interrupt handler (needs to be static for ISR)
    static void updateEncoderISR();
    
public:
    MotorController();
    void begin();
    void update();
    
    // Motor control
    void controlMotor(int pwm_value);
    void stopMotor();
    void moveForward(int speed);
    void moveBackward(int speed);
    
    // Servo control
    void setServoAngle(int angle);
    int getServoAngle();
    
    // Odometry
    long getEncoderCount();
    long getEncoderDelta();
    float getDistanceTraveled(float wheelCircumference, int pulsesPerRevolution);
    
    // Static members for ISR access
    static volatile long encoderCountStatic;
    static const int8_t lookup_table[];
};

extern MotorController motorController; // Global instance

#endif // MOTOR_CONTROLLER_H