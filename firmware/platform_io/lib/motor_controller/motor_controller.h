#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

class MotorController {
public:
    MotorController(int pinLeft, int pinRight);
    void begin();
    void controlMotor(int pwmValue);
    void stopMotor();
    void moveForward(int speed);
    void moveBackward(int speed);
    
private:
    int _pinLeft;
    int _pinRight;
    
    // Timer management for ESP32
    static int _nextMotorTimer;
    int _motorTimer;
    
    void _setupMotorPWM();
    void _cleanupMotorPWM();
};

#endif // MOTOR_CONTROLLER_H