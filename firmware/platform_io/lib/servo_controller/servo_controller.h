#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <ESP32Servo.h>

class ServoController {
public:
    ServoController(int pin);
    void begin();
    void setAngle(int angle);
    int getAngle();

private:
    int _pin;
    Servo _servo;
    int _currentAngle;
};

#endif // SERVO_CONTROLLER_H