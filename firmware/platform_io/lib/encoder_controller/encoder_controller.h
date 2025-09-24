#ifndef ENCODER_CONTROLLER_H
#define ENCODER_CONTROLLER_H

class EncoderController {
public:
    EncoderController(int pinA, int pinB);
    void begin();
    long getCount();
    void resetCount();
    double getVelocity();
    void updateOdometry();
    
private:
    int _pinA;
    int _pinB;
    volatile long _encoderCount;
    long _lastEncoderCount;
    unsigned long _lastTime;
    double _velocity;
    const int _sampleTime;
    
    static const int _lookupTable[];
    
    static EncoderController* _instance;
    static void updateEncoder();
    
    
    void _updateEncoderInstance();
};

#endif // ENCODER_CONTROLLER_H