#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>

class SerialHandler {
    public:
        SerialHandler(unsigned long baudRate);
        void begin();
        void log(const String &message);
        String readLine();
        int getVelocity(String rawString);
        int getAngle(String rawString);
    private:
        unsigned long _baudRate;
        String _message="";
};
#endif // SERIAL_HANDLER_H