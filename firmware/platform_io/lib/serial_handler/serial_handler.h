#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>

struct SerialCommand {
    float velocity;
    float steering_angle;
};

struct SerialData {
    long encoder_count;
    int button_state;
};

void serialSetup(void);
bool getSerialCommand(SerialCommand &cmd);
void sendSerialData(const SerialData &data);
void sendRawData(const String &data);

#endif // SERIAL_HANDLER_H