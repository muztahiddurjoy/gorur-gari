#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>

void serialSetup(int baudRate = 115200);
String incomingSerialData();
void sendSerialData(const String &data);
void sendSerialData(const char *data);

#endif // SERIAL_HANDLER_H

