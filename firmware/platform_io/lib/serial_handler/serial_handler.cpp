#include <Arduino.h>
#include "serial_handler.h"

SerialHandler::SerialHandler(unsigned long baudRate) : _baudRate(baudRate) {
    _message = "";
}
void SerialHandler::begin() {
    Serial.begin(_baudRate);
}

void SerialHandler::log(const String &message) {
    Serial.println(message);
}

String SerialHandler::readLine() {
    String line = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            break;
        }
        line += c;
    }
    _message = line;
    return line;
}

int SerialHandler::getVelocity() {
    if(_message==""){
        return 0;
    }
    Serial.println("Message: " + _message); // Debug print
    int velocity = _message.substring(0,_message.indexOf(",")).toInt();
    return velocity;
}

int SerialHandler::getAngle() {
    if(_message==""){
        return 0;
    }
    int angle = _message.substring(_message.indexOf(",")+1).toInt();
    return angle;
}