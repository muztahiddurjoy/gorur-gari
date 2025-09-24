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

int SerialHandler::getVelocity(String rawString) {
    if(rawString==""){
        return 0;
    }
    int velocity = rawString.substring(0,rawString.indexOf(",")).toInt();
    return velocity;
}

int SerialHandler::getAngle(String rawString) {
    if(rawString==""){
        return 0;
    }
    int angle = rawString.substring(rawString.indexOf(",")+1).toInt();
    return angle;
}