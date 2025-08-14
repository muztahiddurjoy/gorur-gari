#ifndef MYMODULE_H
#define MYMODULE_H

#include <Arduino.h>


extern int buttonState;
extern int activeButton;


void btnSetup(int pin = 0);
void btnLoop();


#endif
