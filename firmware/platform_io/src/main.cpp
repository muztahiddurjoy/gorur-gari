#include "servo_controller.h"

ServoController servo(18);

void setup() {
  Serial.begin(115200);
  servo.begin();
  
}

void loop() {
   
}