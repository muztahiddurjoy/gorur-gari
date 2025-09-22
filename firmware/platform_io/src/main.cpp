#include "servo_controller.h"
#include "encoder_controller.h"

ServoController servo(18);
EncoderController encoder(34, 35);

void setup() {
  Serial.begin(115200);
  servo.begin();
  encoder.begin();
}

void loop() {
    encoder.updateOdometry();
    Serial.print("Encoder Count: ");
    Serial.print(encoder.getCount());
    Serial.print(" | Velocity: ");
    Serial.print(encoder.getVelocity());
    Serial.println(" counts/sec");
    if(servo.getAngle() < 90) {
        servo.setAngle(servo.getAngle() + 1);
    }
    else {
        servo.setAngle(0);
    }
    delay(20);
}