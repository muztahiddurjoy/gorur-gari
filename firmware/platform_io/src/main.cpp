#include "servo_controller.h"
#include "encoder_controller.h"
#include "motor_controller.h"

ServoController servo(18);
EncoderController encoder(34, 35);
MotorController motor(13, 12);  // Only motor pins, no servo pin

int temp_count = 0;
bool dhon = false;;

void setup() {
    Serial.begin(115200);
    motor.begin();   // No servo conflicts
    encoder.begin();
    servo.begin();   // Your servo controller uses timer 0
}

void loop() {
    encoder.updateOdometry();
    
    Serial.print("Encoder Count: ");
    Serial.print(encoder.getCount());
    Serial.print(" | Velocity: ");
    Serial.print(encoder.getVelocity());
    Serial.println(" counts/sec");
    
    // Your servo control logic
    if(servo.getAngle() < 180) {
        servo.setAngle(servo.getAngle() + 1);
    }
    else {
        servo.setAngle(0);
    }
    
    // Motor control
    motor.moveForward(100);  // Changed from 0 to actually move
    
    if(temp_count++ > 200) {
        motor.stopMotor();
        temp_count = 0;
    }
    
    delay(20);
}