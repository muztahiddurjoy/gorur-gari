#include "servo_controller.h"
#include "encoder_controller.h"
#include "motor_controller.h"
#include "serial_handler.h"
#include "button_handler.h"

ServoController servo(18);
EncoderController encoder(34, 35);
MotorController motor(13, 12);  // Only motor pins, no servo pin
SerialHandler serialHandler(115200);
ButtonHandler button(0); // GPIO 0 for the button

int temp_count = 0;

void setup() {
    serialHandler.begin();
    button.begin();
    motor.begin();  // No servo conflicts
    encoder.begin();
    servo.begin();   // Your servo controller uses timer 0
}

void loop() {
    encoder.updateOdometry();
    serialHandler.readLine();
    int buttonState = button.isPressed()?1:0;
    serialHandler.log(String(encoder.getCount())+","+String(encoder.getVelocity())+","+String(buttonState));
    serialHandler.log("Servo angle: " + String(servo.getAngle()));
    int angle = serialHandler.getAngle();
    int velocity = serialHandler.getVelocity();
    servo.setAngle(angle);
    if(velocity < 0) {
        motor.moveBackward(-velocity);
    } else {
        motor.moveForward(velocity);
    }
}