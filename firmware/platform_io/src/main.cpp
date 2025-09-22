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

int last_vel = 0;
int last_angle = 0;

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
    int buttonState = button.isPressed() ? 1 : 0;

    // Log encoder + button state
    serialHandler.log(
        String(encoder.getCount()) + "," +
        String(encoder.getVelocity()) + "," +
        String(buttonState)
    );

    // Get latest commands
    int angle = serialHandler.getAngle();
    int velocity = serialHandler.getVelocity();

    // Update stored values if new commands are received
    if (angle != last_angle) {
        last_angle = angle;
    }
    if (velocity != last_vel) {
        last_vel = velocity;
    }

    // Always apply the last commanded values
    servo.setAngle(last_angle);
    if (last_vel < 0) {
        motor.moveBackward(-last_vel);
    } else {
        motor.moveForward(last_vel);
    }
}
