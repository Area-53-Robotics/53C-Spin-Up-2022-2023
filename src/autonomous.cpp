#include "devices.hpp"
#include "utilities.hpp"

bool auton_mode = true;
void competition_initialize() {
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    auton_mode = !auton_mode;
    if (auton_mode) {
      controller.rumble(".-.-.");
    }
    if (!auton_mode) {
      controller.rumble("....");
    }
  }
}

void autonomous() {
  if (auton_mode) {
    roller();
    fly_motor.move(-97);
    pros::delay(2000);
    move(80);
    pros::delay(50);
    brake();
    pros::delay(100);
    move(80, true);
    pros::delay(1050);
    brake();
    pros::delay(800);
    shoot();
    pros::delay(2000);
    shoot();
    fly_motor.brake();
  } else {
    fly_motor.move(-93);
    pros::delay(3000);
    shoot();
    pros::delay(2500);
    shoot();
    fly_motor.brake();
  }
}