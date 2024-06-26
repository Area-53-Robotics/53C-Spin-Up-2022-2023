#include "devices.hpp"
#include "main.h"
#include "utilities.hpp"
#include "sylib/sylib.hpp"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  bool flywheel_state = true;
  int flywheel_speed = 127;
  int current_color = 1;

  enum Colors {
    Red = 0xFF0000,
    Blue = 0x0000FF,
    Green = 0x00FF00
  };

  while (true) {
	  // Move drivetrain
    left_motors.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    right_motors.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    // Start flywheel
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      if (flywheel_state) {
		// High speed flywheel
        flywheel_speed = 127;
        controller.rumble(".-.-.");
      }
      if (!flywheel_state) {
		// Low speed flywheel
        flywheel_speed = 106;
        controller.rumble("..");
      }
      flywheel_state = !flywheel_state;
    }
    // Toggles indexer
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      indexer.set_value(true);
      pros::delay(27);
      indexer.set_value(false);
    }
	  // Turns flywheel on
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      fly_motor.move(flywheel_speed);
      controller.rumble(".");
    }
	  // Turns flywheel off
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      fly_motor.brake();
      controller.rumble("..");
    }
	  // Starts intake
	  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      intake_motor.move(127);
    }
	  // Stops intake
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      intake_motor.brake();
      controller.rumble(".-");
    }
	// Reverses intake
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      intake_motor.move(-127);
    }
    // Launch string
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      string_launcher.set_value(true);
      pros::delay(250);
      string_launcher.set_value(false);
    }
    // Change LEDs
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      switch (current_color) {
        // Red Alliance
        case 1:
          led_strip.set_all(Red);
          break;
        // Blue Alliance
        case 2:
          led_strip.set_all(Blue);
          break;
        // Green Alliance
        case 3:
          led_strip.set_all(Green);
          break;
        // Turn LEDs off
        default:
          led_strip.clear();
          break;
      }
      current_color++;
    }
    printf("%i",current_color);
  }  
}