#include "utilities.hpp"
#include "devices.hpp"

void brake() {
  left_motors.brake();
  right_motors.brake();
}

void move(int volt) {
  left_motors.move(volt);
  right_motors.move(volt);
}

void move(int volt, bool left) {
  if (left == true) {
    left_motors.move(volt);
  }
  if (left == false) {
    right_motors.move(volt);
  }
}

void move(int volt, double delay) {
  move(volt);
  pros::delay(delay);
  brake();
}

void move(int volt, bool left, double delay) {
  move(volt, left);
  pros::delay(delay);
  brake();
}

void expansion() {
  string_launcher.set_value(true);
  pros::delay(300);
  string_launcher.set_value(false);
}

void shoot() {
  indexer.set_value(true);
  pros::delay(27);
  indexer.set_value(false);
}

void roller() {
  intake_motor.move(-127);
  pros::delay(500);
  intake_motor.brake();
}

void roller(bool normal) {
  if (normal) {
    intake_motor.move(-127);
    pros::delay(500);
    intake_motor.brake();
  } else {
    {
      intake_motor.move(127);
      pros::delay(500);
      intake_motor.brake();
    }
  }
}