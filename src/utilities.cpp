#include "utilities.hpp"
#include "devices.hpp"
#include "cmath"

void brake() {
  left_motors.brake();
  right_motors.brake();
}

void move_pid(float distance) {
  left_tracking_wheel.reset();
  float error = distance;
  float current_position;
  float power;
  float kp;
  while (error > 0.5) {
    current_position = M_PI*2.75 / 3600 * left_tracking_wheel.get_position();
    error = distance - current_position;
    power = error * kp;
    left_motors.move(power);
    right_motors.move(power);
  }
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

void turn(int volt, bool left) {
  if (left == true) {
    left_motors.move(volt);
    right_motors.move(-volt);
  }
  if (left == false) {
    left_motors.move(-volt);
    right_motors.move(volt);
  }
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