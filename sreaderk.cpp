#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor left_motor_1(4, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_motor_2(3, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_motor_3(1, pros::E_MOTOR_GEARSET_06, true);
pros::Motor right_motor_1(20, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_motor_2(12, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_motor_3(11, pros::E_MOTOR_GEARSET_06, false);
pros::Motor fly_motor(8, pros::E_MOTOR_GEARSET_06, true);
pros::Motor intake_motor(19, pros::E_MOTOR_GEARSET_18, false);

pros::ADIDigitalOut indexer(1, false);
pros::ADIDigitalOut string_launcher(2, false);

pros::IMU angle(5);
pros::Rotation rotation_sensor(21);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  pros::lcd::register_btn1_cb(on_center_button);

  angle.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
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

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
/*
void PID(float target){
left_motor_1.set_zero_position(0);
float kP = 10;
float distance = 0;
float error = target-distance;
int power;
float pend = 999999;while (error>0.5)
{
        distance = left_motor_1.get_position() * 10.21;
        error = target-distance;
        power = error*kP;

        if (error > 10)
        {
                left_motor_1.move(127);
                left_motor_2.move(127);
                left_motor_3.move(127);
                right_motor_1.move(127);
                right_motor_2.move(127);
                right_motor_3.move(127);
        }

        if (error <= 10) {
                left_motor_1.move(power);
                left_motor_2.move(power);
                left_motor_3.move(power);
                right_motor_1.move(power);
                right_motor_2.move(power);
                right_motor_3.move(power);
        }
        if (distance > 1 && (pend-distance) < 0.001 && (pend-distance) > -0.001)
        {
                break;
        }
        pend = distance;

        pros::delay(10);
}


}
void NPID(float ntarget){
left_motor_1.set_zero_position(0);
float nkP = 10;
float ndistance = 0;
float nerror = ntarget-ndistance;
int npower;
float nend = 999999;
while (nerror < -0.5)
{
        ndistance = left_motor_1.get_position() * 6.126;
        nerror = ntarget-ndistance;
        npower = nerror*nkP;

        if (nerror < -10)
        {
                left_motor_1.move(-127);
                left_motor_2.move(-127);
                left_motor_3.move(-127);
                right_motor_1.move(-127);
                right_motor_2.move(-127);
                right_motor_3.move(-127);
        }

        if (nerror >= -10) {
                left_motor_1.move(npower);
                left_motor_2.move(npower);
                left_motor_3.move(npower);
                right_motor_1.move(npower);
                right_motor_2.move(npower);
                right_motor_3.move(npower);

                if (nerror>=-1)
                {
                        left_motor_1.brake();
                        left_motor_2.brake();
                        left_motor_3.brake();
                        right_motor_1.brake();
                        right_motor_2.brake();
                        right_motor_3.brake();
                        nerror = -0.4;
                }
        }
        if (ndistance >1 && (nend-ndistance) > -0.001 && (nend-ndistance) <
0.001)
        {
                break;
        }
        nend = ndistance;
        pros::delay(10);
}
}

void Lturn(){
        float lt = 0;
        right_motor_1.set_zero_position(0);
        while(lt < 0.82)
        {
                lt = right_motor_1.get_position();
                left_motor_1.move(-127);
                left_motor_2.move(-127);
                left_motor_3.move(-127);
                right_motor_1.move(127);
                right_motor_2.move(127);
                right_motor_3.move(127);
        }
                left_motor_1.brake();
                left_motor_2.brake();
                left_motor_3.brake();
                right_motor_1.brake();
                right_motor_2.brake();
                right_motor_3.brake();
}

void Rturn(){
        float rt = 0;
        left_motor_1.set_zero_position(0);
        while (rt < 0.82) {
                rt = left_motor_1.get_position();
                left_motor_1.move(127);
                left_motor_2.move(127);
                left_motor_3.move(127);
                right_motor_1.move(-127);
                right_motor_2.move(-127);
                right_motor_3.move(-127);
        }
                left_motor_1.brake();
                left_motor_2.brake();
                left_motor_3.brake();
                right_motor_1.brake();
                right_motor_2.brake();
                right_motor_3.brake();
}
void flypower() {
double xdistance;
double ydistance;
while (true)
{
        xdistance = left_motor_3.get_position()*6.126;
        ydistance = right_motor_3.get_position()*6.126;
}
}
*/
void move(int volt) {
  left_motor_1.move(volt);
  left_motor_2.move(volt);
  left_motor_3.move(volt);
  right_motor_1.move(volt);
  right_motor_2.move(volt);
  right_motor_3.move(volt);
}
void move(int volt, bool left) {
  if (left == true) {
    left_motor_1.move(volt);
    left_motor_2.move(volt);
    left_motor_3.move(volt);
  }
  if (left == false) {
    right_motor_1.move(volt);
    right_motor_2.move(volt);
    right_motor_3.move(volt);
  }
}
void brake() {
  left_motor_1.brake();
  left_motor_2.brake();
  left_motor_3.brake();
  right_motor_1.brake();
  right_motor_2.brake();
  right_motor_3.brake();
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
/*void fly_brake() {
        while (fly_motor.get_voltage() > 10)
        {
                fly_motor.move(fly_motor.get_voltage() - 1);
                pros::delay(18);
        }
                fly_motor.brake();
}*/
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
  move(-127);
  pros::delay(500);
  intake_motor.brake();
  brake();
} 	
void roller(bool normal) {
  if (normal) {
    intake_motor.move(-127);
    move(-127);
    pros::delay(500);
    intake_motor.brake();
    brake();
  } else {
    {
      intake_motor.move(127);
      move(-127);
      pros::delay(500);
      intake_motor.brake();
      brake();
    }
  }
}
/*float xcoord = 0;
float ycoord = 0;
double theta = 0;
double distance = 0;*/
double theta = 0;
double PIDvolt = 0;
double RPIDvolt = 0;
double LPIDvolt = 0;
double turn_volt = 0;
void Course_correct(double target_angle) {
  double error = target_angle - angle.get_rotation();
  if (error > 2) {
    RPIDvolt = PIDvolt * (1 - fabs(0.025 * error));
  }
  if (error > 2) {
    LPIDvolt = PIDvolt * (1 - fabs(0.025 * error));
  }
  if (RPIDvolt < 21) {
    if (RPIDvolt > 2) {
      (RPIDvolt = 21);
    } else if (RPIDvolt < -2) {
      (RPIDvolt = -21);
    } else {
      RPIDvolt = 0;
    }
  }
  if (LPIDvolt < 21) {
    if (LPIDvolt > 2) {
      (LPIDvolt = 21);
    } else if (LPIDvolt < -2) {
      (LPIDvolt = -21);
    } else {
      LPIDvolt = 0;
    }
  }
}

double testingPID = 0;
void PID(double target) {
  rotation_sensor.set_position(0);
  double course_angle = angle.get_rotation();
  double derror = target;
  while (derror > 0.4) {
    derror = target - (rotation_sensor.get_position() * 0.000239);
    PIDvolt = fabs(120 * derror) / target;
    /*testingPID = target - (rotation_sensor.get_position()*0.000239);
    std::string testingPID2 = std::to_string(testingPID);
    controller.set_text(1,1,testingPID2);*/
    if (PIDvolt < 28) {
      if (PIDvolt > 3) {
        PIDvolt = 28;
      } else {
        break;
      }
    }
    /*Course_correct(course_angle);
    left_motor_1.move(LPIDvolt);
    left_motor_2.move(LPIDvolt);
    left_motor_3.move(LPIDvolt);
    right_motor_1.move(RPIDvolt);
    right_motor_2.move(RPIDvolt);
    right_motor_3.move(RPIDvolt);*/
    move(PIDvolt);
  }
  while (derror < -0.4) {
    derror = target - (rotation_sensor.get_position() * 0.000239);
    PIDvolt = fabs(120 * derror) / target;
    /*testingPID = target - (rotation_sensor.get_position()*0.000239);
    std::string testingPID2 = std::to_string(testingPID);
    controller.set_text(1,1,testingPID2);*/
    if (PIDvolt > -28) {
      if (PIDvolt < -2) {
        PIDvolt = -28;
      } else {
        break;
      }
    }
    /*Course_correct(course_angle);
    left_motor_1.move(LPIDvolt);
    left_motor_2.move(LPIDvolt);
    left_motor_3.move(LPIDvolt);
    right_motor_1.move(RPIDvolt);
    right_motor_2.move(RPIDvolt);
    right_motor_3.move(RPIDvolt);*/
    move(PIDvolt);
  }
  brake();
}

void turn(double degrees) {
  angle.reset();
  while (degrees - angle.get_rotation() > 2) {
    turn_volt = fabs((degrees - angle.get_rotation()) / degrees);
    if (turn_volt > 115) {
      turn_volt = 115;
    } else if (turn_volt < 10 && turn_volt > 2) {
      turn_volt = 10;
    }
    left_motor_1.move(-turn_volt);
    left_motor_2.move(-turn_volt);
    left_motor_3.move(-turn_volt);
    right_motor_1.move(turn_volt);
    right_motor_2.move(turn_volt);
    right_motor_3.move(turn_volt);
  }
  while (degrees - angle.get_rotation() < -2) {
    turn_volt = fabs((degrees - angle.get_rotation()) / degrees);
    if (turn_volt > 115) {
      turn_volt = 115;
    } else if (turn_volt < 10 && turn_volt > 2) {
      turn_volt = 10;
    }
    left_motor_1.move(turn_volt);
    left_motor_2.move(turn_volt);
    left_motor_3.move(turn_volt);
    right_motor_1.move(-turn_volt);
    right_motor_2.move(-turn_volt);
    right_motor_3.move(-turn_volt);
  }
  brake();
}

void move_byPolar(double r, double O) {
  turn(O);
  PID(r);
}
/*double odon(bool xaxis) {
        theta = angle.get_rotation()*0.0174533;
        distance = rotation_sensor.get_position()*0.000239;
        xcoord = xcoord + distance*sin(theta);
        ycoord = ycoord + distance*cos(theta);
        if (xaxis)
        {
                return xcoord;
        }
        else
        {
                return ycoord;
        }
        controller.set_text(1,1,xcoord2);
        controller.set_text(1,8,ycoord2);8
  }
    double odon(int ang) {
                theta = angle.get_rotation()*0.0174533;
                distance = rotation_sensor.get_position()*0.000239;
                xcoord = distance*sin(theta);
                ycoord = ycoord + distance*cos(theta);
                if (ang > 0)
                {
                        return theta;
                }
                else
                {
                        return 0.0;
                }
        }*/
void auton_skills() {
  roller();
  fly_motor.move(-127);
  pros::delay(2000);
  move(80, 1700.0);
  shoot();
  pros::delay(2000);
  shoot();
  move(-80, 1500.0);
  move(80, true, 390);
  move(100, 4000.0);
  expansion();
  move(-80, 200.0);
  move(-80, 50.0);
}

void opcontrol() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      intake_motor.move(127);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      intake_motor.brake();
      controller.rumble(".-");
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      intake_motor.move(-127);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      indexer.set_value(true);
      pros::delay(27);
      indexer.set_value(false);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      string_launcher.set_value(true);
      pros::delay(250);
      string_launcher.set_value(false);
    }
    if (fly_motor.get_actual_velocity() > 3500) {
      controller.rumble("..-..-");
      controller.set_text(1, 1, "SHOOT!!");
    }
    /*double distance2 = angle.get_rotation();
    int full_rot_neg = (int)(theta/360);
    theta = angle.get_rotation() - (360*full_rot_neg);
    std::string distance3 = std::to_string(theta);
    double velocity = 6*fly_motor.get_actual_velocity();
    std::string velocity2 = std::to_string(velocity);
    controller.set_text(1,1,distance3);*/
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      PID(48);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      roller();
      fly_motor.move(-127);
      pros::delay(2000);
      move(80, 1700.0);
      shoot();
      pros::delay(2000);
      shoot();
      move(-80, 1500.0);
      move(80, true, 390);
      move(100, 4000.0);
      expansion();
      move(-80, 200.0);
      move(-80, 50.0);
      /*roller();
      fly_motor.move(-97);
      pros::delay(2000);
      move(80);
      pros::delay(50);
      brake();
      pros::delay(100);
      move(80, true);
      pros::delay(730);
      brake();
      pros::delay(800);
      shoot();
      pros::delay(2000);
      shoot();
      fly_motor.brake();*/
    }
    /*if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    {
            intake_motor.move(-127);
            move(-127);
            pros::delay(400);
            intake_motor.brake();
            brake();
            pros::delay(2000);
            move(80,1800.0);
            fly_motor.move(-127);
            pros::c::delay(5000);
            shoot();
            pros::c::delay(3800);
            shoot();
            fly_motor.brake();
            move(-60,1700.0);
            move(45, true,600.0);
            move(70,1300.0);
            string_launcher.set_value(true);
            pros::delay(250);
            string_launcher.set_value(false);
            move(-70,550.0);
            move(80,240.0);
    }*/
    pros::delay(30);
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