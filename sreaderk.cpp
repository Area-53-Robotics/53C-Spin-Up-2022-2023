#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <string>
#include <type_traits>
#include <utility>
#include <cmath>

pros::Controller Controller1(pros::E_CONTROLLER_MASTER);

pros::Motor left_motor1(3,pros::E_MOTOR_GEARSET_06,true);
pros::Motor left_motor2(1,pros::E_MOTOR_GEARSET_06,true);
pros::Motor left_motor3 (4,pros::E_MOTOR_GEARSET_06,true);
pros::Motor right_motor1(20,pros::E_MOTOR_GEARSET_06,false);
pros::Motor right_motor2(11,pros::E_MOTOR_GEARSET_06,false);
pros::Motor right_motor3(12,pros::E_MOTOR_GEARSET_06,false);

pros::Motor fly_motor(8,pros::E_MOTOR_GEARSET_06,true);
pros::Motor intake_motor(19,pros::E_MOTOR_GEARSET_18,false);

pros::ADIDigitalOut piston(1, false);
pros::ADIDigitalOut piston2(2, false);

pros::IMU angle(5);
//pros::ADIEncoder angle(5,6);
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

	angle.reset(true);
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
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
{
	auton_mode = !auton_mode;
	if (auton_mode)
	{
		Controller1.rumble(".-.-.");
		Controller1.set_text(1,1, "On R");
	}
	if (!auton_mode)
	{
		Controller1.rumble("....");
		Controller1.set_text(1,1,"Off R");
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

void move(int volt) {
	left_motor1.move(volt);
	left_motor2.move(volt);
	left_motor3.move(volt);
	right_motor1.move(volt);
	right_motor2.move(volt);
	right_motor3.move(volt);
}
void move(int volt, bool left) {
	if (left == true)
	{
		left_motor1.move(volt);
		left_motor2.move(volt);
		left_motor3.move(volt);
	}
	if (left == false)
	{
		right_motor1.move(volt);
		right_motor2.move(volt);
		right_motor3.move(volt);
	}
}
void brake() {
	left_motor1.brake();
	left_motor2.brake();
	left_motor3.brake();
	right_motor1.brake();
	right_motor2.brake();
	right_motor3.brake();
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
	piston2.set_value(true);
	pros::delay(300);
	piston2.set_value(false);
}
void shoot() {
	piston.set_value(true);
	pros::delay(27);
	piston.set_value(false);
}
void roller() {
	intake_motor.move(-127);
	move(-127);
	pros::delay(650);
	intake_motor.brake();
	brake();
}
void roller(bool normal) {
	if (normal)
	{
		intake_motor.move(-127);
		move(-109);
		pros::delay(500);
		intake_motor.brake();
		brake();
	}
	else {
	{
		intake_motor.move(-127);
		move(-109);
		pros::delay(1000);
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
/*void Course_correct(double target_angle) {
	double error = target_angle - angle.get_value();
	if (error > 2)
	{
		RPIDvolt = PIDvolt * (1 - fabs(0.025*error));
	}
	if (error > 2)
	{
		LPIDvolt = PIDvolt * (1 - fabs(0.025*error));
	}
	if (RPIDvolt < 21)
	{
		if (RPIDvolt > 2)
		{
			(RPIDvolt = 21);
		}
		else if (RPIDvolt < -2)
		{
			(RPIDvolt = -21);
		}
		else
		{
			RPIDvolt = 0;
		}
	}
	if (LPIDvolt < 21)
	{
		if (LPIDvolt > 2)
		{
			(LPIDvolt = 21);
		}
		else if (LPIDvolt < -2)
		{
			(LPIDvolt = -21);
		}
		else
		{
			LPIDvolt = 0;
		}
	}
}*/

double testingPID = 0;
void PID(double target) {
	rotation_sensor.set_position(0);
	double course_angle = angle.get_rotation();
	double derror = target;
	while (derror > 0.4)
	{
		derror = target - (rotation_sensor.get_position()*0.000239);
		PIDvolt = fabs(120*derror)/target;
		/*testingPID = target - (rotation_sensor.get_position()*0.000239);
		std::string testingPID2 = std::to_string(testingPID);
		Controller1.set_text(1,1,testingPID2);*/
		if (PIDvolt < 28)
		{
			if (PIDvolt > 3)
			{
				PIDvolt = 28;
			}
			else
			{
				break;
			}
		}
		/*Course_correct(course_angle);
		left_motor1.move(LPIDvolt);
		left_motor2.move(LPIDvolt);
		left_motor3.move(LPIDvolt);
		right_motor1.move(RPIDvolt);
		right_motor2.move(RPIDvolt);
		right_motor3.move(RPIDvolt);*/
		move(PIDvolt);
	}
		while (derror < -0.4)
	{
		derror = target - (rotation_sensor.get_position()*0.000239);
		PIDvolt = fabs(120*derror)/target;
		/*testingPID = target - (rotation_sensor.get_position()*0.000239);
		std::string testingPID2 = std::to_string(testingPID);
		Controller1.set_text(1,1,testingPID2);*/
		if (PIDvolt > -28)
		{
			if (PIDvolt < -2)
			{
				PIDvolt = -28;
			}
			else
			{
				break;
			}
		}
		/*Course_correct(course_angle);
		left_motor1.move(LPIDvolt);
		left_motor2.move(LPIDvolt);
		left_motor3.move(LPIDvolt);
		right_motor1.move(RPIDvolt);
		right_motor2.move(RPIDvolt);
		right_motor3.move(RPIDvolt);*/
		move(PIDvolt);
	}
	brake();
}

double angle_reference = 0;
double angle_read = angle.get_rotation();
void turn(double degrees)
{	
angle.tare();	
angle_read = angle_reference + angle.get_rotation();
if (fabs(degrees - angle_read) > 0.56)
{
	int check2 = 1;
	bool again2 = true;
	double breakcheck2;
	while (degrees - angle_read > 1)
	{
		angle_read = angle_reference + angle.get_rotation();
		turn_volt = fabs((degrees - angle_read)*1.3+15);
		std::string angle_read2 = std::to_string(angle_read);
		Controller1.set_text(1,1,angle_read2);
		if (turn_volt > 115)
		{
			turn_volt = 115;
		}
		else if (turn_volt < 30 && turn_volt > 0.2)
		{
			turn_volt = 30;
		}
		left_motor1.move(turn_volt);
		left_motor2.move(turn_volt);
		left_motor3.move(turn_volt);
		right_motor1.move(-turn_volt);
		right_motor2.move(-turn_volt);
		right_motor3.move(-turn_volt);
		check2+=1;
		if (check2%20000 == 0)
		{
			if (fabs(angle.get_rotation() - breakcheck2) < 0.1)
			{
				again2 = false;
				break;
			}
			breakcheck2 = angle.get_rotation();
		}
		pros::delay(8);
	}
	while (degrees - angle_read < -1)
	{
		angle_read = angle_reference + angle.get_rotation();
		turn_volt = fabs((degrees - angle_read)*1.3+15);
		std::string angle_read2 = std::to_string(angle_read);
		Controller1.set_text(1,1,angle_read2);
		if (turn_volt > 115)
		{
			turn_volt = 115;
		}
		else if (turn_volt < 30 && turn_volt > 0.2)
		{
			turn_volt = 30;
		}
		left_motor1.move(-turn_volt);
		left_motor2.move(-turn_volt);
		left_motor3.move(-turn_volt);
		right_motor1.move(turn_volt);
		right_motor2.move(turn_volt);
		right_motor3.move(turn_volt);
		check2+=1;
		if (check2%20000 == 0)
		{
			if (fabs(angle.get_rotation() - breakcheck2) < 0.1)
			{
				again2 = false;
				break;
			}
			breakcheck2 = angle.get_rotation();
		}
		pros::delay(8);
	}
	brake();
	pros::delay(500);
	angle_reference = angle_read;
	if (again2 && fabs(degrees - angle_read) > 2)
	{
		turn(degrees - angle_read);
	}
}	
}

void turn(double degrees, bool left_side)
{	
angle.tare();	
angle_read = angle_reference + angle.get_rotation();
if (fabs(degrees - angle_read) > 0.56)
{
	int check2 = 1;
	bool again2 = true;
	double breakcheck2;
	while (degrees - angle_read > 0.4)
	{
		angle_read = angle_reference + angle.get_rotation();
		turn_volt = fabs((degrees - angle_read)*1.86 + 10);
		std::string angle_read2 = std::to_string(angle_read);
		Controller1.set_text(1,1,angle_read2);
		if (turn_volt > 115)
		{
			turn_volt = 115;
		}
		else if (turn_volt < 22 && turn_volt > 0.2)
		{
			turn_volt = 22;
		}
		if (left_side == true)
		{
			left_motor1.move(turn_volt);
			left_motor2.move(turn_volt);
			left_motor3.move(turn_volt);
		}
		if (left_side == false)
		{
			right_motor1.move(-turn_volt);
			right_motor2.move(-turn_volt);
			right_motor3.move(-turn_volt);
		}
		check2+=1;
		if (check2%20000 == 0)
		{
			if (fabs(angle.get_rotation() - breakcheck2) < 0.1)
			{
				again2 = false;
				break;
			}
			breakcheck2 = angle.get_rotation();
		}
		pros::delay(8);
	}
	while (degrees - angle_read < -0.4)
	{
		angle_read = angle_reference + angle.get_rotation();
		turn_volt = fabs((degrees - angle_read)*1.86 + 10);
		std::string angle_read2 = std::to_string(angle_read);
		Controller1.set_text(1,1,angle_read2);
		if (turn_volt > 115)
		{
			turn_volt = 115;
		}
		else if (turn_volt < 22 && turn_volt > 0.2)
		{
			turn_volt = 22;
		}
		if (left_side == true)
		{
			left_motor1.move(-turn_volt);
			left_motor2.move(-turn_volt);
			left_motor3.move(-turn_volt);
		}
		if (left_side == false)
		{
			right_motor1.move(turn_volt);
			right_motor2.move(turn_volt);
			right_motor3.move(turn_volt);
		}
		check2+=1;
		if (check2%20000 == 0)
		{
			if (fabs(angle.get_rotation() - breakcheck2) < 0.1)
			{
				again2 = false;
				break;
			}
			breakcheck2 = angle.get_rotation();
		}
	}
	brake();
	angle_reference = angle_read;
	angle.tare();
	pros::delay(600);
	if (again2 && fabs(degrees - angle_read) > 0.6)
	{
		turn(degrees - angle_read,left_side);
	}
}	
}

void move_inch(double inches)
{
	rotation_sensor.set_position(0);
	angle.tare();
	double kp = 2.35;
	int check = 0;
	double breakcheck;
	bool again = true;
	double current = rotation_sensor.get_position()*0.000273;
	if (inches > 0)
	{
		while (inches - current > 0.1)
		{
			move(kp*(inches-current)+20);
			current = rotation_sensor.get_position()*0.00027;
			std::string current2 = std::to_string(current);
			Controller1.set_text(1,1,current2);
			check+=1;
			if (check%1425 == 0)
			{
				if (fabs(rotation_sensor.get_position() - breakcheck) < 0.001);
				{
					again = false;
					break;
				}
				breakcheck = rotation_sensor.get_position();
			}
			pros::delay(2);
		}
	} 
	if (inches < 0)
	{
		while (current - inches > 0.1)
		{
			move(-kp*(current-inches)-20);
			current = rotation_sensor.get_position()*0.00027;
			std::string current2 = std::to_string(current);
			Controller1.set_text(1,1,current2);
			check+=1;
			if (check%1425 == 0)
			{
				if (fabs(rotation_sensor.get_position() - breakcheck) < 0.001)
				{
					again = false;
					break;
				}
				breakcheck = rotation_sensor.get_position();
			}
			pros::delay(2);
		} 
	}
	brake();
	pros::delay(600);
	angle_read = angle_reference + angle.get_rotation();
	angle_reference = angle_read;
	if (again && fabs(inches-current) > 0.3)
	{
		move_inch(inches-current);
	}
}

void move_polar(double r, double O)
{
	turn(O);
	move_inch(r);
}

void auton_skills()
{
	fly_motor.move(-120);
	for (int i = 0; i < 9; i++)
	{
		while (fly_motor.get_actual_velocity() > -405)
		{
			pros::delay(10);
		}
		shoot();
		pros::delay(2000);
	}
	fly_motor.brake();
	/*roller(false);
	pros::delay(100);
	move_inch(24);
	turn(90);
	move_inch(-27);
	roller(false);
	pros::delay(100);
	angle_reference = 90;*/
	/*fly_motor.move(-127);
	move_inch(4.5);
	move(85, true, 220.0);
	move_inch(65);
	move(85,false,250.0);
	move_inch(5);
	pros::delay(1000);
	shoot();
	pros::delay(2000);
	shoot();
	fly_motor.brake();// Add more shooting function
	// Add other rollers
	move_inch(-45);
	move(80,false,450);
	expansion();
	move_inch(2);
	*/
	/*move_inch(5);
	move_polar(310,30);
	turn(-90);
	move_inch(-4);
	roller(false);
	pros::delay(100);
	angle_reference = -90;
	move_inch(26.5);
	turn(0);
	move(-27);
	roller(false);*/
}

void opcontrol() {
bool mstate = true;
int speed = 127;
while(true) {
left_motor1.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
left_motor2.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
left_motor3.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
right_motor1.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
right_motor2.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
right_motor3.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) 
{
	if (mstate)
	{
		speed = 127;
		Controller1.rumble(".-.-.");
	}
	if (!mstate)
	{
		speed = 106;
		Controller1.rumble("..");
	}
	mstate = !mstate;
}
if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
{
	fly_motor.move(-speed);
	Controller1.rumble(".");
}
if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) 
{
	fly_motor.brake();
	Controller1.rumble("..");
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
{
	intake_motor.move(127);
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
{
	intake_motor.brake();
	Controller1.rumble(".-");
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
{
	intake_motor.move(-127);
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
{
	piston.set_value(true);
	pros::delay(27);
	piston.set_value(false);
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
{
	piston2.set_value(true);
	pros::delay(250);
	piston2.set_value(false);
}
    if (fly_motor.get_actual_velocity()*6 > 3500)
    {
        Controller1.rumble("..-..-");
        Controller1.set_text(1,1,"SHOOT!!");
    }
	double velocity = fly_motor.get_actual_velocity();
	std::string velocity2 = std::to_string(velocity);
	angle_read = angle.get_rotation() + angle_reference;
	std::string angled = std::to_string(angle_read);
	Controller1.set_text(1,1,velocity2);
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
{
	angle_reference = -90;
	angle.tare();
	move_inch(-30);
	turn(0);
	roller();
	move_inch(3);
	turn(-50);
	move_inch(67.9);
	fly_motor.move(-127);
	turn(45);
	while (fly_motor.get_actual_velocity() > -497)
	{
		pros::delay(10);
	}
	shoot();
	pros::delay(200);
	while (fly_motor.get_actual_velocity() > -497)
	{
		pros::delay(10);
	}
	shoot();
	fly_motor.brake();
	}
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
{ 
	auton_skills();
	/*roller();
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
	move(-80, 50.0);*/
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
/*if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
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
	piston2.set_value(true);
	pros::delay(250);
	piston2.set_value(false);
	move(-70,550.0);
	move(80,240.0);
}*/
pros::delay(30);
}

void autonomous() {
	if (!auton_mode)
	{
	roller();
	move_inch(3);
	turn(52);
	move_inch(67.4);
	fly_motor.move(-127);
	turn(-45);
	while (fly_motor.get_actual_velocity() > -497)
	{
		pros::delay(10);
	}
	shoot();
	pros::delay(200);
	while (fly_motor.get_actual_velocity() > -497)
	{
		pros::delay(10);
	}
	shoot();
	fly_motor.brake();
	}
	else
	{
	auton_skills();
	angle_reference = -90;
	angle.tare();
	move_inch(-30);
	turn(0);
	roller();
	move_inch(3);
	turn(-44);
	move_inch(70);
	fly_motor.move(-127);
	turn(45);
	while (fly_motor.get_actual_velocity() > -497)
	{
		pros::delay(10);
	}
	shoot();
	pros::delay(200);
	while (fly_motor.get_actual_velocity() > -497)
	{
		pros::delay(10);
	}
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

