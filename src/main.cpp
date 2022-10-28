#include "main.h"
#include <algorithm>
#include <iterator>
#include <utility>
pros::Controller Controller1(pros::E_CONTROLLER_MASTER);

pros::Motor left_motor1 (1,pros::E_MOTOR_GEARSET_36,true);
pros::Motor left_motor2(2,pros::E_MOTOR_GEARSET_36,true);
pros::Motor left_motor3(3,pros::E_MOTOR_GEARSET_36,true);
pros::Motor right_motor1(4,pros::E_MOTOR_GEARSET_36,false);
pros::Motor right_motor2(5,pros::E_MOTOR_GEARSET_36,false);
pros::Motor right_motor3(8,pros::E_MOTOR_GEARSET_36,false);

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

	left_motor1.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	right_motor1.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	left_motor3.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	right_motor3.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);

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
void competition_initialize() {}

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

void PID(float target){
left_motor1.set_zero_position(0);
float kP = 10;
float distance = 0;
float error = target-distance;
int power;
float pend = 999999;
while (error>0.5)
{
	distance = left_motor1.get_position() * 10.21;
	error = target-distance;
	power = error*kP;

	if (error > 10)
	{
		left_motor1.move(127);
		left_motor2.move(127);
		left_motor3.move(127);
		right_motor1.move(127);
		right_motor2.move(127);
		right_motor3.move(127);
	}
	
	if (error <= 10) {
		left_motor1.move(power);
		left_motor2.move(power);
		left_motor3.move(power);
		right_motor1.move(power);
		right_motor2.move(power);
		right_motor3.move(power);
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
left_motor1.set_zero_position(0);
float nkP = 10;
float ndistance = 0;
float nerror = ntarget-ndistance;
int npower;
float nend = 999999;
while (nerror < -0.5)
{
	ndistance = left_motor1.get_position() * 6.126;
	nerror = ntarget-ndistance;
	npower = nerror*nkP;

	if (nerror < -10)
	{
		left_motor1.move(-127);
		left_motor2.move(-127);
		left_motor3.move(-127);
		right_motor1.move(-127);
		right_motor2.move(-127);
		right_motor3.move(-127);
	}
	
	if (nerror >= -10) {
		left_motor1.move(npower);
		left_motor2.move(npower);
		left_motor3.move(npower);
		right_motor1.move(npower);
		right_motor2.move(npower);
		right_motor3.move(npower);

		if (nerror>=-1)
		{
			left_motor1.brake();
			left_motor2.brake();
			left_motor3.brake();
			right_motor1.brake();
			right_motor2.brake();
			right_motor3.brake();
			nerror = -0.4;
		}
	}
	if (ndistance >1 && (nend-ndistance) > -0.001 && (nend-ndistance) < 0.001)
	{
		break;
	}
	nend = ndistance;
	pros::delay(10);
}
}

void Lturn(){
	float lt = 0; 
	right_motor1.set_zero_position(0);
	while(lt < 0.1121) 
	{
		lt = right_motor1.get_position();
		left_motor1.move(-127);
		left_motor2.move(-127);
		left_motor3.move(-127);
		right_motor1.move(127);
		right_motor2.move(127);
		right_motor3.move(127);
	}
		left_motor1.brake();
		left_motor2.brake();
		left_motor3.brake();
		right_motor1.brake();
		right_motor2.brake();
		right_motor3.brake();
}

void Rturn(){
	float rt = 0;
	left_motor1.set_zero_position(0);
	while (rt < 0.1121) {
		rt = left_motor1.get_position();
		left_motor1.move(127);
		left_motor2.move(127);
		left_motor3.move(127);
		right_motor1.move(-127);
		right_motor2.move(-127);
		right_motor3.move(-127);
	}
		left_motor1.brake();
		left_motor2.brake();
		left_motor3.brake();
		right_motor1.brake();
		right_motor2.brake();
		right_motor3.brake();
}
void flypower() {
double xdistance;
double ydistance;
while (true)
{
	xdistance = left_motor3.get_position()*6.126;
	ydistance = right_motor3.get_position()*6.126;
}
}

void opcontrol() {

while(true) {
left_motor1.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
left_motor2.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
left_motor3.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
right_motor1.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
right_motor2.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
right_motor3.move(Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
 
pros::Motor fly_motor(10,pros::E_MOTOR_GEARSET_36,true);
if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
{
	fly_motor.move(127);
}
if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) 
{
	fly_motor.brake();
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) 
{
	PID(10); 
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) 
{
	NPID(-10); 
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
{
	Lturn(); 
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
{
	Rturn(); 
}
if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
{
	PID(20);
	Lturn(); 
	PID(15);
	Rturn();
	PID(10);
	Lturn();
	PID(15);
	Lturn();
	PID(10);
	Lturn();
	PID(10);
	Rturn();
	PID(15);
	Lturn();
	PID(10);
}
pros::delay(30);
}
}

void autonomous() {

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