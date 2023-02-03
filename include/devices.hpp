#pragma once

#include "api.h"
#include "pros/adi.hpp"
#include "sylib/sylib.hpp"

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

/*
pros::Motor left_motor_1 (4,pros::E_MOTOR_GEARSET_06,true);
pros::Motor left_motor_2(3,pros::E_MOTOR_GEARSET_06,true);
pros::Motor left_motor_3(1,pros::E_MOTOR_GEARSET_06,true);
pros::Motor right_motor_1(20,pros::E_MOTOR_GEARSET_06,false);
pros::Motor right_motor_2(12,pros::E_MOTOR_GEARSET_06,false);
pros::Motor right_motor_3(11,pros::E_MOTOR_GEARSET_06,false);
*/

inline pros::Motor fly_motor(8,pros::E_MOTOR_GEARSET_06);
inline pros::Motor intake_motor(19,pros::E_MOTOR_GEARSET_18,false);

inline pros::Motor_Group left_motors({4, 3, 1});
inline pros::Motor_Group right_motors({20, 12, 11});

inline pros::ADIDigitalOut indexer(1, false);
inline pros::ADIDigitalOut string_launcher(2, false);

inline pros::ADIEncoder tracking_wheels(3, 4);

inline sylib::Addrled led_strip(22, 5, 10);