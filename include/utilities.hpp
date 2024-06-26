#pragma once
#include "devices.hpp"
#include "api.h"

void brake();
void move_pid(float distance);
void move(int volt);
void move(int volt, bool left);
void move(int volt, double delay);
void move(int volt, bool left, double delay);
void turn(int volt, bool left);
void expansion();
void shoot();
void roller();
void roller(bool normal);