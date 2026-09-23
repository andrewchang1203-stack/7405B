#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "lift.h"

extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

extern lemlib::Chassis chassis;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern Lift lift;

extern pros::Controller controller;
extern pros::MotorGroup cascade;