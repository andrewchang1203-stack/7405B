#pragma once
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "lemlib/api.hpp"



extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

extern lemlib::Chassis chassis;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern pros::Motor leftLift;
extern pros::Motor rightLift;

extern pros::Controller controller;