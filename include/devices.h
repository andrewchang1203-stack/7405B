#pragma once

#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "lift.h"


// motors
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup cascade;

// sensors
extern pros::Imu imu;
extern pros::Rotation horizontal_rotation;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::OdomSensors sensors;

// lemlib config
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::Chassis chassis;

// controller and pneumatics
extern pros::Controller controller;
extern pros::adi::Pneumatics claw;