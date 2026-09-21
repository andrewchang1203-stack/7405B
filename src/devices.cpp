#include "main.h"

pros::MotorGroup left_motor_group({-11, 12}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({18, -19}, pros::MotorGears::blue);

pros::MotorGroup lift({-1, 10}, pros::MotorGears::blue);


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, //track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              10, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(1);
pros::Rotation horizontal_rotation(0);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_275, -5.75);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttle_curve(3,    // joystick deadband out of 127
                                    10,    //minimum output where drivetrain will move out of 127
                                        1.019  //expo curve gain
);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);     //ali code

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,
                        &throttle_curve,                  //ali code
                        &steer_curve// odometry sensors      //ali code
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics claw = pros::adi::Pneumatics('A', true);


// initialize function. Runs on program startup

