#pragma once
#include "main.h"
#include "lemlib/api.hpp"  // provides lemlib::PID
#include <atomic>
#include <vector>

/**
 * PID-controlled cascade lift (winch driven).
 *
 * - Two motors in a pros::MotorGroup (negative port = reversed)
 * - One V5 Rotation Sensor on the winch shaft, read in DEGREES.
 *   get_position() is cumulative, so it keeps counting past 360 (multiple turns).
 * - lemlib::PID does the math; a background task runs it every 10 ms.
 *
 * ---------------------------------------------------------------------------
 * USAGE (main.cpp)
 * ---------------------------------------------------------------------------
 *   // ports {1, -2} = two lift motors (2 reversed), rotation sensor on port 3
 *   //             kP    kI   kD   windup  min  max   (degrees of winch rotation)
 *   Lift lift({1, -2}, 3, 0.6, 0.0, 3.0, 20.0, 0, 720);
 *
 *   void initialize() {
 *       // ...lemlib chassis calibrate, etc...
 *       lift.start();               // lift must be fully DOWN at boot
 *   }
 *
 *   void opcontrol() {
 *       pros::Controller master(pros::E_CONTROLLER_MASTER);
 *       while (true) {
 *           if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) lift.setTarget(720); // up
 *           if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) lift.setTarget(0);   // down
 *           if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) lift.nudge(+4);  // fine adjust up
 *           if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) lift.nudge(-4);  // fine adjust down
 *           pros::delay(10);
 *       }
 *   }
 *
 *   // In autonomous:
 *   //   lift.setTarget(720);
 *   //   lift.waitUntilSettled(1500);
 * ---------------------------------------------------------------------------
 */
class Lift {
  public:
    Lift(std::vector<int8_t> motorPorts, int8_t rotationPort,
         float kP, float kI, float kD, float windupRange,
         double minDeg, double maxDeg);

    /** Zero the sensor (lift must be fully lowered) and start the PID task. */
    void start();

    /** Set target in degrees of winch rotation. Clamped to [minDeg, maxDeg]. */
    void setTarget(double degrees);

    /** Move the current target by a small amount (for manual fine control). */
    void nudge(double deltaDegrees);

    double getTarget() const;
    double getPosition();  // degrees

    bool atTarget(double tolerance = 8.0);

    /** Block until at target or timeout (ms). Handy in autonomous. */
    void waitUntilSettled(int timeoutMs = 2000, double tolerance = 8.0);

    // Tunables you can tweak after construction
    float kG = 0.0f;          // constant "hold against gravity" power (0-127) added when lift is raised
    int maxUpPower = 127;     // limit power going up
    int maxDownPower = 80;    // limit power going down (prevents slack/tangled string)

  private:
    void loop();

    pros::MotorGroup motors;
    pros::Rotation rotation;
    lemlib::PID pid;

    std::atomic<double> target{0.0};
    double minDeg;
    double maxDeg;
};