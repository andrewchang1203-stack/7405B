#include "lift.h"
#include <algorithm>
#include <cmath>

Lift::Lift(std::vector<int8_t> motorPorts,      //ports of the lift motors
                        int8_t rotationPort,    //port of the rotation sensor
                        float kP,               //p of pid
                        float kI,               //i of pid (should be 0 bc i is a chud)
                        float kD,               //d of pid
                        float windupRange,      //windup of i (should be 0)
                        double minDeg,          //minimum amount of rotation degrees (so bot doesn't kill itself)
                        double maxDeg)          //max amount rotation degrees (so bot doesn't kill itself)
    : motors(motorPorts),
      rotation(rotationPort),
      pid(kP, kI, kD, windupRange, true),  // true = reset integral when error crosses zero
      minDeg(minDeg),
      maxDeg(maxDeg) {}

void Lift::start() {
    motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  // helps the lift stay put when idle
    rotation.reset_position();
    pros::delay(50);  // give the sensor a moment to zero
    target = minDeg;
    pid.reset();

    // Runs forever in the background; safe to leave as a local object.
    pros::Task liftTask([this]() { this->loop(); });
}

void Lift::setTarget(double degrees) {
    target = std::clamp(degrees, minDeg, maxDeg);
}

void Lift::nudge(double deltaDegrees) {
    setTarget(target.load() + deltaDegrees);
}

double Lift::getTarget() const { return target.load(); }

// cumulative angle in degrees (sensor reports centidegrees)
double Lift::getPosition() { return rotation.get_position() / 100.0; }

bool Lift::atTarget(double tolerance) {
    return std::abs(target.load() - getPosition()) < tolerance;
}

void Lift::waitUntilSettled(int timeoutMs, double tolerance) {
    int elapsed = 0;
    while (!atTarget(tolerance) && elapsed < timeoutMs) {
        pros::delay(10);
        elapsed += 10;
    }
}

void Lift::loop() {
    while (true) {
        double pos = getPosition();

        // Safety: if the sensor reads way out of range (unplugged, wrong direction,
        // string snapped, etc.) stop driving instead of running away.
        if (pos < minDeg - 60 || pos > maxDeg + 60) {
            motors.brake();
            pros::delay(10);
            continue;
        }

        double error = target.load() - pos;
        double output = pid.update(error);

        // Feed-forward to fight gravity once the lift is off the bottom
        if (target.load() > minDeg + 10) output += kG;

        output = std::clamp(output, (double)-maxDownPower, (double)maxUpPower);

        // Sitting at the bottom: just let the brake hold it
        if (target.load() <= minDeg + 2 && pos < minDeg + 10) {
            motors.brake();
            pid.reset();
        } else {
            motors.move(static_cast<int>(output));
        }

        pros::delay(10);  // keep loop time constant so the D term is meaningful
    }
}
