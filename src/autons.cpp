#include "main.h"
#include "devices.h"

void far() {
    chassis.setPose(0, 0, 0);
    cascade.move_absolute(1320, 127);
    pros::delay(200);
    chassis.moveToPoint(0, -10, 500, {.forwards = false});
    chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 70});
    pros::delay(500);
    chassis.moveToPoint(0, -12.5, 1000, {.forwards = false});
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(20, -12.5, 1500, {.forwards = false});
    chassis.waitUntilDone();
    cascade.move_absolute(0, 127);
    claw.toggle();
}

void close() {
    chassis.setPose(0, 0, 0);
    cascade.move_absolute(1320, 127);
    pros::delay(200);
    chassis.moveToPoint(0, -10, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 50});
    pros::delay(500);
    chassis.moveToPoint(0, -7, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 50});
    pros::delay(500);
    chassis.moveToPoint(0, -12.7, 1000, {.forwards = false});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-20, -12.7, 1500, {.forwards = false});
    chassis.waitUntilDone();
    cascade.move_absolute(350, 127);
    pros::delay(200);
    claw.toggle();
}