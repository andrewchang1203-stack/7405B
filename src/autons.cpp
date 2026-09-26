#include "main.h"
#include "devices.h"

void test() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.turnToHeading(90, 3000);
    cascade.move_absolute(720, 127);
    pros::delay(200);
    chassis.moveToPoint(0, 5, 500);
    chassis.moveToPoint(0, -20, 1000, {.forwards = false, .minSpeed = 127});
    pros::delay(500);
    chassis.moveToPoint(0,8.3,1000);
    chassis.turnToHeading(90,1000);
    
    // chassis.moveToPoint(0, 30, 1000);
}