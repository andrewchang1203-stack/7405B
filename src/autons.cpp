#include "main.h"
#include "devices.h"

void far() {
    chassis.setPose(0, 0, 0);
    cascade.move_absolute(1320, 127);
    pros::delay(200);
    chassis.moveToPoint(0, -10, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 700, {.maxSpeed = 45});
    pros::delay(500);
    chassis.moveToPoint(0, -7, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 700, {.maxSpeed = 45});
    pros::delay(500);
    chassis.moveToPoint(0, -12.7, 1000, {.forwards = false});
    chassis.turnToHeading(-96, 1000);
    chassis.moveToPoint(20, -12.7, 1000, {.forwards = false, .maxSpeed = 55});
    chassis.waitUntilDone();
    cascade.move_absolute(400, 127);
    pros::delay(500);
    claw.toggle();
    pros::delay(600);
    chassis.moveToPoint(-6, -13.5, 800);
    chassis.turnToHeading(-250,800);
    chassis.moveToPoint(-21, -7, 1000, {.forwards = false, .maxSpeed = 55});
    chassis.moveToPoint(-25, -5, 1000, {.forwards = false, .maxSpeed = 40});
    chassis.waitUntilDone();
    claw.toggle();
    pros::delay(500);
    cascade.move_absolute(2420, 127);
    chassis.turnToHeading(-386, 1000);
    chassis.moveToPoint(-10, -24, 500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    claw.toggle();
    //chassis.swingToPoint(-21, -7, DriveSide::LEFT, 1000, {.forwards = false});
}

void far2(){
     chassis.setPose(0, 0, 0);
    cascade.move_absolute(1320, 127);
    pros::delay(200);
    chassis.moveToPoint(0, -10, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 700, {.maxSpeed = 45});
    pros::delay(500);
    chassis.moveToPoint(0, -7, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 700, {.maxSpeed = 45});
    pros::delay(500);
    //flip toggle
    chassis.moveToPoint(0, -12.7, 700, {.forwards = false});
    chassis.turnToHeading(-96, 1000);
    chassis.moveToPoint(10, -12.7, 500, {.forwards = false, .maxSpeed = 45});
    chassis.moveToPoint(20, -12.7, 1000, {.forwards = false, .maxSpeed = 40});
    //move to goal
    chassis.waitUntilDone();
    cascade.move_absolute(400, 127);
    pros::delay(500);
    claw.toggle();
    pros::delay(600);
    //place pin
    chassis.moveToPoint(9, -13, 800);
    chassis.turnToHeading(-27.5   ,700); 
    chassis.moveToPoint(21.7, -29.5, 1000, {.forwards = false, .maxSpeed = 45});  //22.4 -28
    chassis.waitUntilDone();
    pros::delay(200);
    claw.toggle();
    pros::delay(200);
    //get second pin
    cascade.move_absolute(2520, 127);
    chassis.turnToPoint(21.5, -12.7, 1000, {.forwards = false});
    chassis.moveToPoint(21.5, -12.7, 1000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();
    cascade.move_absolute(400, 127);
    pros::delay(500);
    claw.toggle();
    //to goal second time
    //chassis.moveToPoint(-6, -13.5, 1000);

}   

void close() {
    // chassis.setPose(0, 0, 0);
    // cascade.move_absolute(1320, 127);
    // pros::delay(200);
    // chassis.moveToPoint(0, -10, 500, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 50});
    // pros::delay(500);
    // chassis.moveToPoint(0, -7, 500, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 50});
    // pros::delay(500);
    // chassis.moveToPoint(0, -12.7, 1000, {.forwards = false});
    // chassis.turnToHeading(95, 1000);
    // chassis.moveToPoint(-20, -12.7, 1500, {.forwards = false});
    // chassis.waitUntilDone();
    // cascade.move_absolute(350, 127);
    // pros::delay(200);
    // claw.toggle();

    chassis.setPose(0, 0, 0);
    cascade.move_absolute(1320, 127);
    pros::delay(200);
    chassis.moveToPoint(0, -10, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 45});
    pros::delay(500);
    chassis.moveToPoint(0, -7, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 10, 1000, {.maxSpeed = 45});
    pros::delay(500);
    //flip toggle
    chassis.moveToPoint(0, -12.7, 1000, {.forwards = false});
    chassis.turnToHeading(95, 1000);
    chassis.moveToPoint(-20, -13.5, 2000, {.forwards = false, .maxSpeed = 55});
    chassis.waitUntilDone();
    cascade.move_absolute(400, 127);
    pros::delay(200);
    claw.toggle();
    pros::delay(600);
    chassis.moveToPoint(0, -13.5, 1000);
    
}