#include "main.h"
#include "devices.h"

void test() {
    lift.set_zero_position(0);
    lift.move_absolute(90000, 500);
    
    
}