#include "main.h"
#include "lemlib/api.hpp"
#include <algorithm>
#include <cmath>

#include "autons.h"

void nearSide(){
      chassis.setPose(0, 0, 45);
    //bakc it up
    chassis.moveToPoint(-18, -12, 2000, {.forwards = false});
    chassis.moveToPoint(-28, -12, 2000, {.forwards = false});

    chassis.moveToPoint(-20, -12, 2000);
    chassis.turnToHeading(-90, 1200);

    chassis.moveToPoint(-8, 4, 2000, {.forwards = false});
    chassis.waitUntil(3);
    LBWing.set_value(true);
    chassis.waitUntil(16);
    LBWing.set_value(false);
    chassis.moveToPoint(0, 4, 2000, {.forwards = false});


    chassis.turnToHeading(180, 1200);
    chassis.moveToPoint(0, 33, 3000, {.forwards = false});
    // chassis.turnToHeading(180, 1200);
    // LBWing.set_value(true);
    // chassis.turnToHeading(190, 1200);
    // chassis.turnToHeading(30, 1200);
    // chassis.turnToHeading(45, 1200);
    // LBWing.set_value(true);
}

void farSide(){

}

void safeFarSide(){

}

void skills(){

}