#ifndef ROBOTH
#define ROBOTH

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

// controller
extern pros::Controller con;

// rotation sensors
extern pros::Rotation roto;

// distance sensors
extern pros::Distance distanceSensorL;
extern pros::Distance distanceSensorR;

// chassis motors
extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RB;
extern pros::Motor RM;
extern pros::Motor LM;


// intake/mechanism motors
extern pros::Motor intake;
extern pros::Motor Lever;

// imu
extern pros::Imu imu;

// pneumatics
extern pros::ADIDigitalOut scraper;
extern pros::ADIDigitalOut descore;
extern pros::ADIDigitalOut lift;
extern pros::ADIDigitalOut blocker;
 

// constants
extern const unsigned int PutPct[];
extern const unsigned int PutVol[];

// timer struct
struct Timer {
    unsigned long start = 0;
    void reset() { start = pros::millis(); }
    unsigned long elapsed() { return pros::millis() - start; }
};



#endif