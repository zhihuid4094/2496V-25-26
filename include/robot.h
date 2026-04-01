#ifndef ROBOTH
#define ROBOTH

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

// controller
extern pros::Controller con;

// rotation sensors
extern pros::Rotation roto;
extern pros::Rotation ODOMX;
extern pros::Rotation ODOMY;

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

extern pros::Motor TEST;
extern pros::Motor TEST2;

// intake/mechanism motors
extern pros::Motor Lintake;
extern pros::Motor Rintake;
extern pros::Motor HOOKS;
extern pros::Motor LadyBrown;

// imu
extern pros::Imu imu;

// pneumatics
extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut doinkerClamp;
extern pros::ADIDigitalOut doinker;
extern pros::ADIDigitalOut hood;
extern pros::ADIDigitalOut blocker;
extern pros::ADIDigitalOut hang;
extern pros::ADIDigitalOut colorSorter;
extern pros::ADIDigitalOut scraper;
extern pros::ADIDigitalOut linkage;
extern pros::ADIDigitalOut descore;
extern pros::ADIDigitalOut pto;
extern pros::ADIDigitalOut lift;
extern pros::ADIDigitalOut park;

// sensors
extern pros::ADIDigitalIn selec;
extern pros::Optical OpticalC;

// state variables
extern bool initializing;
extern bool SCREEN_ON;
extern int controlmode;
extern double speedmode;
extern int count;
extern bool auto_manual_select;

// constants
extern const unsigned int PutPct[];
extern const unsigned int PutVol[];

// timer struct
struct Timer {
    unsigned long start = 0;
    void reset() { start = pros::millis(); }
    unsigned long elapsed() { return pros::millis() - start; }
};

extern Timer T1, T2, T3, T4, T5;
extern Timer TACC, FINISH, TURN1, TURN2;

// math helpers
extern int sgn(double number);
extern float reduce_0_to_360(float angle);
extern float reduce_negative_180_to_180(float angle);
extern float to_rad(float angle_deg);
extern float to_deg(float angle_rad);
extern float clamp(float input, float min, float max);
extern bool is_reversed(double input);
extern float to_volt(float percent);
extern double calculateAngleError(double targetAngle, double currentAngle);

// motor control
extern void Left(int speedpct);
extern void Right(int speedpct);
extern void LeftVol(int vol_input);
extern void RightVol(int vol_input);
extern void RunVol(int basepct);
extern void TurnVol(int turnpct);
extern void RunVolAuto(int basepct, int outtime);
extern void TurnVolAuto(int basepct, int outtime);
extern void BaseMotorStop(int mode);
extern void IntakeStop(int mode);
extern void IntakeReset();

// autons
extern void auto1();
extern void auto2();
extern void auto3();
extern void auto4();
extern void auto5();
extern void auto6();
extern void auto7();
extern void auto8();
extern void auto9();
extern void auto10();
extern void auto11();
extern void autoA();
extern void autoB();

#endif