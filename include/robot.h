//externing to make sure that these electrical component declarations are defined (the names can't be used more than once) and will be used multiple times in multiple files
//extern: external linkage

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"



#ifndef ROBOTH
#define ROBOTH

//controller
extern pros::Controller con;

//rotation sensor
extern pros::Rotation roto;
extern pros::Rotation ODOMX;
extern pros::Rotation ODOMY;

//chassis motors
extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RB;
extern pros::Motor RM;
extern pros::Motor LM;

extern pros::Motor TEST;
extern pros::Motor TEST2;

//lift
extern pros::Motor Mintake;

//hooks
extern pros::Motor FTintake;
extern pros::Motor FMintake;

extern pros::Motor HOOKS;
extern pros::Motor LadyBrown;


//sensors
// extern pros::ADIEncoder encLeft;
// extern pros::ADIEncoder encRight;
// extern pros::ADIEncoder encMid;

//imu
extern pros::Imu imu;

//pneumatics
extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut doinkerClamp;
extern pros::ADIDigitalOut doinker;
extern pros::ADIDigitalOut hood;
extern pros::ADIDigitalOut blocker;
extern pros::ADIDigitalOut hang;
extern pros::ADIDigitalOut colorSorter;
extern pros::ADIDigitalOut scraper;
//gps
// extern pros::Gps gps1;
// extern pros::c::gps_status_s_t gpsData;

//auton selector
extern pros::ADIDigitalIn selec;

extern pros::Optical OpticalC;

//vision
// extern pros::Vision vision;

extern const unsigned int PutPct[];
extern const unsigned int PutVol[];

class PID
{
public:
  float error = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float starti = 0;
  float settle_error = 0;
  float settle_time = 0;
  float timeout = 0;
  float accumulated_error = 0;
  float previous_error = 0;
  float output = 0;
  float time_spent_settled = 0;
  float time_spent_running = 0;
  float update_period = 10;

  PID(float error, float kp, float kd, float starti);

  PID(float error, float kp, float ki, float kd, float starti);

  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout);

  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout, float update_period);

  float compute(float error);

  bool is_settled();
};

extern double  X_position;
extern double  Y_position;

#endif