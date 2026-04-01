#include "api.h"
#include "main.h"
#include "robot.h"
#include "pros/motors.h"

// 2496v
#define LF_PORT 12
#define LM_PORT 13
#define LB_PORT 11
#define RF_PORT 18
#define RM_PORT 19
#define RB_PORT 20
#define IMU_PORT 6
#define Lintake_PORT 17
#define Rintake_PORT 7
#define ODOMX_PORT 16
#define ODOMY_PORT 1
#define TEST_PORT 5
#define TEST_PORT2 6
#define OPTICAL_PORT 8
#define ROTO_PORT 2
#define Distance_PORTL 16
#define Distance_PORTR 9

// rotation sensors
pros::Rotation roto(ROTO_PORT);
pros::Rotation ODOMX(ODOMX_PORT);
pros::Rotation ODOMY(ODOMY_PORT);
pros::Distance distanceSensorL(Distance_PORTL);
pros::Distance distanceSensorR(Distance_PORTR);

// chassis motors
pros::Motor LF(LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM(LM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB(LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF(RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM(RM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB(RB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Motor TEST(TEST_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor TEST2(TEST_PORT2, pros::E_MOTOR_GEARSET_06, false);

// intake motors
pros::Motor Rintake(Rintake_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Lintake(Lintake_PORT, pros::E_MOTOR_GEARSET_06, false);

// pneumatics
pros::ADIDigitalOut lift('B', false);
pros::ADIDigitalOut pto('D', false);
pros::ADIDigitalOut descore('E', false);
pros::ADIDigitalIn  selec('F');
pros::ADIDigitalOut park('G', false);
pros::ADIDigitalOut scraper('C', false);

// sensors
pros::Imu imu(IMU_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);
pros::Optical OpticalC(OPTICAL_PORT);

// motor power lookup tables
const unsigned int PutPct[101] = {
    0,  1,  2,  3,  4,  5,  5,  6,  6,  7,
    7,  7,  8,  8,  9,  9, 10, 11, 12, 13,
   14, 15, 16, 16, 17, 18, 19, 20, 22, 23,
   22, 24, 26, 28, 28, 29, 30, 31, 31, 32,
   33, 34, 35, 36, 37, 38, 39, 40, 40, 41,
   41, 41, 42, 43, 44, 45, 46, 46, 47, 48,
   49, 50, 51, 52, 55, 57, 59, 61, 63, 65,
   67, 69, 71, 73, 75, 77, 79, 81, 83, 85,
   85, 88, 88, 91, 91, 94, 94, 95, 97, 97,
  100,
};
const unsigned int PutVol[101] = {
    0,  1,  2,  3,  4,  4,  4,  5,  5,  5,
    6,  6,  7,  7,  8,  8,  9, 10, 11, 12,
   12, 14, 14, 16, 16, 18, 18, 20, 20, 22,
   22, 24, 24, 26, 26, 27, 27, 28, 28, 29,
   29, 30, 31, 31, 32, 32, 32, 33, 33, 34,
   34, 35, 35, 36, 37, 37, 38, 39, 40, 40,
   41, 41, 42, 43, 44, 45, 46, 46, 47, 48,
   49, 50, 51, 52, 55, 57, 59, 61, 63, 65,
   67, 69, 71, 73, 75, 77, 79, 81, 83, 85,
   85, 88, 88, 91, 91, 94, 94, 95, 97, 97,
  100
};

// state variables
int controlmode = 1;
double speedmode = 0;
int count = 0;

// timers
Timer T1, T2, T3, T4, T5;
Timer TACC, FINISH, TURN1, TURN2;