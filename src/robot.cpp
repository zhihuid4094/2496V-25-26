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
#define ROTO_PORT 2
#define Distance_PORTL 16
#define Distance_PORTR 9

// rotation sensors
pros::Rotation roto(ROTO_PORT);
pros::Distance distanceSensorL(Distance_PORTL);
pros::Distance distanceSensorR(Distance_PORTR);

// chassis motors
pros::Motor LF(LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM(LM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB(LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF(RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM(RM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB(RB_PORT, pros::E_MOTOR_GEARSET_06, false);

// intake motors
pros::Motor Rintake(Rintake_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Lintake(Lintake_PORT, pros::E_MOTOR_GEARSET_06, false);

// pneumatics
pros::ADIDigitalOut lift('B', false);
pros::ADIDigitalOut pto('D', false);
pros::ADIDigitalOut descore('E', false);
pros::ADIDigitalOut park('G', false);
pros::ADIDigitalOut scraper('C', false);

// sensors
pros::Imu imu(IMU_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);
