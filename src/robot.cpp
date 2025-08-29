//constructors for everything electrical

#include "api.h"
//#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pros/motors.h"





// 2496v
#define LF_PORT 6
#define LM_PORT 9
#define LB_PORT 8
#define RF_PORT 3
#define RM_PORT 4
#define RB_PORT 5
#define IMU_PORT 7
#define HOOKS_PORT 14
#define Mintake_PORT 20
#define FMintake_PORT 11
#define FTintake_PORT 1
#define ODOMX_PORT 18
#define ODOMY_PORT 19
#define TEST_PORT 12
#define TEST_PORT2 13
#define LDB_PORT 15
#define OPTICAL_PORT 17
#define ROTO_PORT 2




//rotation sensor
pros::Rotation roto(ROTO_PORT);
pros::Rotation ODOMX(ODOMX_PORT);
pros::Rotation ODOMY(ODOMY_PORT);

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Motor TEST (TEST_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor TEST2 (TEST_PORT2, pros::E_MOTOR_GEARSET_06, false);

//intake
pros::Motor Mintake (Mintake_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor FTintake (FTintake_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor FMintake (FMintake_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor_Group intake({FMintake, Mintake, FTintake});


//pneumatics
    //Hood
    pros::ADIDigitalOut hood ('A', false);

    //blocker
    pros::ADIDigitalOut blocker ('B', false);

    //auton selector
    pros::ADIDigitalIn selec ('F');

    pros::ADIDigitalOut colorSorter('C', false);

    pros::ADIDigitalOut scraper ('D', false);



//sensors
// pros::ADIEncoder encLeft ({{1, 1, 2}, false});
// pros::ADIEncoder encRight ({{1, 3, 4}, false});
// pros::ADIEncoder encMid ({{1, 5, 6}, false});

pros::Imu imu (IMU_PORT);

//controller
pros::Controller con (pros::E_CONTROLLER_MASTER);

//optical sensor
pros::Optical OpticalC (OPTICAL_PORT);

const unsigned int PutPct[101] = {
  0,  1,  2,  3,  4,  5,  5,  6,  6,  7,  
  7 , 7,  8,  8,  9,  9,  10, 11, 12, 13, 
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
  6 , 6,  7,  7,  8,  8,   9, 10, 11, 12, 
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

int controlmode = 1;
int testmode=0;
double speedmode = 0;
int count =0;

char zhanwei='R';

int flage = 1;
int flage1 = 1;
int flage2 = 0;
int flage3 = 0;
int flage4=0;
int flage5=0;
int flage6=0;
int flage7=0;
int flage8=0;
int flage9=0;
int flage10=0;
bool flage11=true;
int flage12=2;
int flage13=0;
int flage14=0;
int flage15=0;
int flage16=0;
int flage17=0;
int flage18=1;
int flage19=0;
int flage20=0;
int flage21=0;
int flage22=0;
int flage23=0;
int flage24=0;
int flage25=0;
int flage26=1;
int flage27=0;
int fenhuankg=1;
double X_position=0.0;
double Y_position=0.0;
double  WheelRadius=1;//从动轮半径（英寸）
double  Wheel_X_distance=0.5;//横向从动轮轮到旋转中线的位置
double  Wheel_Y_distance=2;//纵向从动轮轮到旋转中心的位置
double  Gyrobefore_rad=0.0;

double dis=0;

double initialdis=78;
int side=2;
int auton = 0;
int Roller_auto1_1=0;
int Roller_auto2_1=0;
int Roller_auto3_1=0;
int CD=0;

double return_angle=imu.get_rotation();
double initial_degree = 0;

Timer T1,T2,T3,T4,T5;

Timer TAUTO1_1,TAUTO1_2,TAUTO2_1,TAUTO2_2,TAUTO3_1,TAUTO3_2;
Timer TAUTO66_3,TACC,FINISH;
Timer TURN1,TURN2;

//gps
// #define X_OFFSET .225
// #define Y_OFFSET .223
// #define X_INITIAL 1.54
// #define Y_INITIAL 1.14
// #define HEADING_INITIAL 90
// pros::Gps gps1(GPS_PORT, X_INITIAL, Y_INITIAL, HEADING_INITIAL, X_OFFSET, Y_OFFSET);
// pros::c::gps_status_s_t gpsData;

// pros::Vision vision (18);