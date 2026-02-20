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

//hooks
extern pros::Motor Lintake;
extern pros::Motor Rintake;
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
extern pros::ADIDigitalOut linkage;
extern pros::ADIDigitalOut descore;
extern pros::ADIDigitalOut pto;
extern pros::ADIDigitalOut lift;
extern pros::ADIDigitalOut park;

//gps
// extern pros::Gps gps1;
// extern pros::c::gps_status_s_t gpsData;

//auton selector
extern pros::ADIDigitalIn selec;

extern pros::Optical OpticalC;

//vision
// extern pros::Vision vision;


// State variables
extern bool initializing;
extern bool SCREEN_ON;
extern bool BSCREEN_ON;
extern bool CSCREEN_ON;
extern int controlmode;
extern int testmode;
extern double speedmode;
extern int count;
extern bool auto_manual_select;


// Constants
extern const unsigned int PutPct[];
extern const unsigned int PutVol[];

struct Timer {
    unsigned long start = 0;

    void reset() { start = pros::millis(); }
    unsigned long elapsed() { return pros::millis() - start; }
};
// Timers

extern Timer T1, T2, T3, T4, T5;
extern Timer TAUTO1_1, TAUTO1_2, TAUTO2_1, TAUTO2_2;
extern Timer TAUTO3_1, TAUTO3_2, TAUTO66_3;
extern Timer TACC, FINISH, TURN1, TURN2;
extern Timer T_Roller;

// Positions and angles
extern double X_position;
extern double Y_position;
extern double WheelRadius;
extern double Wheel_X_distance;
extern double Wheel_Y_distance;
extern double Gyrobefore_rad;
extern double return_angle;

// PID functions
extern int sgn(double number);
extern int setMotorPct(int Value);

// Motor control functions
extern void Left(int speedpct);
extern void Right(int speedpct);
extern void LeftVol(int vol_input);
extern void RightVol(int vol_input);
extern void Run(int basepct);
extern void RunVol(int basepct);
extern void Turn(int turnpct);
extern void Intake(int getpct);
extern void UpDown(int uppct);
extern void Push(int pushpct);
extern void Clip(int clippct);
extern void Flywheel(int flywheelpct);
extern void FlywheelVel(int vel_input);
extern void FlywheelVol(int vol_input);
extern void IntakeVol(int vol_input);
extern void PushVol(int vol_input);
extern void Roller(int vol_input);
extern void UpdownVol(int vol_input);
extern void CatapultVol(int vol_input);

extern void RunAuto(int basepct, int outtime);
extern void RunAuto2(int basepct,int basepct2, int outtime,int Ipec1=0,int Ipec2=0);
extern void RunVolAuto(int basepct, int outtime);
extern void TurnAuto(int basepct, int outtime);
extern void IntakeAuto(int getpct, int outtime);
extern void RollerAuto(int basepct, int outtime);
extern void UpDownAuto(int uppct, int outtime);

extern void PushAuto(int pushpct2, int pushtime);
extern void ClipAuto(int clippct2, int cliptime);
extern void CatapultAuto(int basepct, int outtime);
extern void FlywheelAuto(int flywheelpct, int flywheeltime);
extern void FlywheelVelAuto(int flywheelpct, int outtime);
extern void FlywheelVolAuto(int flywheelpct, int outtime);

extern void UpDownencode(int pct,int degree,int outtime);
// extern void UpDownencodeTo(int pct, int degree);

extern void Pushencode(int pct, int degree);
extern void Runencode(int pct,int degree,int outtime=4000);
extern void Intakeencode(int pct, int degre);
extern void Clipencode(int pct, int degree);

extern void UpDowntask(int degree);
extern void Runpid(int maxpct, int encode, int outtime);
extern void Pushtask(int degree,int pct1);

extern void Runpid(int maxpct, int encode, int outtime);
extern void Turnpid(int maxpct, int encode, int outtime) ;
extern void Flywheelpid(int frpm, int change_steps=-1, int start_point=0) ;
extern void Flywheelvolpid(int frpm, int gear=1,int change_steps=-1, int start_point=0);

extern void auto2();
extern void auto1();
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


float reduce_0_to_360(float angle);

float reduce_negative_180_to_180(float angle);

float reduce_negative_90_to_90(float angle);

float to_rad(float angle_deg);

float to_deg(float angle_rad);
extern void LeftCircle(int maxpower, int deg1, int outtime);


extern void RunXY(double speed_limit, double targetX, double targetY,double err_1, double outtime);
extern void UpdateXY();
extern void RunXY_Straight(double speed_limit, double targetX, double targetY,double err_1, double outtime,int p_point);
extern void FaceToXY(double speed_limit, double targetX, double targetY,double err_1, double outtime);

 

extern void RunpidStraight(double speed_limit=70,int aim=100,double newgyro=0,double speed_limit2=40,int dec_point=-1,int change_steps=-1,int start_point=0,int outtime=1800,int p_point=0);
extern void RunpidStraightTo(double speed_limit=70,int aim=100,double newgyro=0,double speed_limit2=40,int dec_point=-1,int change_steps=-1,int start_point=0,int outtime=1800,int p_point=-1);
extern void Run_spin_pid(double speed_limit, int aim,double err_1,double speed_limit2, int dec_point, int change_steps,int start_point,int outtime, double newgyro, int p_point,int turn_point,int turn_side=2,int turn_right=0);
extern void RunpidsinleN(double speed_limitL=100,double speed_limitR=50, int aim=500,int change_steps=-1,int start_point=0,double Nerr=8,int outtime=5500);
extern void just_stop(int stopmod=1);
extern void RunpidTo(double speed_limit, int aim,double err_1,double speed_limit2, int dec_point, int change_steps,int start_point,int outtime, double newgyro, int p_point,int turn_point,int turn_side=2,int turn_right=0);

extern void RunLine(int v=80, int colour=60 ,int at_least_time=0);
extern void RunLineStraight(int v=80, int colour=60 ,int at_least_S=200);

extern double returnangle;

extern void TurnencodepidN(int aim,int newgyro,double speed_limit=40,double speed_limit2=30,int dec_point=-1,bool (*pf)(void)=NULL,int start_point=0);
extern void TurnpidSingle(int max_speed=100,double aim=0,double B=0.2,bool side=0, double Kp=2.4,int error=1,int switch_left=1,int switch_right=1,int outtime=900);
extern void TurnpidSingleTo(int max_speed=100,double aim=0,double B=0.2,bool side=0, double Kp=2.4,int error=1,int switch_left=1,int switch_right=1 ,int outtime=900 );
extern void RunAutoStraight(int v,int time);
extern void RunAutoStraightTo(int v, int time,int angle) ;

extern void just_run_straight(double output,double newgyro);
extern void just_run_straightTo(double output,double newgyro);

extern void IntakeReset();
extern void IntakeencodeTo(int pct,int degree);
extern void IntakeencodeFor(int pct,int degree);
extern void Intakeencode(int pct, int degree);

extern void PushencodeFor(int pct,int degree);
extern void PushencodeTo(int pct,int degree);

extern void UpDownencodeTo(int pct, int degree);
extern void UpDownencodeFor(int pct,int degree);

extern double SpeedLimit(int speed,int max=100);

extern void PIDmove(int Aim,int maxspeed=100,int StartIntegrate=100,int AccTime=500,int tolerance=8);
extern void PIDturn_1(int Aim);
extern void PIDTurn_encoder(int goal);

extern void front_sonar_run(int distance,double direction,int speed_limit=90,bool limit_acc=true);

extern void RunTouch(int v=60,int outtime=2500);
// extern void RunSonarTo(int distance=200,int speed_limit=80,double direction=0,bool limit_acc=0);
// extern void RunSonar(int distance=200,int speed_limit=80,double direction=0,bool limit_acc=0);

extern void SonarPID(int Aim=2000,int maxspeed=60);
void RunSonarTo(double speed_limit=80,int distance=200,int err__1=20 , int change_steps=-1,int start_point=0,double newgyro=0,int mode =1,int outtime=5000);
void RunSonar(int distance=200,int speed_limit=80,double direction=0,int err__1=20,bool limit_acc=0, int change_steps=-1,int start_point=0);
void RunSonarbackTo(int distance=200,int speed_limit=80,double direction=0,int err__1=20 ,bool limit_acc=0, int change_steps=-1,int start_point=0);
void RunSonarback(int distance=200,int speed_limit=80,double direction=0,int err__1=20,bool limit_acc=0, int change_steps=-1,int start_point=0);
void RunSonar_back_2(int distance=200,int speed_limit=80,double direction=0,int err__1=20,bool limit_acc=0, int change_steps=-1,int start_point=0);

extern void drive_to_white_line(int speed=50,int outtime=5000);

extern void RunpidStraightN(double speed_limit=70, int aim=100,double err_1=5,double speed_limit2=40, int dec_point=-1, int change_steps=-1,int start_point=0,int outtime=18000,double newgyro=0,int p_point=-1);
//速度，距离，误差值，第二速度，减速点，运行多线程序号，运行线程距离，保护时间，修正方向,pid参数选择
extern void RunpidStraightNTo(double speed_limit=70, int aim=100,double err_1=5,double speed_limit2=40, int dec_point=-1, int change_steps=-1,int start_point=0,int outtime=18000,double newgyro=0,int p_point=-1);

extern void RunpidStraightNToHC(double speed_limit=70, int aim=100,double err_1=5,double speed_limit2=40, int dec_point=-1, int change_steps=-1,int start_point=0,int outtime=18000,double newgyro=0,int p_point=-1);
//速度，距离，误差值，第二速度，减速点，运行多线程序号，运行线程距离，保护时间，修正方向（方位）,pid参数选择
extern void TurnpidN(int max_speed=100,double aim=0,double howerr=0.7,double goals=0,int outtime=12000); 
//速度，角度，误差，保护时间
extern void TurnpidNTo(int max_speed=100,double aim=0,double howerr=0.7,int outtime=12000);
//速度，角度（方位），误差，保护时间
extern void TurnpidLNTo(int max_speed=100,double min_speed=0,double aim=0,double howerr=0.7,int outtime=12000);

extern void RunVolpid(double aim=0,double threshold=0,double targetangle=0);

extern void TurnVolpidNTo(int max_speed=100, double aim=0, double howerr=0.7, int outtime=12000,int p_point=-1);

extern void TurnVolpidN(int max_speed=100, double aim=0, double howerr=0.7, int outtime=12000,int p_point=-1);

extern void TurnVolpidSingle(int max_speed=100, double aim=0, double B=1,bool side=1,double Kp=0.7, int error=1,int switch_left=0, int switch_right=0,int outtime=4000);

extern void fPIDTurn(double dDegrees, double P, double D);

extern void runturnvolpid(double aim=0, double threshold=10.0,double targetangle=0);

extern void RunVolpid(double aim,double newgyro,double threshold);

extern void set_location(float X_value,float Y_value);

////////////////////////////////////////////////////////////////////////////////////////
float reduce_0_to_360(float angle);

float clamp(float input, float min, float max);

bool is_reversed(double input);

float to_volt(float percent);



bool is_line_settled(float desired_X, float desired_Y, float desired_angle_deg, float current_X, float current_Y);

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

  PID(float error, float kp, float ki, float kd, float starti);

  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout);

  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout, float update_period);

  float compute(float error);

  bool is_settled();
};

float clamp_min_voltage(float drive_output, float drive_min_voltage);

float left_voltage_scaling(float drive_output, float heading_output);

float right_voltage_scaling(float drive_output, float heading_output);

extern void drive_to_point(float X_target, float Y_target, float drive_min_speed, float drive_max_speed, float heading_max_speed, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_starti, float heading_starti, int p_point, int steps, float b_point);

extern void drive_to_pose(double X_target, double Y_target, double angle_target, double lead, double setback, double drive_min_speed, double drive_max_speed, double heading_max_speed, double drive_settle_error, double drive_settle_time, double drive_timeout, double drive_starti, double heading_starti, int p_point);

extern double initial_degree;

extern double calculateAngleError(double targetAngle , double currentAngle);

extern void UpDownencodeNTo(int pct=100, int degree=0,int err=2,int outtime=2000, int p_point=1);

extern  void IntakeencodeNTo(int pct, int degree,int err=2, int outtime=1000, int p_point=1);
#endif