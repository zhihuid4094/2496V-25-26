#include "robot.h"
#include "pros/vision.hpp"

using namespace pros;
using namespace std;

extern pros::Motor_Group intake;

int steps=0;

void set_location(float X_value,float Y_value){
X_position=X_value;
Y_position=Y_value;
}

float compute(float error){
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float accumulated_error = 0;
  float previous_error = 0;
  float output = 0;
  if (fabs(error) < 100){
    accumulated_error+=error;
  }
  if ((error>0 && previous_error<0)||(error<0 && previous_error>0)){ 
    accumulated_error = 0; 
  }
  output = kp*error + ki*accumulated_error + kd*(error-previous_error);

  previous_error=error;

  return output;
}

//getting the x position
float get_X_position(){
  return X_position;
}

// getting the y position
float get_Y_position(){
  return Y_position;
}

//making sure angle is within -180 and 180
float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { 
        angle += 360; 
    }
    if(angle >= 180) { 
        angle -= 360; 
    }
  }
  return(angle);
}

//making sure angle is within -90 and 90
float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { 
        angle += 180; 
    }
    if(angle >= 90) { 
        angle -= 180;
     }
  }
  return(angle);
}

//angle to radians
float to_rad(float angle_deg){
  return(angle_deg/(180.0/M_PI));
}

//radians to angle
float to_deg(float angle_rad){
  return(angle_rad*(180.0/M_PI));
}

//getting the sign
int sgn(double number) {

  if (number < 0)
    return -1;
  else if (number == 0)
    return 0;
  else
    return 1;
}

//setting motor power
int setMotorPct(int Value) {
  if (Value < 0) {
    Value = -PutPct[-Value];
  } else if (Value > 0) {
    Value = PutPct[Value];
  }
  return 0;
}

//setting the motor voltage
int setMotorVol(int Value) {
  if (Value < 0) {
    Value = -PutVol[-Value];
  } else if (Value > 0) {
    Value = PutVol[Value];
  }
  return 0;
}

//all motors stop
void AllMotorStop(int mode = 0) {
  if (mode == 1) {
    LF.move(0);
    LF.set_brake_mode(MOTOR_BRAKE_BRAKE);
    LM.move(0);
    LM.set_brake_mode(MOTOR_BRAKE_BRAKE);
    LB.move(0);
    LB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    RF.move(0);
    RF.set_brake_mode(MOTOR_BRAKE_BRAKE);
    RM.move(0);
    RM.set_brake_mode(MOTOR_BRAKE_BRAKE);
    RB.move(0);
    RB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    Mintake.move(0);
    Mintake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    FTintake.move(0);
    FTintake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    FMintake.move(0);
    FMintake.set_brake_mode(MOTOR_BRAKE_BRAKE);
  }
  else {
    LF.move(0);
    LF.set_brake_mode(MOTOR_BRAKE_COAST);
    LM.move(0);
    LM.set_brake_mode(MOTOR_BRAKE_COAST);
    LB.move(0);
    LB.set_brake_mode(MOTOR_BRAKE_COAST);
    RF.move(0);
    RF.set_brake_mode(MOTOR_BRAKE_COAST);
    RM.move(0);
    RM.set_brake_mode(MOTOR_BRAKE_COAST);
    RB.move(0);
    RB.set_brake_mode(MOTOR_BRAKE_COAST);
    Mintake.move(0);
    Mintake.set_brake_mode(MOTOR_BRAKE_COAST);
    FTintake.move(0);
    FTintake.set_brake_mode(MOTOR_BRAKE_COAST);
    FMintake.move(0);
    FMintake.set_brake_mode(MOTOR_BRAKE_COAST);

    // flywheel1.stop(brake);
    // clip.stop(coast);
    // push.stop(coast);
    // push1.stop(coast);
    // updown1.stop(coast);
    // ////catapult.stop(coast);
    // flywheel1.stop(coast);
    // flywheel2.stop(coast);
  }
}

//stopping all chassis motors
void BaseMotorStop(int mode = 1) {
  if (mode == 2) {
    LF.move(0);
    LF.set_brake_mode(MOTOR_BRAKE_HOLD);
    LM.move(0);
    LM.set_brake_mode(MOTOR_BRAKE_HOLD);
    LB.move(0);
    LB.set_brake_mode(MOTOR_BRAKE_HOLD);
    RF.move(0);
    RF.set_brake_mode(MOTOR_BRAKE_HOLD);
    RM.move(0);
    RM.set_brake_mode(MOTOR_BRAKE_HOLD);
    RB.move(0);
    RB.set_brake_mode(MOTOR_BRAKE_HOLD);
  }
  else if (mode == 1) {
    LF.move(0);
    LF.set_brake_mode(MOTOR_BRAKE_BRAKE);
    LM.move(0);
    LM.set_brake_mode(MOTOR_BRAKE_BRAKE);
    LB.move(0);
    LB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    RF.move(0);
    RF.set_brake_mode(MOTOR_BRAKE_BRAKE);
    RM.move(0);
    RM.set_brake_mode(MOTOR_BRAKE_BRAKE);
    RB.move(0);
    RB.set_brake_mode(MOTOR_BRAKE_BRAKE);
  }
  else {
    LF.move(0);
    LF.set_brake_mode(MOTOR_BRAKE_COAST);
    LM.move(0);
    LM.set_brake_mode(MOTOR_BRAKE_COAST);
    LB.move(0);
    LB.set_brake_mode(MOTOR_BRAKE_COAST);
    RF.move(0);
    RF.set_brake_mode(MOTOR_BRAKE_COAST);
    RM.move(0);
    RM.set_brake_mode(MOTOR_BRAKE_COAST);
    RB.move(0);
    RB.set_brake_mode(MOTOR_BRAKE_COAST);
  }
  
}

void LeftMotorstop() {
  LF.move(0);
  LF.set_brake_mode(MOTOR_BRAKE_BRAKE);
  LM.move(0);
  LM.set_brake_mode(MOTOR_BRAKE_BRAKE);
  LB.move(0);
  LB.set_brake_mode(MOTOR_BRAKE_BRAKE);
}

void RightMotorstop() {
  RF.move(0);
  RF.set_brake_mode(MOTOR_BRAKE_BRAKE);
  RM.move(0);
  RM.set_brake_mode(MOTOR_BRAKE_BRAKE);
  RB.move(0);
  RB.set_brake_mode(MOTOR_BRAKE_BRAKE);
}

void BaseMotorReset() { 
  LF.tare_position(); //or set_zero_position(0) or set_zero_position(LF.get_position()); (sets current encoder position to 0)
  LB.tare_position();
	RF.tare_position();
	RB.tare_position();
  RM.tare_position();
	LM.tare_position();
  BaseMotorStop();
}

void IntakeStop(int mode = 0) {
  if (mode == 2) {
    Mintake.move(0);
    Mintake.set_brake_mode(MOTOR_BRAKE_HOLD);
     FTintake.move(0);
    FTintake.set_brake_mode(MOTOR_BRAKE_HOLD);
    FMintake.move(0);
    FMintake.set_brake_mode(MOTOR_BRAKE_HOLD);
  }
  else if (mode == 1) {
    Mintake.move(0);
    Mintake.set_brake_mode(MOTOR_BRAKE_BRAKE);
     FTintake.move(0);
    FTintake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    FMintake.move(0);
    FMintake.set_brake_mode(MOTOR_BRAKE_BRAKE);
  }
  else {
    Mintake.move(0);
    Mintake.set_brake_mode(MOTOR_BRAKE_COAST);
     FTintake.move(0);
    FTintake.set_brake_mode(MOTOR_BRAKE_COAST);
    FMintake.move(0);
    FMintake.set_brake_mode(MOTOR_BRAKE_COAST);
  }
}

void IntakeReset() {
  Mintake.tare_position();
  Mintake.move(0);
  FTintake.tare_position();
  FTintake.move(0);
  FMintake.tare_position();
  FMintake.move(0);
}

void Left(int speedpct){
  int speed = speedpct * 127 / 100;
  LF.move(speed);
  LM.move(speed);
  LB.move(speed);
}

void Right(int speedpct){
  int speed = speedpct * 127 / 100;
  RF.move(speed);
  RM.move(speed);
  RB.move(speed);
}

//左侧马达驱动
//left motor movement by voltage
void LeftVol(int vol_input) {
   int voltage = 128 * vol_input;

  LF.move_voltage(voltage);
  LM.move_voltage(voltage);
  LB.move_voltage(voltage);
}

void RightVol(int vol_input) {
   int voltage = 128 * vol_input;

  RF.move_voltage(voltage);
  RM.move_voltage(voltage);
  RB.move_voltage(voltage);
}

void Roller(int vol_input) {
  int voltage = 128 * vol_input;

  Mintake.move_voltage(voltage);
  FTintake.move_voltage(voltage);
  FMintake.move_voltage(voltage);
}

//straight by voltage
void RunVol(int basepct) {
  LeftVol(basepct);
  RightVol(basepct);
}

//turns
void TurnVol(int turnpct) {
  LeftVol(turnpct);
  RightVol(-turnpct);
}

//timebased straights
void RunVolAuto(int basepct, int outtime) {
  RunVol(basepct);
  delay(outtime);
  BaseMotorStop();
}

//timebased turns
void TurnVolAuto(int basepct, int outtime) {
  TurnVol(basepct);
  delay(outtime);
  BaseMotorStop();
}

// 角度误差计算函数
//calculating the angledifference
double calculateAngleError(double targetAngle , double currentAngle) {

  // 计算两个角度之间的最短距离
  //calculating the shortest distance of the two angles
  double angleDiff = targetAngle - currentAngle;
  
  // 根据周期性调整角度差，选择最短路径
  //picking the shortest route
    if (fabs(angleDiff) > 180.0) {
      if (angleDiff > 0) {
          angleDiff -= 360.0;
      } else {
          angleDiff += 360.0;
      }
  }

  return angleDiff;
}
//global turns
//turn pid til angle

//转角度
//application: TurnVolpidNto(how fast(0-100),how far(0-2000), how much error accepted(usually 1),timeout(100-1000 milliseconds),pid case (reference bottom))
void TurnVolpidNTo(int max_speed, double aim, double howerr, int outtime,int p_point) {
 
  //PID values
  double Kp = 3.8;
  double Ki = 0.006;
  double Kd = 0.7;

  //pid value cases
  switch (p_point) 
  {
    case 0: break;
    case 1:Kp = 1.65;Ki = 0;Kd =8;break; //TurnVolpidNTo(70,90,1,1500,1);(90-115)
    case 2:Kp = 1.9;Ki = 0;Kd =7;break; //TurnVolpidNTo(70,44,1,1500,2);(45)
    case 3:Kp = 1.6;Ki = 0;Kd =7;break;//TurnVolpidNTo(70,65,2,1500,3);(65)
    case 4:Kp = 2.2;Ki = 0;Kd =9;break;//TurnVolpidNTo(70,25,1,1500,4);(25)
    case 5:Kp = 1.3;Ki = 0;Kd =8;break; //TurnVolpidNTo(70,80,1,1500,5);(75)
    case 6:Kp = 1.58;Ki = 0;Kd =6.5;break; //TurnVolpidNTo(70,85,1,1500,6);(85)
    case 7:Kp = 1.29;Ki = 0;Kd =7;break; //TurnVolpidNTo(80,74,2,1500,7);(70)
    case 8:Kp = 2.1;Ki = 0;Kd =8;break;//TurnVolpidNTo(70,35,1,1500,8);(35)
    case 9:Kp = 1.53;Ki = 0;Kd =8;break; //TurnVolpidNTo(70,90,1,1500,9);(160)
    case 10:Kp = 1.53;Ki = 0;Kd =8;break; //TurnVolpidNTo(70,90,1,1500,10);(180+)
    case 11:Kp = 1.59;Ki = 0;Kd =7;break;//TurnVolpidNTo(70,65,2,1500,11);(65)
    case 12:Kp = 4;Ki = 0;Kd =8;break;//TurnVolpidNTo(70,15,1,1500,12);
    case 13:Kp = 4.5;Ki = 0;Kd =8;break;//TurnVolpidNTo(70,8,1,1500,13);
    case 14:Kp = 2.1;Ki = 0;Kd =7;break;//TurnVolpidNTo(70,35,1,1500,14);(35)
    case 15:Kp = 2.5;Ki = 0;Kd =8;break;//TurnVolpidNTo(70,20,1,1500,15);(25)
    case 16:Kp = 1.48;Ki = 0;Kd =7.5;break; //TurnVolpidNTo(70,75,1,1500,16);(75)
    case 17:Kp = 1.9;Ki = 0;Kd =7;break; //TurnVolpidNTo(70,75,1,1500,17);(75)
    case 18:Kp = 2.3;Ki = 0;Kd =7;break;//TurnVolpidNTo(70,35,1,1500,18);(35)
    case 19:Kp = 3.2;Ki = 0;Kd =21.5;break;//TurnVolpidNTo(70,15,1,1500,12);
    case 20:Kp =5.5;Ki = 0;Kd =8;break;//TurnVolpidNTo(70,8,1,1500,20);
    ///////////////////////342 4.125 6M////////////////////////////
    case 21:Kp = 1.58;Ki = 0;Kd =8;break; //TurnVolpidNTo(70,90,1,1500,21);(90-115)
    case 22:Kp = 1.68;Ki = 0;Kd =8;break; //TurnVolpidNTo(70,170,1,1500,22);(180+)
    case 23:Kp = 1.6;Ki = 0;Kd =9;break; //TurnVolpidNTo(70,90,1,1500,21);(90-115)
    case 24:Kp = 1.68;Ki = 0;Kd =8;break; //TurnVolpidNTo(70,170,1,1500,22);(180+)
    case 25:Kp = 1.48;Ki = 0;Kd =7.5;break; //TurnVolpidNTo(70,75,1,1500,25);(75)
    case 26:Kp = 1.4;Ki = 0;Kd =10;break; //TurnVolpidNTo(70,75,1,1500,25);(75)
    case 27:Kp = 1.42;Ki = 0;Kd =11;break; //TurnVolpidNTo(70,75,1,1500,25);(75)
    case 28:Kp = 2.5;Ki = 0.005;Kd =21;break; //TurnVolpidNTo(100,120,0.3,700,28);(120)
    case 29:Kp = 2.5;Ki = 0.005;Kd =21;break; //TurnVolpidNTo(100,100,0.3,700,29);(100)
    case 30:Kp = 2.5;Ki = 0.005;Kd =22;break; //TurnVolpidNTo(100,80,0.3,700,30);(80)
    case 31:Kp = 3.8;Ki = 0.005;Kd =0.7;break; //TurnVolpidNTo(100,60,0.3,700,31);(60)
    case 32:Kp = 3.4;Ki = 0.005;Kd =0.8;break; //TurnVolpidNTo(100,40,0.3,700,32);(40)
    case 33:Kp = 2.2;Ki = 0;Kd =21;break; //TurnVolpidNTo(100,20,0.3,700,33);(20)
    case 34:Kp = 2.5;Ki = 0;Kd =21;break; //TurnVolpidNTo(100,20,0.3,700,33);(20)

    default:Kp = 2.47;Ki = 0.005;Kd =21;
  }
//90 degrees is 30
//other values
  double err_now = 0;
  double err_last = 0;
  double value_now = 0;
  double EI = 0, ED = 0;
  double output;
  int sampletime = 10;

  T1.reset();
  T4.reset();

  //画个长方形
  //draw rectangle
  pros::lcd::initialize(); // initialize LCD once in main
   pros::lcd::set_background_color(255, 0, 0); // set background color
  pros::screen::fill_rect(0, 0, 400, 400);     // draw filled rectangle
  
  
  while (1) {
    double value_now = imu.get_rotation();
    err_now = calculateAngleError(aim, value_now);

    EI += err_now;
    if (fabs(err_now) > 10) EI = 0;
    ED = err_now - err_last;

    output = Kp * err_now + Ki * EI + Kd * ED;
    if (fabs(output) > max_speed)
      output = sgn(output) * max_speed;

    TurnVol(output);
    err_last = err_now;

    pros::delay(sampletime);

    if (fabs(err_now) > howerr) T1.reset();
    if (T1.elapsed() > 25 || T4.elapsed() >= outtime) {
      BaseMotorStop(1);
      break;
    }
  }

  pros::lcd::initialize();                 // initialize the LCD (do once in main)
  pros::lcd::set_background_color(0, 0, 255); // set background color to blue (RGB)
  pros::screen::fill_rect(0, 0, 400, 400);    // draw filled rectangle

         // draw filled rectangle
  LF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  RF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  LF.move(0);   // stop the motor
  RF.move(0);

  LM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  RM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  LM.move(0);
  RM.move(0);

  LB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  RB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  LB.move_velocity(0);
  RB.move_velocity(0);

  con.print(1, 0, "Err: %.2f ", err_now);
}

//直走转角度
//straigh then turn
//application: RunpidStraightNto(how fast(0-100), how far(-1500~1500),)
void RunpidStraightNTo(double speed_limit, int aim, double err_1,
                       double speed_limit2, int dec_point, int change_steps,
                       int start_point, int outtime, double newgyro, int p_point) 
{
 // Inertial1.resetRotation();
  double Kp = 0.22;  // 0.39
  double Ki = 0; // 0.001
  double Kd = 0.245; // 0.173
 switch (p_point) 
{
    case 0: break;
    ////////////////////////////////400 3.25 6M////////////////////////////////////////
    case 1:Kp = 0.15;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(100,2970,20,10,400,0,0,1800,0,1);(3050)
   case 2:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
    case 3:Kp = 0.148;Ki = 0;Kd =0.11;break; //RunpidStraightNTo(65,1200,12,45,1000,0,0,1800,0,3);
    case 4:Kp = 0.09;Ki = 0;Kd =0.15;break;//RunpidStraightNTo(20,200,6,5,20,0,0,1800,0,4);
    case 5:Kp = 0.19;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,5);
    case 6:Kp = 0.18;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,6);
    case 7:Kp = 0.14;Ki = 0;Kd =0.048;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,7);
    case 8:Kp = 0.121;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,8);
     case 9:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(60,900,10,15,150,0,0,1800,0,9);
     case 10:Kp = 0.16;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,10);
      case 11:Kp = 0.15;Ki = 0;Kd =0.15;break;//RunpidStraightNTo(30,300,10,0,0,0,0,1800,0,11);
      case 12:Kp = 0.11;Ki = 0;Kd =0.12;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,12);
      //////////////////////////////////257 4.125 4M/////////////////////////////////////////
       case 13:Kp = 0.16;Ki = 0;Kd =0.17;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,13);
       case 14:Kp = 0.141;Ki = 0;Kd =0.07;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,14);
        case 15:Kp = 0.1198;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,15);
        case 16:Kp = 0.11;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(50,800,10,10,120,0,0,1800,0,16);
         case 17:Kp = 0.18;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,17);
          case 18:Kp = 0.15;Ki = 0;Kd =0.05;break; //RunpidStraightNTo(75,1250,12,25,200,0,0,1800,0,18);
          case 19:Kp = 0.23;Ki = 0;Kd =0.28;break; //RunpidStraightNTo(95,2600,20,10,100,0,0,1800,0,19);
          case 20:Kp = 0.13;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
          case 21:Kp = 0.1355;Ki = 0;Kd =0.01;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,21);
          case 22:Kp = 0.1357;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,22);
          case 23:Kp = 0.13;Ki = 0;Kd =0.065;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,23);
          case 24:Kp = 0.111;Ki = 0;Kd =0.1;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,24);
           case 25:Kp = 0.13;Ki = 0;Kd =0.06;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,25);
            case 26:Kp = 0.09;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,26);
            case 27:Kp = 0.141;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,27);
             case 28:Kp = 0.109;Ki = 0;Kd =0.001;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,28);
                case 29:Kp = 0.13;Ki = 0;Kd =0.04;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,29);
                    case 30:Kp = 0.121;Ki = 0;Kd =0.03;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,30);
                     case 31:Kp = 0.137;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
                    ///////////////////////////////////////////342 4.125 6M//////////////////////////////////////////////////
                     case 32:Kp = 0.113;Ki = 0;Kd =0;break; //RunpidStraightNTo(100,2530,5,20,1800,-1,0,18000,21,32);
                      case 33:Kp = 0.09;Ki = 0;Kd =0.001;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,33);
                      case 34:Kp = 0.113;Ki = 0;Kd =0.001;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,34);
                       case 35:Kp = 0.11;Ki = 0;Kd =0.09;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
                       case 36:Kp = 0.17;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                        case 37:Kp = 0.130;Ki = 0;Kd =0.28;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,37);
                        case 38:Kp = 0.132;Ki = 0;Kd =0.01;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,38);
                          case 39:Kp = 0.23;Ki = 0;Kd =0.21;break;//RunpidStraightNTo(600,1000,20,100,400,0,0,4000,0,39);
    default:Kp = 0.1;Ki = 0;Kd =0;
    //case 39 0.345, 0.01, 0.05
 }
  double value_now = imu.get_rotation();
  double EI = 0, ED = 0;
  int sampletime = 10;
  double err_now = err_1 + 1;
  double err_last = 0;
  double max_v = speed_limit;
  double Kt = 0;
  double Ktv = 0;
  double value_now_L = 0;
  double value_last_L = 0;
  double value_now_R = 0;
  double value_last_R = 0;
  double outputL, outputR;
  double ET = 0;
  double ETV = 0;
  double sum_dec = 0;
  double K_gyro = 0.7;
  double angle_err = 0;
  double acc = 0.2;

  LF.tare_position();
  RF.tare_position();
  T1.reset();
  T2.reset();
  T3.reset();
  T4.reset();
  TACC.reset();

  while (1) {
    double returnangle = return_angle;

    max_v = acc * TACC.elapsed();
    if (TACC.elapsed() > 500) max_v = speed_limit;
    if (max_v >= speed_limit) max_v = speed_limit;

    if (dec_point != -1) {
      if (fabs(LF.get_position()) > dec_point) {
        max_v = speed_limit - (fabs(LF.get_position()) - dec_point) / 10.0;
        if (max_v < speed_limit2) max_v = speed_limit2;
      }
    }

    if (change_steps != -1 && fabs(LF.get_position()) >= start_point) {
      steps = change_steps;
      change_steps = -1;
    }

    value_now = LF.get_position();

    if (T2.elapsed() > 100) {
      T2.reset();
      value_last_R = value_now_R;
      value_last_L = value_now_L;
      value_now_R = RF.get_position();
      value_now_L = LF.get_position();
      ETV = (value_now_R - value_last_R) - (value_now_L - value_last_L);
      ET = value_now_R - value_now_L;
      sum_dec += Ktv * ETV;
    }

    if (T3.elapsed() > sampletime) {
      T3.reset();
      EI += err_now;
      err_last = err_now;
      err_now = aim - value_now;
      ED = err_now - err_last;
    }

    if (fabs(err_now) > 100) EI = 0;

    outputL = Kp * err_now + Ki * EI + Kd * ED;
    if (fabs(outputL) > max_v) outputL = sgn(outputL) * max_v;

  angle_err = newgyro - returnangle;
  if (angle_err > 180) angle_err -= 360;
  if (angle_err < -180) angle_err += 360;

    outputR = outputL - Kt * ET - Ktv * ETV - K_gyro * angle_err;
    if (outputL == 0) outputR = 0;

    LF.move(outputL);
    RF.move(outputR);
    LM.move(outputL);
    RM.move(outputR);
    LB.move(outputL);
    RB.move(outputR);

    pros::delay(sampletime);

    if (fabs(err_now) < err_1 || T4.elapsed() >= outtime) {
      BaseMotorStop(0);
      con.print(1, 0, "Err: %.2f OutL: %.2f", err_now, outputL);
      con.print(2, 0, "OutR: %.2f Gyro: %.2f", outputR, returnangle);
      break;
    }
  }
  BaseMotorStop(1);
}
////////////////////////////////////////////Time Based Code EWWW////////////////
//直走到指定角度
// void RunpidStraightN(double speed_limit, int aim,double err_1,double speed_limit2, int dec_point, int change_steps,int start_point,int outtime, double newgyro, int p_point)
// {

//   //PID参数
//   double Kp = 0.22;  // 0.39
//   double Ki = 0; // 0.001
//   double Kd = 0.245; // 0.173

//   //选择PID参数
//  switch (p_point) 
// {
//   case 0: break;
//   ////////////////////////////////400 3.25 6M////////////////////////////////////////
//   case 1:Kp = 0.15;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(100,2970,20,10,400,0,0,1800,0,1);(3050)
//   case 2:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
//   case 3:Kp = 0.148;Ki = 0;Kd =0.11;break; //RunpidStraightNTo(65,1200,12,45,1000,0,0,1800,0,3);
//   case 4:Kp = 0.09;Ki = 0;Kd =0.15;break;//RunpidStraightNTo(20,200,6,5,20,0,0,1800,0,4);
//   case 5:Kp = 0.19;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,5);
//   case 6:Kp = 0.18;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,6);
//   case 7:Kp = 0.14;Ki = 0;Kd =0.048;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,7);
//   case 8:Kp = 0.121;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,8);
//   case 9:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(60,900,10,15,150,0,0,1800,0,9);
//   case 10:Kp = 0.16;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,10);
//   case 11:Kp = 0.11;Ki = 0;Kd =0.18;break;//RunpidStraightNTo(30,300,10,0,0,0,0,1800,0,11);
//   case 12:Kp = 0.11;Ki = 0;Kd =0.12;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,12);
//   //////////////////////////////////257 4.125 4M/////////////////////////////////////////
//   case 13:Kp = 0.16;Ki = 0;Kd =0.17;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,13);
//   case 14:Kp = 0.141;Ki = 0;Kd =0.07;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,14);
//   case 15:Kp = 0.1198;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,15);
//   case 16:Kp = 0.11;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(50,800,10,10,120,0,0,1800,0,16);
//   case 17:Kp = 0.18;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,17);
//   case 18:Kp = 0.15;Ki = 0;Kd =0.05;break; //RunpidStraightNTo(75,1250,12,25,200,0,0,1800,0,18);
//   case 19:Kp = 0.23;Ki = 0;Kd =0.28;break; //RunpidStraightNTo(95,2600,20,10,100,0,0,1800,0,19);
//   case 20:Kp = 0.13;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
//   case 21:Kp = 0.1355;Ki = 0;Kd =0.01;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,21);
//   case 22:Kp = 0.1357;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,22);
//   case 23:Kp = 0.13;Ki = 0;Kd =0.065;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,23);
//   case 24:Kp = 0.111;Ki = 0;Kd =0.1;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,24);
//   case 25:Kp = 0.13;Ki = 0;Kd =0.06;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,25);
//   case 26:Kp = 0.09;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,26);
//   case 27:Kp = 0.141;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,27);
//   case 28:Kp = 0.109;Ki = 0;Kd =0.001;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,28);
//   case 29:Kp = 0.13;Ki = 0;Kd =0.04;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,29);
//   case 30:Kp = 0.121;Ki = 0;Kd =0.03;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,30);
//   case 31:Kp = 0.137;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
//   ///////////////////////////////////////////342 4.125 6M//////////////////////////////////////////////////
//   case 32:Kp = 0.13;Ki = 0;Kd =0.03;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
//   case 33:Kp = 0.08;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,33);
//   case 34:Kp = 0.17;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,34);
//   case 35:Kp = 0.11;Ki = 0;Kd =0.09;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
//   case 36:Kp = 0.17;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
//   case 37:Kp = 0.115;Ki = 0;Kd =0.09;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,37);
//   case 38:Kp = 0.132;Ki = 0;Kd =0.01;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,38);
//   case 39:Kp = 0.05;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);

//   default:Kp = 0.1;Ki = 0;Kd =0;
//   }

//  //其他参数
//   double value_now = 0;
//   double EI = 0, ED = 0;
//   int sampletime = 10;
//   double err_now = err_1+1;
//   double err_last = 0;
//   double max_v = speed_limit;
//   double Kt = 0;
//   double Ktv = 0;
//   double value_now_L = 0;
//   double value_last_L = 0;
//   double value_now_R = 0;
//   double value_last_R = 0;
//   double outputL, outputR;
//   double ET = 0;
//   double ETV = 0;
//   double sum_dec = 0;
//   double K_gyro = 0.7;
//   double angle_err = 0;
//   double acc = 0.2;

//   //计时器和马达复位
//   LF.tare_position();
//   RF.tare_position();
//   T1.reset();
//   T2.reset();
//   T3.reset();
//   T4.reset();
//   TACC.reset();
//   while (1) {

//     //调试输出
//     double vel_rpm = LF.get_actual_velocity();
//     double vel_pct = vel_rpm / 200.0 * 100.0;  // convert to percentage
//     printf("%.2f%%\n", vel_pct);
//     max_v = acc * TACC.elapsed();

//     if (TACC.elapsed() > 500)
//       max_v = speed_limit;

//     //控制输出最大值
//     if (max_v >= speed_limit) {
//       max_v = speed_limit;
//     }

//     //减速点输出控制
//     if (dec_point != -1) {
//       if (fabs(LF.get_position()) > dec_point) {
//         max_v =
//             speed_limit -
//             (fabs(LF.get_position()) - dec_point) /
//                 50.0;
//         if (max_v < speed_limit2) {
//           max_v = speed_limit2;
//         }
//       }
//     }

//     //线程控制
//     if (change_steps != -1) {
//       if (start_point <= fabs(value_now_L)) {
//         steps = change_steps;
//         change_steps = -1;
//       }
//     }

//     //读取马达数值
//     value_now = LF.get_position();
//     if (T2.elapsed() > 100) {
//       T2.reset();
//       value_last_R = value_now_R;
//       value_last_L = value_now_L;
//       value_now_R = RF.get_position();
//       value_now_L = LF.get_position();
//       ETV = (value_now_R - value_last_R) - (value_now_L - value_last_L);
//       ET = value_now_R - value_now_L;
//       sum_dec += Ktv * ETV;
//     }

//     //计算PID输出
//     if (T3.elapsed() > sampletime) {
//       T3.reset();
//       EI = EI + err_now;
//       err_last = err_now;
//       err_now = aim - value_now;
//       ED = err_now - err_last;
//     }
    
//     if (fabs(err_now) > 100)
//       EI = 0;
//       outputL = Kp * err_now + Ki * EI + Kd * ED;
    
//     //限制最大输出
//     if (fabs(outputL) > max_v)
//       outputL = sgn(outputL) * max_v;
//       angle_err = newgyro ;
    
//     if (fabs(angle_err) < 1)   angle_err = 0;
//     outputR = outputL - Kt * ET - Ktv * ETV - K_gyro * (angle_err);
    
//     //限制输出
//     if (outputL == 0){
//       outputR = 0;
//     }

//     if(outputL>max_v){
//       outputL=max_v;
//     }

//     if(outputR>max_v){
//       outputR=max_v;
//     }

//     //输出
//    // Spin left motors
//   LF.move_velocity(outputL);
//   LM.move_velocity(outputL);
//   LB.move_velocity(outputL);

// // Spin right motors
//   RF.move_velocity(outputR);
//   RM.move_velocity(outputR);
//   RB.move_velocity(outputR);
    
//     pros::delay(sampletime);
  
//       //判断停止条件
//       if (fabs(err_now) < err_1|| T4.elapsed() >= outtime){
//       // Stop left motors with coast
//       LF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//       LF.move_velocity(0);

//       LM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//       LM.move_velocity(0);

//       LB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//       LB.move_velocity(0);

//       // Stop right motors with coast
//       RF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//       RF.move_velocity(0);

//       RM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//       RM.move_velocity(0);

//       RB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//       RB.move_velocity(0);
//         break;
//     }
    
//   }

//   //底盘马达全停
//   BaseMotorStop(1);
// }

//直走到指定角度漂移
//arcturns
void RunpidTo(double speed_limit, int aim,double err_1,double speed_limit2, int dec_point, int change_steps,int start_point,int outtime, double newgyro, int p_point,int turn_point, int turn_side,int turn_right)
{
 // Inertial1.resetRotation();
  double Kp = 0.22;  // 0.39
  double Ki = 0; // 0.001
  double Kd = 0.245; // 0.173
  double K_gyro = 0.4;
  double Kd_gyro = 0;
  
 switch (p_point) 
{
    case 0: break;
    ////////////////////////////////400 3.25 6M////////////////////////////////////////
    case 1:Kp = 0.15;Ki = 0;Kd =0.12;break; //RunpidStraightNTo(100,2970,20,10,400,0,0,1800,0,1);(3050)
    case 2:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
    case 3:Kp = 0.148;Ki = 0;Kd =0.11;break; //RunpidStraightNTo(65,1200,12,45,1000,0,0,1800,0,3);
    case 4:Kp = 0.001;Ki = 0;Kd =0;break;//RunpidStraightNTo(20,200,6,5,20,0,0,1800,0,4);
    case 5:Kp = 0.19;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,5);
    case 6:Kp = 0.18;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,6);
    case 7:Kp = 0.14;Ki = 0;Kd =0.048;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,7);
    case 8:Kp = 0.121;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,8);
    case 9:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(60,900,10,15,150,0,0,1800,0,9);
    case 10:Kp = 0.16;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,10);
    case 11:Kp = 0.15;Ki = 0;Kd =0.15;break;//RunpidStraightNTo(30,300,10,0,0,0,0,1800,0,11);
    case 12:Kp = 0.11;Ki = 0;Kd =0.12;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,12);
      //////////////////////////////////257 4.125 4M/////////////////////////////////////////
    case 13:Kp = 0.16;Ki = 0;Kd =0.17;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,13);
    case 14:Kp = 0.141;Ki = 0;Kd =0.07;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,14);
    case 15:Kp = 0.1198;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,15);
    case 16:Kp = 0.11;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(50,800,10,10,120,0,0,1800,0,16);
    case 17:Kp = 0.18;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,17);
    case 18:Kp = 0.15;Ki = 0;Kd =0.05;break; //RunpidStraightNTo(75,1250,12,25,200,0,0,1800,0,18);
    case 19:Kp = 0.04;Ki = 0;Kd =0.4;break; //RunpidStraightNTo(95,2600,20,10,100,0,0,1800,0,19);
    case 20:Kp = 0.13;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
    case 21:Kp = 0.1355;Ki = 0;Kd =0.01;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,21);
    case 22:Kp = 0.1357;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,22);
    case 23:Kp = 0.13;Ki = 0;Kd =0.065;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,23);
    case 24:Kp = 0.111;Ki = 0;Kd =0.1;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,24);
           case 25:Kp = 0.13;Ki = 0;Kd =0.06;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,25);
            case 26:Kp = 0.09;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,26);
            case 27:Kp = 0.141;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,27);
             case 28:Kp = 0.109;Ki = 0;Kd =0.001;K_gyro = 0.3;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,28);
                case 29:Kp = 0.109;Ki = 0;Kd =0.001;K_gyro = 0.5;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,29);
                    case 30:Kp = 0.121;Ki = 0;Kd =0.03;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,30);
                     case 31:Kp = 0.137;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
                    ///////////////////////////////////////////342 4.125 6M//////////////////////////////////////////////////
                     case 32:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 0.25;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                      case 33:Kp = 0.08;Ki = 0;Kd =0.3;K_gyro = 0.3;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,33);
                      case 34:Kp = 0.17;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,34);
                       case 35:Kp = 0.11;Ki = 0;Kd =0.09;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
                       case 36:Kp = 0.13;Ki = 0;Kd =0.8;K_gyro = 1.3;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
              case 37:Kp = 0.09;Ki = 0;Kd =0.5;break;//RunpidStraightNTo(30,300,10,0,0,0,0,1800,0,11);
              case 38:Kp = 0.12;Ki = 0;Kd =0.14;K_gyro = 0.55;break; //  RunpidTo(100,2000,10,80,1000,0,0,1000,0,38);
              case 39:Kp = 0.16;Ki = 0;Kd =0.23;K_gyro = 0.6;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
              case 40:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 0.65;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,40);
               case 41:Kp = 0.12;Ki = 0;Kd =0.3;K_gyro = 0.9;break; //  RunpidTo(100,2000,10,80,1000,0,0,1000,0,38);
               case 42:Kp = 0.09;Ki = 0;Kd =0.5;K_gyro = 1.5;break;//RunpidStraightNTo(30,300,10,0,0,0,0,1800,0,11);
               case 43:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1.8;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
               case 44:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 0.7;break; // RunpidTo(80,2600,10,65,1000,6,400,1200,180,44,600,2);
               case 45:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1.4;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
               case 46:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1.1;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
               case 47:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 1.7;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,40);
                case 48:Kp = 0.15;Ki = 0;Kd =0.15;K_gyro = 1.7;break;break;//RunpidStraightNTo(30,300,10,0,0,0,0,1800,0,11);
                case 49:Kp = 0.13;Ki = 0;Kd =0.13;K_gyro = 0.35;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                 case 50:Kp = 0.13;Ki = 0;Kd =0.13;K_gyro = 0.5;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                  case 51:Kp = 0.09;Ki = 0;Kd =0.16;K_gyro = 0.72;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,51);
                   case 52:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 0.3;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                   case 53:Kp = 0.145;Ki = 0;Kd =0.5;K_gyro = 0.65;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                    case 54:Kp = 0.08;Ki = 0;Kd =0.15;K_gyro = 0.75;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                    case 55:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1.2;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                    case 56:Kp = 0.12;Ki = 0;Kd =0.3;K_gyro = 0.3;break; //  RunpidTo(100,2000,10,80,1000,0,0,1000,0,38);
                      case 57:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 0.8;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                        case 58:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 1.1;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                          case 59:Kp = 0.13;Ki = 0;Kd =0.1;K_gyro = 0.5;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                   case 60:Kp = 0.12;Ki = 0;Kd =0.3;K_gyro = 2;break; //  RunpidTo(100,2000,10,80,1000,0,0,1000,0,38);
                    case 61:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 0.93;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                    case 62:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 0.3;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                    case 63:Kp = 0.12;Ki = 0;Kd =0.3;K_gyro = 0.9;break; //  RunpidTo(100,2000,10,80,1000,0,0,1000,0,38);
                    
                     case 64:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 0.65;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);case 63:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                     case 65:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 0.5;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);case 63:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1;break;
                     case 66:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1.55;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36); //1.55
                     
                      case 67:Kp = 0.08;Ki = 0;Kd =0.14;K_gyro = 0.55;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,32);
                      case 68:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 0.8;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                      case 69:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 0.5;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                      case 70:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 1;Kd_gyro=0;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36); 
                                    case 71:Kp = 0.16;Ki = 0;Kd =0.4;K_gyro = 3.5;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36); 
    default:Kp = 0.1;Ki = 0;Kd =0;K_gyro = 0.01;Kd_gyro=0;
 }
  double value_now = 0;
  double EI = 0, ED = 0;
  int sampletime = 10;
  double err_now = err_1 +1;
  double err_last = 0;
  double prev_angle_error=0;
  double angle_ED=0;
  double angle_EI=0;
  double max_v = speed_limit;
  double Kt = 0;
  double Ktv = 0;
  double value_now_L = 0;
  double value_last_L = 0;
  double value_now_R = 0;
  double value_last_R = 0;
  double outputL, outputR;
  double ET = 0;
  double ETV = 0;
  double sum_dec = 0;
  
  double angle_err = 0;
  double acc = 0.2;
    // double V = 3;
  LF.tare_position();
  RF.tare_position();
  T1.reset();
  T2.reset();
  T3.reset();
  T4.reset();
  TACC.reset();
   // double chushizhi = Inertial1.rotation(degrees);
  while (1) {
    double vel_rpm = LF.get_actual_velocity();
    double vel_pct = vel_rpm / 200.0 * 100.0;  // convert to percentage
    printf("%.2f%%\n", vel_pct);
    double returnangle =imu.get_rotation();
    max_v = acc * TACC.elapsed();
    if (TACC.elapsed() > 0)
      max_v = speed_limit;
    if (max_v >= speed_limit) {
      max_v = speed_limit;
    }
    if (dec_point != -1) {
      if (fabs(LF.get_position()) > dec_point) {
        max_v =speed_limit - (fabs(LF.get_position()) - dec_point) /10.0;
        if (max_v < speed_limit2) { max_v = speed_limit2;
        }
      }
    }
    if (change_steps != -1) {
      if (fabs(LF.get_position())>=start_point ) {
        steps = change_steps;
        change_steps = -1;
      }
    }
    value_now = LF.get_position();
    if (T2.elapsed() >= 0) {
      T2.reset();
      value_last_R = value_now_R;
      value_last_L = value_now_L;
      value_now_R = RF.get_position();
      value_now_L = LF.get_position();
      ETV = (value_now_R - value_last_R) - (value_now_L - value_last_L);
      ET = value_now_R - value_now_L;
      sum_dec += Ktv * ETV;
    }
    if (T3.elapsed() > sampletime) {
      T3.reset();
      EI = EI + err_now;
      err_last = err_now;
      err_now = aim - value_now;
      ED = err_now - err_last;
    }
    if (fabs(err_now) > 100)   EI = 0;
    outputL = Kp * err_now + Ki * EI + Kd * ED;
    outputR = Kp * err_now + Ki * EI + Kd * ED;
    if (fabs(outputL) > max_v)
      outputL = sgn(outputL) * max_v;
      if (fabs(outputR) > max_v)
      outputR = sgn(outputR) * max_v;
      if(fabs(LF.get_position())>turn_point && turn_side==2){
      
    
    angle_EI= angle_EI+angle_err;
    prev_angle_error=angle_err;
    angle_err = newgyro - returnangle;
    angle_ED = angle_err-prev_angle_error;
    
    if (fabs(angle_err) < 1)   angle_err = 0;
    // if(angle_err>0)
    // outputR = outputL - K_gyro * (angle_err);
     
    // else if(angle_err<0)
    outputL = outputL + Kt * ET + Kd_gyro*angle_ED * ETV + K_gyro * (angle_err);
    outputR = outputR - Kt * ET -Kd_gyro*angle_ED - K_gyro * (angle_err);
      }
if(fabs(LF.get_position())>turn_point&&turn_side==1){
      
    angle_err = newgyro - returnangle;
    if (fabs(angle_err) < 1)   angle_err = 0;
    // if(angle_err>0)
    // outputR = outputL - K_gyro * (angle_err);
     
    // else if(angle_err<0)
    // outputL = outputL + Kt * ET + Ktv * ETV + K_gyro * (angle_err);
    outputR = outputR - Kt * ET -Kd_gyro*angle_ED - K_gyro * (angle_err);
      }
      if(fabs(LF.get_position())>turn_point&&turn_side==0){
      
    angle_err = newgyro - returnangle;
    if (fabs(angle_err) < 1)   angle_err = 0;
    // if(angle_err>0)
    // outputR = outputL - K_gyro * (angle_err);
     
    // else if(angle_err<0)
    outputL = outputL + Kt * ET + Kd_gyro*angle_ED * ETV + K_gyro * (angle_err);
    // outputR = outputR - Kt * ET - Ktv * ETV - K_gyro * (angle_err);
      }
    if (outputL == 0){
      outputR = 0;
    }
    if(outputL>max_v){
      outputL=max_v;
    }
    if(outputR>max_v){
      outputR=max_v;
    }
      LeftVol(outputL);
      RightVol(outputR);
    // LeftMotor1.spin(vex::directionType::fwd,outputL , vex::velocityUnits::pct);
    // RightMotor1.spin(vex::directionType::fwd, outputR, vex::velocityUnits::pct);
    // LeftMotor2.spin(vex::directionType::fwd, outputL, vex::velocityUnits::pct);
    // RightMotor2.spin(vex::directionType::fwd, outputR, vex::velocityUnits::pct);
    // LeftMotor3.spin(vex::directionType::fwd, outputL, vex::velocityUnits::pct);
    // RightMotor3.spin(vex::directionType::fwd, outputR, vex::velocityUnits::pct);
    pros::delay(sampletime);
    if(turn_right==0){
      if ((fabs(err_now) < err_1)|| T4.elapsed() >= outtime){
      // T1.clear();
      LF.move_velocity(outputL);
        LM.move_velocity(outputL);
        LB.move_velocity(outputL);

        RF.move_velocity(outputR);
        RM.move_velocity(outputR);
        RB.move_velocity(outputR);
      break;
  }
    }
    if(turn_right==1){
      if (((fabs(err_now) < err_1)&&fabs(angle_err) < 3)|| T4.elapsed() >= outtime){
      // T1.clear();

     BaseMotorStop(0);
      break;
  }
    }
    
  }
 BaseMotorStop(1);
}

void RunpidStraightNToHC(double speed_limit, int aim, double err_1,
                       double speed_limit2, int dec_point, int change_steps,
                       int start_point, int outtime, double newgyro, int p_point) 
{
 // Inertial1.resetRotation();
  double Kp = 0.22;  // 0.39
  double Ki = 0; // 0.001
  double Kd = 0.245; // 0.173
 switch (p_point) 
{
    case 0: break;
    ////////////////////////////////400 3.25 6M////////////////////////////////////////
    case 1:Kp = 0.15;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(100,2970,20,10,400,0,0,1800,0,1);(3050)
   case 2:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
    case 3:Kp = 0.148;Ki = 0;Kd =0.11;break; //RunpidStraightNTo(65,1200,12,45,1000,0,0,1800,0,3);
    case 4:Kp = 0.09;Ki = 0;Kd =0.15;break;//RunpidStraightNTo(20,200,6,5,20,0,0,1800,0,4);
    case 5:Kp = 0.19;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,5);
    case 6:Kp = 0.18;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,6);
    case 7:Kp = 0.14;Ki = 0;Kd =0.048;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,7);
    case 8:Kp = 0.121;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,8);
     case 9:Kp = 0.16;Ki = 0;Kd =0.1;break;//RunpidStraightNTo(60,900,10,15,150,0,0,1800,0,9);
     case 10:Kp = 0.16;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,10);
      case 11:Kp = 0.15;Ki = 0;Kd =0.15;break;//RunpidStraightNTo(30,300,10,0,0,0,0,1800,0,11);
      case 12:Kp = 0.11;Ki = 0;Kd =0.12;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,12);
      //////////////////////////////////257 4.125 4M/////////////////////////////////////////
       case 13:Kp = 0.16;Ki = 0;Kd =0.17;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,13);
       case 14:Kp = 0.141;Ki = 0;Kd =0.07;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,14);
        case 15:Kp = 0.1198;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(95,2970,20,10,100,0,0,1800,0,15);
        case 16:Kp = 0.11;Ki = 0;Kd =0.17;break; //RunpidStraightNTo(50,800,10,10,120,0,0,1800,0,16);
         case 17:Kp = 0.18;Ki = 0;Kd =0.3;break; //RunpidStraightNTo(30,500,10,10,100,0,0,1800,0,17);
          case 18:Kp = 0.15;Ki = 0;Kd =0.05;break; //RunpidStraightNTo(75,1250,12,25,200,0,0,1800,0,18);
          case 19:Kp = 0.23;Ki = 0;Kd =0.28;break; //RunpidStraightNTo(95,2600,20,10,100,0,0,1800,0,19);
          case 20:Kp = 0.13;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
          case 21:Kp = 0.1355;Ki = 0;Kd =0.01;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,21);
          case 22:Kp = 0.1357;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,22);
          case 23:Kp = 0.13;Ki = 0;Kd =0.065;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,23);
          case 24:Kp = 0.111;Ki = 0;Kd =0.1;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,24);
           case 25:Kp = 0.13;Ki = 0;Kd =0.06;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,25);
            case 26:Kp = 0.09;Ki = 0;Kd =0.02;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,26);
            case 27:Kp = 0.141;Ki = 0;Kd =0.03;break; // RunpidStraightNTo(79,1700,15,78,1100,0,0,1500,0,27);
             case 28:Kp = 0.109;Ki = 0;Kd =0.001;break; //RunpidStraightNTo(100,2770,20,70,1970,0,0,4000,0,28);
                case 29:Kp = 0.13;Ki = 0;Kd =0.04;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,29);
                    case 30:Kp = 0.121;Ki = 0;Kd =0.03;break; //RunpidStraightNTo(79,1600,15,78,1100,0,0,1500,0,30);
                     case 31:Kp = 0.137;Ki = 0;Kd =0.15;break; //RunpidStraightNTo(100,3000,20,56,2000,0,0,5000,0,20);
                    ///////////////////////////////////////////342 4.125 6M//////////////////////////////////////////////////
                     case 32:Kp = 0.113;Ki = 0;Kd =0;break; //RunpidStraightNTo(100,2530,5,20,1800,-1,0,18000,21,32);
                      case 33:Kp = 0.09;Ki = 0;Kd =0.001;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,33);
                      case 34:Kp = 0.113;Ki = 0;Kd =0.001;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,34);
                       case 35:Kp = 0.11;Ki = 0;Kd =0.09;break;//RunpidStraightNTo(45,800,10,15,150,0,0,1800,0,2);
                       case 36:Kp = 0.17;Ki = 0;Kd =0.23;break; //RunpidStraightNTo(60,1000,20,6,60,0,0,1800,0,36);
                        case 37:Kp = 0.130;Ki = 0;Kd =0.28;break; //RunpidStraightNTo(60,800,10,10,120,0,0,1800,0,37);
                        case 38:Kp = 0.132;Ki = 0;Kd =0.01;break; //RunpidStraightNTo(80,1800,10,65,1000,0,0,1000,0,38);
                          case 39:Kp = 0.4;Ki = 0.2;Kd =0.3;break;//RunpidStraightNTo(20,200,6,5,20,0,0,1800,0,4);
                          case 40:Kp = 0.3;Ki = 0.2;Kd =0.005;break;//RunpidStraightNTo(20,200,6,5,20,0,0,1800,0,4);
    default:Kp = 0.1;Ki = 0;Kd =0;
 }
  double value_now = 0;
  double EI = 0, ED = 0;
  int sampletime = 10;
  double err_now = err_1+1;
  double err_last = 0;
  double max_v = speed_limit;
  double Kt = 0;
  double Ktv = 0;
  double value_now_L = 0;
  double value_last_L = 0;
  double value_now_R = 0;
  double value_last_R = 0;
  double outputL, outputR;
  double ET = 0;
  double ETV = 0;
  double sum_dec = 0;
  double K_gyro = 0.7;
  double angle_err = 0;
  double acc = 0.2;
  LF.tare_position();
  RF.tare_position();
  T1.reset();
  T2.reset();
  T3.reset();
  T4.reset();
  TACC.reset();
   // double chushizhi = Inertial1.rotation(degrees);
  while (1) {
    //  printf(".2f\n", LeftMotor1.velocity(pct));
    // double returnangle = Inertial1.rotation(degrees);
  double returnangle = return_angle;

   max_v = acc * TACC.elapsed();
    if (TACC.elapsed() > 500)
      max_v = speed_limit;
    if (max_v >= speed_limit) {
      max_v = speed_limit;
    }
    if (dec_point != -1) {
      if (fabs(LF.get_position()) > dec_point) {
        max_v =speed_limit - (fabs(LF.get_position()) - dec_point) /10.0;
        if (max_v < speed_limit2) { max_v = speed_limit2;
        }
      }
    }
    if (change_steps != -1) {
      if (fabs(LF.get_position())>=start_point ) {
        steps = change_steps;
        change_steps = -1;
      }
    }
    value_now = LF.get_position();;
    if (T2.elapsed() > 100) {
      T2.reset();
      value_last_R = value_now_R;
      value_last_L = value_now_L;
      value_now_R = RF.get_position();
      value_now_L = LF.get_position();
      ETV = (value_now_R - value_last_R) - (value_now_L - value_last_L);
      ET = value_now_R - value_now_L;
      sum_dec += Ktv * ETV;
    }
    if (T3.elapsed() > sampletime) {
      T3.reset();
      EI = EI + err_now;
      err_last = err_now;
      err_now = aim - value_now;
      ED = err_now - err_last;
    }
    if (fabs(err_now) > 100)
      EI = 0;
    outputL = Kp * err_now + Ki * EI + Kd * ED;
    // outputR = Kp * err_now + Ki * EI + Kd * ED;
    if (fabs(outputL) > max_v)
      outputL = sgn(outputL) * max_v;
      // if (fabs(outputR) > max_v)
      // outputR = sgn(outputR) * max_v;
    angle_err = newgyro - returnangle;
    if (fabs(angle_err) < 1)  { angle_err = 0;}
    // if(angle_err>0)
    outputR = outputL - Kt * ET - Ktv * ETV - K_gyro * (angle_err);
    // else if(angle_err<0)
    // outputL = outputR - Kt * ET - Ktv * ETV - K_gyro * (angle_err);
    if (outputL == 0)
      outputR = 0;
    LF.move_velocity(outputL);   // Left Front
    RF.move_velocity(outputR);   // Right Front
    LM.move_velocity(outputL);   // Left Middle
    RM.move_velocity(outputR);   // Right Middle
    LB.move_velocity(outputL);   // Left Back
    RB.move_velocity(outputR);   // Right Back
    LF.move(outputL);   // Left Front
    RF.move(outputR);   // Right Front
    LM.move(outputL);   // Left Middle
    RM.move(outputR);   // Right Middle
    LB.move(outputL);   // Left Back
    RB.move(outputR);   // Right Back
      // LeftMotor4.spin(vex::directionType::fwd, outputL, vex::velocityUnits::pct);
    // RightMotor4.spin(vex::directionType::fwd, outputR, vex::velocityUnits::pct);
    pros::delay(sampletime);
        if (fabs(err_now) < err_1|| T4.elapsed() >= outtime){
  
    BaseMotorStop(0);
      break;
        }
        con.print(1, 0, "Err: %.2f OutL: %.2f", err_now, outputL);
  con.print(2, 0, "OutR: %.2f Gyro: %.2f", outputR, returnangle);
      }
      BaseMotorStop(1);
    }


void drive_to_white_line(int speed,int outtime)
{
  OpticalC.set_led_pwm(100);

  //阈值
  double threshold = 15;

  T4.reset();
  while (1) {

    //运行
    RunVol(speed);

    //停止条件
    if (OpticalC.get_brightness() >= threshold || T4.elapsed() > outtime) {
        // Stop left motors with coast
        LF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LF.move_velocity(0);

        LM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LM.move_velocity(0);

        LB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LB.move_velocity(0);

        // Stop right motors with coast
        RF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        RF.move_velocity(0);

        RM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        RM.move_velocity(0);

        RB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        RB.move_velocity(0);

        break;
    } 
  }
  steps=0;
}

// void UpdateXY() {
//     // 本地坐标系变量
//     double local_Y_position = 0.0;
//     double local_X_position = 0.0;
//     double local_length;
//     double local_gyro;
    
//     // 计算本地坐标系坐标变化
//     double Straightmove = 2 * M_PI * WheelRadius * (Encoder1.position(vex::rotationUnits::deg) / 360);
//     double Sidemove = 2 * M_PI * WheelRadius * (Encoder2.position(vex::rotationUnits::deg) / 360);
    
//     // 获取并更新陀螺仪角度
//     double Gyro_rad = Inertial1.rotation(degrees) / (180 / M_PI);
//     double Gyroerror_rad = Gyro_rad - Gyrobefore_rad;
//     Gyrobefore_rad = Gyro_rad; // 更新之前的角度
    
//     // 使用圆弧拟合计算本地坐标
//     if (fabs(Gyroerror_rad) < 1e-2) { // 如果 Gyroerror_rad 接近于0，使用近似处理
//       local_Y_position = Straightmove;
//       local_X_position = Sidemove;
//   } else {
//       double factor = 2 / Gyroerror_rad;
//       local_Y_position = (factor * (Straightmove + Wheel_Y_distance * Gyroerror_rad)) * sin(Gyroerror_rad / 2);
//       local_X_position = (factor * (Sidemove + Wheel_X_distance * Gyroerror_rad)) * sin(Gyroerror_rad / 2);
//     }
    
//     // 如果本地坐标变化量为零，则不进行后续计算
//     if (local_X_position == 0 && local_Y_position == 0) {
//       return; // 直接返回，不更新位置
//     }
    
//     // 计算本地坐标系的角度和长度
//     local_gyro = atan2(local_Y_position, local_X_position);
//     local_length = sqrt(local_Y_position * local_Y_position + local_X_position * local_X_position);
    
//     // 更新全局坐标
//     double global_gyro = local_gyro - Gyrobefore_rad - (Gyroerror_rad / 2);
//     X_position += local_length * cos(global_gyro);
//     Y_position += local_length * sin(global_gyro);
    
//     // 重置编码器位置
//     Encoder1.resetPosition();
//     Encoder2.resetPosition();
// }

//角度规范化
float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

//限制最大输出
float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

//判断是否反转
bool is_reversed(double input){
  if(input<0) return(true);
  return(false);
}

//将输入功率转化为电压
float to_volt(float percent){
  return(percent*12.8/100.0);
}


// 角度规范化函数，确保角度在 -π 到 π 之间
double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// void RunXY_Straight(double speed_limit, double targetX, double targetY,double err_1, double outtime,int p_point) {

//     // 计算目标方向
//     double err_Y = targetY - Y_position;
//     double err_X = targetX - X_position;
//     double start_X = X_position;
//     double start_Y = Y_position;
//     double target_angle_rad = wrapAngle(atan2(err_X, err_Y));
//     double target_angle = target_angle_rad * (180 / M_PI);
   

//     // 计算直线行驶的距离
//     double covered_distance;

//     // PID 控制器参数
//     double Kp = 2.5;
//     double Ki = 0.001;
//     double Kd = 3;
//     double Kp2 = 4;
//     double Ki2 = 0.003;
//     double Kd2 = 5;
//     double err_now = 0;
//     double err_last = err_now;
//     const double sampletime = 10.0;
//     double outputL, outputR;
//     double K_gyro = 0.7;
//     double angle_err = 0.0;
//     switch (p_point) 
// {
//     case 0: break;
//     ////////////////////////////////400 3.25 6M////////////////////////////////////////
//     case 1:Kp = 0.15;Ki = 0;Kd =0.12;K_gyro = 0.01;break; //RunpidStraightNTo(100,2970,20,10,400,0,0,1800,0,1);(3050)
//     case 2:Kp = 2;Ki = 0.001;Kd =0.4;K_gyro = 0.3;break;
//     default:Kp = 2.5;Ki = 0.001;Kd =3;K_gyro = 0.7;
//  }

//     //  估算直线行驶的距离
//     double err_distance = sqrt(pow(err_X, 2) + pow(err_Y, 2));

//     //计算误差角度
//     angle_err = calculateAngleError(target_angle,return_angle);
   
//     // 计时器清零
//     T4.clear();

//     while (true) {

//       // 更新行走距离
//       covered_distance = sqrt(pow(X_position - start_X, 2) + pow(Y_position - start_Y, 2));

//       // 更新误差
      
//       err_now = sqrt(pow(err_X, 2) + pow(err_Y, 2));
//         // 陀螺仪误差校正
//       err_Y = targetY - Y_position;
//       err_X = targetX - X_position;

//       // 再次计算误差角度
//       target_angle_rad = atan2(err_X, err_Y);
//       target_angle = target_angle_rad * (180 / M_PI);
//       angle_err = calculateAngleError(target_angle,return_angle);

//       // 积分和微分项
//       double EI = 0.0, ED = 0.0;
      
//       // 微积分计算
//       EI = EI + err_now;
//       ED = err_now - err_last;

//       err_last = err_now;
      
//       // 陀螺仪误差校正
//       if (fabs(angle_err) < 1) angle_err = 0;

//       // 第二套PID参数
//       if(err_now < err_distance*0.4){
//         Kp = Kp2;
//         Ki = Ki2;
//         Kd = Kd2;
//       }

//       // // PID计算
//       outputL = Kp * err_now + Ki * EI + Kd * ED;
//       outputR = Kp * err_now + Ki * EI + Kd * ED;

//       //判断要前进还是后退
    

//       // 限制输出值
//       if (outputL > fabs(speed_limit)) outputL = speed_limit;
//       if (outputR > fabs(speed_limit)) outputR = speed_limit;

//       // 方向矫正
    
//       outputR = outputR - K_gyro * angle_err;
//       outputL = outputL + K_gyro * angle_err;
      
      
//       // 调试输出
//       printf("%.2f\n", angle_err);

//       // 控制电机
//       LeftMotor1.spin(vex::directionType::fwd, outputL*0.128, voltageUnits::volt);
//       RightMotor1.spin(vex::directionType::fwd, outputR*0.128, voltageUnits::volt);
//       LeftMotor2.spin(vex::directionType::fwd, outputL*0.128, voltageUnits::volt);
//       RightMotor2.spin(vex::directionType::fwd, outputR*0.128, voltageUnits::volt);
//       LeftMotor3.spin(vex::directionType::fwd, outputL*0.128, voltageUnits::volt);
//       RightMotor3.spin(vex::directionType::fwd, outputR*0.128, voltageUnits::volt);
//       // LeftVol(outputL);
//       // RightVol(outputR);
//       // 停止条件(误差小于阈值/超过保护时间/行走距离过长)
//       if (fabs(err_now) < err_1 || T4.time() > outtime || covered_distance > 1.2*err_distance) {
//           LeftMotor1.stop(vex::brakeType::coast);
//           RightMotor1.stop(vex::brakeType::coast);
//           LeftMotor2.stop(vex::brakeType::coast);
//           RightMotor2.stop(vex::brakeType::coast);
//           LeftMotor3.stop(vex::brakeType::coast);
//           RightMotor3.stop(vex::brakeType::coast);
//           break;
//       }

//       // 延迟
//       sleep(sampletime);
//     }
    
//     // 刹车
//     BaseMotorStop(1);
// }
/////////////////////////////////////////////////////////////////////////////////////////////////////
// 编码器转弯面向坐标
// void FaceToXY(double speed_limit, double targetX, double targetY,double err_1, double outtime) {

//     // 计算目标方向
//     double err_Y = targetY - Y_position;
//     double err_X = targetX - X_position;
//     double target_angle_rad = wrapAngle(atan2(err_X, err_Y));
//     double target_angle = target_angle_rad * (180 / M_PI);

//     double angle_err = 0.0;

//     //计算误差角度
//     angle_err = calculateAngleError(target_angle,return_angle);

//     // 使用 PID 控制器旋转到目标角度
//     if(fabs(angle_err)>1){
//     TurnVolpidNTo(speed_limit, target_angle,err_1, outtime);
//     }
// }

// //是否到达
// bool is_line_settled(float desired_X, float desired_Y, float desired_angle_deg, float current_X, float current_Y){
//   return( (desired_Y-Y_position) * cos(to_rad(desired_angle_deg)) <= -(desired_X-X_position) * sin(to_rad(desired_angle_deg)) );
// }

// //获取绝对朝向
// float get_absolute_heading(){
//  return (reduce_0_to_360(Inertial1.rotation()));
// }

// //限制最小速度
// float clamp_min_voltage(float drive_output, float drive_min_voltage){
//   if(drive_output < 0 && drive_output > -drive_min_voltage){
//       return -drive_min_voltage;
//   }
//   if(drive_output > 0 && drive_output < drive_min_voltage){
//     return drive_min_voltage;
//   }
//   return drive_output;
// }

// //左侧电机缩放比例
// float left_voltage_scaling(float drive_output, float heading_output){
//   float ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/12.8;
//   if (ratio > 1) {
//     return (drive_output+heading_output)/ratio;
//   }
//   return drive_output+heading_output;
// }

// //右侧电机缩放比例
// float right_voltage_scaling(float drive_output, float heading_output){
//   float ratio = std::max(std::fabs(drive_output+heading_output), std::fabs(drive_output-heading_output))/12.8;
//   if (ratio > 1) {
//     return (drive_output-heading_output)/ratio;
//   }
//   return drive_output-heading_output;
// }



//////////////////////////////////////////////////////////////////////////////


// void drive_to_pose(double X_target, double Y_target, double angle_target, double lead, double setback, double drive_min_speed, double drive_max_speed, 
// double heading_max_speed, double drive_settle_error, double drive_settle_time, double drive_timeout, double drive_starti, double heading_starti, int p_point){

// double drive_kp=0.1;
// double drive_ki=0;
// double drive_kd=0.1;
// double heading_kp=0.1;
// double heading_ki=0;
// double heading_kd=0.1;

// //选择PID参数
// switch (p_point) 
// {
//     case 0: break;
//     case 1:drive_kp = 0.15;drive_ki = 0;drive_kd =0.12;heading_kp = 0.1;heading_ki = 0.01;heading_kd = 0.1;break; 
//     case 2:drive_kp = 0.45;drive_ki = 0.001;drive_kd =1;heading_kp = 0.4;heading_ki = 0;heading_kd = 0.2;break; 
//     case 3:drive_kp = 2.8;drive_ki = 0;drive_kd =1;heading_kp = 1.8;heading_ki = 0;heading_kd = 0.8;break;//drive_to_point(0, 30, 50, 100, 40, 1, 200, 1500,0, 0, 5, 0, 0);(50,约为两个个地垫)
//     case 4:drive_kp = 2;drive_ki = 0.001;drive_kd =0;heading_kp = 1.5;heading_ki = 0.001;heading_kd = 0;break; 
//     default:drive_kp = 0.15;drive_ki = 0;drive_kd =0.12;heading_kp = 0.01;heading_ki = 0.01;heading_kd = 0.01;
//  }

//  //计算目标距离和PID
//  double target_distance = hypot(X_target-X_position, Y_target-Y_position);
//  PID drivePID(target_distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
//  PID headingPID(to_deg(atan2(X_target-X_position, Y_target-Y_position))-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
//  bool line_settled = is_line_settled(X_target, Y_target, angle_target, X_position, Y_position);
//  bool prev_line_settled = line_settled;
//  bool crossed_center_line = false;
//  bool center_line_side = is_line_settled(X_target, Y_target, angle_target+90, X_position, Y_position);
//  bool prev_center_line_side = center_line_side;

//  //循环直到到达目标且调整到正确姿态
//  while(!drivePID.is_settled()){
//    line_settled = is_line_settled(X_target, Y_target, angle_target, X_position, Y_position);
//    if(line_settled && !prev_line_settled){ break; }
//     prev_line_settled = line_settled;

//    center_line_side = is_line_settled(X_target, Y_target, angle_target+90, X_position, Y_position);
//    if(center_line_side != prev_center_line_side){
//       crossed_center_line = true;
//     }

//    target_distance = hypot(X_target-X_position, Y_target-Y_position);

//   //计算胡萝卜点
//    float carrot_X = X_target - sin(to_rad(angle_target)) * (lead * target_distance + setback);
//    float carrot_Y = Y_target - cos(to_rad(angle_target)) * (lead * target_distance + setback);

//   //计算朝向误差和路程误差
//    float drive_error = hypot(carrot_X - X_position, carrot_Y - Y_position);
//    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_X - X_position,carrot_Y - Y_position))-get_absolute_heading());

//    if(drive_error<drive_settle_error || crossed_center_line || drive_error < setback){
//       heading_error = reduce_negative_180_to_180(angle_target-get_absolute_heading()); 
//       drive_error = target_distance;
//    }

//   //计算输出
//    float drive_output = drivePID.compute(drive_error);
//    float heading_scale_factor = cos(to_rad(heading_error));
//    drive_output*=heading_scale_factor;
//    heading_error = reduce_negative_90_to_90(heading_error);
//    float heading_output = headingPID.compute(heading_error);

//    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_speed, fabs(heading_scale_factor)*drive_max_speed);
//    heading_output = clamp(heading_output, -heading_max_speed, heading_max_speed);

//    drive_output = clamp_min_voltage(drive_output, drive_min_speed);

//    //控制输出
//    LeftMotor1.spin(vex::directionType::fwd, left_voltage_scaling(drive_output, heading_output), voltageUnits::volt);
//    RightMotor1.spin(vex::directionType::fwd, right_voltage_scaling(drive_output, heading_output), voltageUnits::volt);
//    LeftMotor2.spin(vex::directionType::fwd,left_voltage_scaling(drive_output, heading_output), voltageUnits::volt);
//    RightMotor2.spin(vex::directionType::fwd, right_voltage_scaling(drive_output, heading_output), voltageUnits::volt);
//    LeftMotor3.spin(vex::directionType::fwd, left_voltage_scaling(drive_output, heading_output), voltageUnits::volt);
//    RightMotor3.spin(vex::directionType::fwd, right_voltage_scaling(drive_output, heading_output), voltageUnits::volt);

//    sleep(10);

//  }

// }


///////////////////////////////////////////////////////////////

//走到坐标
void drive_to_point(float X_target, float Y_target, float drive_min_speed, float drive_max_speed, float heading_max_speed, float drive_settle_error, 
float drive_settle_time, float drive_timeout, float drive_starti, float heading_starti, int p_point, int b_steps, float b_point){

//PID参数
float drive_kp=0.1;
float drive_ki=0;
float drive_kd=0.1;
float heading_kp=0.1;
float heading_ki=0;
float heading_kd=0.1;

//选择PID参数
switch (p_point) 
{
    case 0: break;
    case 1:drive_kp = 2.8;drive_ki = 0;drive_kd =1;heading_kp = 1.5;heading_ki = 0;heading_kd = 0.2;break; 
    case 2:drive_kp = 2.9;drive_ki = 0;drive_kd =0.5;heading_kp = 1.5;heading_ki = 0;heading_kd = 0.2;break; 
    case 3:drive_kp = 2.9;drive_ki = 0;drive_kd =1;heading_kp = 0.5;heading_ki = 0;heading_kd = 0.1;break; 
    case 4:drive_kp = 3.3;drive_ki = 0;drive_kd =1;heading_kp = 2;heading_ki = 0;heading_kd = 0.8;break;//drive_to_point(0, 30, 50, 100, 40, 1, 200, 1500,0, 0, 4, 0, 0);(30,约为一个地垫)
    case 5:drive_kp = 2.8;drive_ki = 0;drive_kd =1;heading_kp = 1.8;heading_ki = 0;heading_kd = 0.8;break;//drive_to_point(0, 30, 50, 100, 40, 1, 200, 1500,0, 0, 5, 0, 0);(50,约为两个个地垫)
    case 6:drive_kp = 2.7;drive_ki = 0;drive_kd =1;heading_kp = 1.6;heading_ki = 0;heading_kd = 0.8;break;//drive_to_point(0, 72, 50, 100, 40, 1, 200, 1500,0, 0, 6, 0, 0);(72,约为三个地垫)
    case 7:drive_kp = 4;drive_ki = 0;drive_kd =2;heading_kp = 2.3;heading_ki = 0;heading_kd = 1;break;//drive_to_point(0, 12, 90, 100, 60, 1, 200, 1500,0, 0, 7, 0, 0);(12,一个地垫内的微调)
    case 8:drive_kp = 2;drive_ki = 0.001;drive_kd =0;heading_kp = 1.5;heading_ki = 0.001;heading_kd = 0;break; //auto5
    case 9:drive_kp = 3.8;drive_ki = 0;drive_kd =1;heading_kp = 2.5;heading_ki = 0;heading_kd = 0.2;break; 
    case 10:drive_kp = 2;drive_ki = 0;drive_kd =1;heading_kp = 2;heading_ki = 0;heading_kd = 0;break; 
    default:drive_kp = 0.15;drive_ki = 0;drive_kd =0.12;heading_kp = 0.01;heading_ki = 0.01;heading_kd = 0.01;

}

// void kalmanTask(void* param) {
//     KalmanFilter2D kf(0.01); // 10ms loop
//     while (true) {
//         // 1. Predict
//         kf.predict();

//         // 2. Read odometry
//         double x_pos = (readLeft() + readRight()) / 2.0;
//         double x_vel = (readLeft() - readRight()) / 2.0;
//         double y_pos = readBack();
//         double y_vel = 0; // optional

//         Vector4d measurement;
//         measurement << x_pos, x_vel, y_pos, y_vel;

//         // 3. Update Kalman filter (only for state tracking)
//         kf.update(measurement);

//         // 4. Optional: print or log filtered position
//         Vector2d filtered_pos = kf.getPosition();
//         pros::lcd::print(0, "Filtered XY: %.2f, %.2f", filtered_pos(0), filtered_pos(1));

//         pros::delay(10); // 10ms loop
//     }
// }
 }
