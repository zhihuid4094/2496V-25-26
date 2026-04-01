#include "main.h"
#include "api.h"
#include "auton.h"
#include "derrickPID.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"


using namespace pros;
using namespace std;

double universal_target_heading = 0 ;

double driveKP = .3;
double driveKI = 0;
double driveKD = 8;
double driveMAXI = 500;


double HCKP = .6;
double HCKI = .001;
double HCKD = .1;
double HCMAXI = 500;

double wallKP = .075;
double wallKI = 0;
double wallKD = 0;  


double turnKP = 1.65;
double turnKI = 0;
double turnKD = 8;
double turnMAXI = 500;

double arcKP = 6;
double arcKI = 0.002;
double arcKD = 6;
double arcMAXI = 50;

// ================================
// DRIVE p_point constants 
// ================================
double driveKP1 = 0.5;  double driveKI1 = 0.0;  double driveKD1 = 0.2;  // case 1 < 500
double driveKP2 = 0.3;  double driveKI2 = 0.0;  double driveKD2 = 0.4;  // case 2 < 1000
double driveKP3 = 0.2;  double driveKI3 = 0.0;  double driveKD3 = 0.5;  // case 3 < 1500
double driveKP4 = 0.5;  double driveKI4 = 0.0;  double driveKD4 = 0.2;  // case 4 < 2000
double driveKP5 = 0.3;  double driveKI5 = 0.0;  double driveKD5 = 0.4;  // case 5 < 2500
double driveKP6 = 0.2;  double driveKI6 = 0.0;  double driveKD6 = 0.5;  // case 6 < 3000

// ================================
// TURN p_point constants 
// ================================
double turnKP1 = 0.0;  double turnKI1 = 0.0;  double turnKD1 = 0.0;  // case 1 < 5
double turnKP2 = 0.0;  double turnKI2 = 0.0;  double turnKD2 = 0.0;  // case 2 < 10
double turnKP3 = 0.0;  double turnKI3 = 0.0;  double turnKD3 = 0.0;  // case 3 < 20
double turnKP4 = 0.0;  double turnKI4 = 0.0;  double turnKD4 = 0.0;  // case 4 < 40
double turnKP5 = 0.0;  double turnKI5 = 0.0;  double turnKD5 = 0.0;  // case 5 < 60
double turnKP6 = 0.0;  double turnKI6 = 0.0;  double turnKD6 = 0.0;  // case 6 < 80
double turnKP7 = 0.0;  double turnKI7 = 0.0;  double turnKD7 = 0.0;  // case 7 < 100
double turnKP8 = 0.0;  double turnKI8 = 0.0;  double turnKD8 = 0.0;  // case 8 < 120
double turnKP9 = 0.0;  double turnKI9 = 0.0;  double turnKD9 = 0.0;  // case 9 < 140
double turnKP10 = 0.0;  double turnKI10 = 0.0;  double turnKD10 = 0.0;  // case 10 < 160
double turnKP11 = 0.0;  double turnKI11 = 0.0;  double turnKD11 = 0.0;  // case 11 < 180


void chasMove(int left, int right) { //voltage to each chassis motor
    LF.move(left);
    LM.move(left);
    LB.move(left);
    RF.move(right);
    RM.move(right);
    RB.move(right);
}

void resetEncoders(){
  LF.tare_position();
  LM.tare_position();
  LB.tare_position();
  RF.tare_position();
  RM.tare_position();
  RB.tare_position();
}

void chasBrake(){
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void chasSlow(int speed, int ms) {
    chasMove(speed, speed);
    delay(ms);
    chasBrake();
}


//--------------------------------------------------------------------------------------------
//CALCPID
//--------------------------------------------------------------------------------------------

double calcPID(int error, double kP=HCKP, double kI=HCKI, double kD=HCKD, double totalError=0,
   double prevError=0, double integralThreshold=30, double maxI=HCMAXI) {
    
  // calculate integral
  if (fabs(error) < integralThreshold)
  {
    totalError += error;
  }

  if (error > 0){
   if (totalError > maxI){
     totalError = maxI;
   }
   else if(totalError < maxI){
   }
  }
  else{
   if (totalError > -maxI){
   }
   else if(totalError < -maxI){
     totalError = -maxI;
   }
  }

  // calculate derivative
   float derivative = error - prevError;
   prevError = error;


   // calculate output
   double speed = (error * kP) + (totalError * kI) + (derivative * kD);


  if (speed > 127){
    speed = 127;
  }
  else if (speed < -127){
    speed = -127;
  }

  return speed;
}

//--------------------------------------------------------------------------------------------
// DRIVE STRAIGHT
//--------------------------------------------------------------------------------------------
void drivePID(int desiredValue, int maxSpeed, int timeout = 5000,
              int errorThreshold = 15, int settleCount = 50,
              int dec_point = -1, int minSpeed = 10, int p_point = 0, 
              int chainValue = 0,  int triggerDist = -1, int triggerSpeed = 0)
{
    bool enableDrivePID = true;
    double prevError = 0;
    double totalError = 0;
    int count = 0;
    int time = 0;

    double kP = driveKP;
    double kI = driveKI;
    double kD = driveKD;
    double maxI = driveMAXI;
    int integralThreshold = 150;

    switch (p_point)
{
    case 0: break;
    case 1: kP = driveKP1; kI = driveKI1; kD = driveKD1; break;
    case 2: kP = driveKP2; kI = driveKI2; kD = driveKD2; break;
    case 3: kP = driveKP3; kI = driveKI3; kD = driveKD3; break;
    case 4: kP = driveKP4; kI = driveKI4; kD = driveKD4; break;
    case 5: kP = driveKP5; kI = driveKI5; kD = driveKD5; break;
    case 6: kP = driveKP6; kI = driveKI6; kD = driveKD6; break;
    default: break;
}

    resetEncoders();
    con.clear();

     int pidTarget = (chainValue != 0) ? desiredValue + chainValue : desiredValue;


    int startTime = pros::millis();
    double acc = 0.4; // speed units per ms, higher = faster ramp (TUNE)

    while (enableDrivePID)
    {
        if (time > timeout) {
            enableDrivePID = false;
        }

        // get position of motors
        double chassisLeftPos  = (LF.get_position() + LM.get_position() + LB.get_position()) / 3;
        double chassisRightPos = (RF.get_position() + RM.get_position() + RB.get_position()) / 3;

        // heading logic
        double currentIMUValue = imu.get_heading();
        if (currentIMUValue > 180) {
            currentIMUValue = currentIMUValue - 360;
        }

        if ((universal_target_heading < 0) && (currentIMUValue > 0)) {
            if ((currentIMUValue - universal_target_heading) >= 180) {
                universal_target_heading = universal_target_heading + 360;
                currentIMUValue = imu.get_heading();
            }
        }
        else if ((universal_target_heading > 0) && (currentIMUValue < 0)) {
            if ((universal_target_heading - currentIMUValue) >= 180) {
                currentIMUValue = imu.get_heading();
            }
        }

        double headingError = universal_target_heading - currentIMUValue;
        double headingCorrection = calcPID(headingError);

        // get avg of motors
        double currentValue = (chassisRightPos + chassisLeftPos) / 2;
        double currentPos = fabs(currentValue);

        // proportional
        double error = pidTarget - currentValue;

        if (triggerDist != -1 && fabs(error) <= triggerDist) {
            Lintake.move(triggerSpeed);
        }

        // derivative
        double derivative = error - prevError;

        // integral
        if (fabs(error) < integralThreshold) {
            totalError += error;
        }

        if (error > 0) {
            if (totalError > maxI) {
                totalError = maxI;
            }
        }
        else {
            if (totalError < -maxI) {
                totalError = -maxI;
            }
        }

        double speed = (error * kP + derivative * kD + totalError * kI);

        // acceleration ramp
        double rampedMax = acc * (pros::millis() - startTime);
        if (rampedMax > maxSpeed) rampedMax = maxSpeed;

        // deceleration
        if (dec_point != -1 && currentPos > dec_point) {
            double decMax = maxSpeed - (currentPos - dec_point) / 10.0;
            if (decMax < minSpeed) decMax = minSpeed;
            if (rampedMax > decMax) rampedMax = decMax;
        }

        // clamp speed to rampedMax (handles both accel and decel)
        if (speed > rampedMax)  speed = rampedMax;
        if (speed < -rampedMax) speed = -rampedMax;


        chasMove(speed + headingCorrection, speed - headingCorrection);

        prevError = error;

        if (chainValue != 0) {
            // chain mode: exit immediately when real target is reached
            if (desiredValue > 0 && currentValue >= desiredValue) {
                enableDrivePID = false;
            }
            else if (desiredValue < 0 && currentValue <= desiredValue) {
                enableDrivePID = false;
            }
        }
        else {
            // normal mode: settle near target
            if (fabs(error) < errorThreshold) {
                count++;
            }
            if (count > settleCount) {
                enableDrivePID = false;
            }
        }
        
        if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0){
            con.print(0, 0, "error: %.2f          ", error);
        } else if (time % 100 == 0 && time % 150 != 0){
            con.print(1, 0, "IMU: %.2f          ", imu.get_heading());
        } else if (time % 150 == 0){
            con.print(2, 0, "time: %d            ", time);
        }

        delay(10);
        time += 10;
    }

    chasBrake();
}

//--------------------------------------------------------------------------------------------
// DRIVE STRAIGHT USING DISTANCE SENSOR
//--------------------------------------------------------------------------------------------

void drivePIDW(int desiredValue, int maxSpeed, int timeout = 5000, int wallDistanceTarget = 5000,
                  int wallOffStart1 = -1, int wallOnAgain1 = -1,
                  int wallOffStart2 = -1, int wallOnAgain2 = -1,
                  int errorThreshold = 15, int settleCount = 50,
                  int sensorSide = 0, int dec_point = -1, int minSpeed = 10, int p_point = 0,
                  int chainValue = 0, int triggerDist = -1, int triggerSpeed = 0)
{
    bool enableDrivePID = true;

    double prevError = 0;
    double totalError = 0;

    int count = 0;
    int time = 0;

    double wallPrevError = 0;
    double wallTotalError = 0;

    double kP = driveKP;
    double kI = driveKI;
    double kD = driveKD;
    double maxI = driveMAXI;
    int integralThreshold = 150;

    switch (p_point)
    {
        case 0: break;
        case 1: kP = driveKP1; kI = driveKI1; kD = driveKD1; break;
        case 2: kP = driveKP2; kI = driveKI2; kD = driveKD2; break;
        case 3: kP = driveKP3; kI = driveKI3; kD = driveKD3; break;
        case 4: kP = driveKP4; kI = driveKI4; kD = driveKD4; break;
        case 5: kP = driveKP5; kI = driveKI5; kD = driveKD5; break;
        case 6: kP = driveKP6; kI = driveKI6; kD = driveKD6; break;
        default: break;
    }

    resetEncoders();
    con.clear();

    double pidTarget = (chainValue != 0) ? desiredValue + chainValue : desiredValue;

    int startTime = pros::millis();
    double acc = 0.4;

    while (enableDrivePID)
    {
        if (time > timeout) {
            enableDrivePID = false;
        }

        double chassisLeftPos  = (LF.get_position() + LM.get_position() + LB.get_position()) / 3.0;
        double chassisRightPos = (RF.get_position() + RM.get_position() + RB.get_position()) / 3.0;

        double currentValue = (chassisRightPos + chassisLeftPos) / 2.0;
        double pos = fabs(currentValue);
        double currentPos = pos;

        // heading logic
        double currentIMUValue = imu.get_heading();
        if (currentIMUValue > 180) {
            currentIMUValue = currentIMUValue - 360;
        }

        if ((universal_target_heading < 0) && (currentIMUValue > 0)) {
            if ((currentIMUValue - universal_target_heading) >= 180) {
                universal_target_heading = universal_target_heading + 360;
                currentIMUValue = imu.get_heading();
            }
        }
        else if ((universal_target_heading > 0) && (currentIMUValue < 0)) {
            if ((universal_target_heading - currentIMUValue) >= 180) {
                currentIMUValue = imu.get_heading();
            }
        }

        double headingError = universal_target_heading - currentIMUValue;
        double headingCorrection = calcPID(headingError);

        // =====================
        // DRIVE PID
        // =====================
        double error = pidTarget - currentValue;

        if (triggerDist != -1 && abs(error) <= triggerDist) {
            Lintake.move(triggerSpeed);
        }
        
        double derivative = error - prevError;

        if (fabs(error) < integralThreshold) {
            totalError += error;
        }
        if (totalError > maxI) totalError = maxI;
        if (totalError < -maxI) totalError = -maxI;

        double speed = error * kP + derivative * kD + totalError * kI;

        // acceleration ramp
        double rampedMax = acc * (pros::millis() - startTime);
        if (rampedMax > maxSpeed) rampedMax = maxSpeed;

        // deceleration
        if (dec_point != -1 && currentPos > dec_point) {
            double decMax = maxSpeed - (currentPos - dec_point) / 10.0;
            if (decMax < minSpeed) decMax = minSpeed;
            if (rampedMax > decMax) rampedMax = decMax;
        }

        // clamp speed
        if (speed > rampedMax)  speed = rampedMax;
        if (speed < -rampedMax) speed = -rampedMax;

        // =====================
        // WALL FOLLOWING PID
        // =====================
        double wallDistance = (sensorSide == 0) ? distanceSensorL.get() : distanceSensorR.get();

        double wallError = wallDistanceTarget - wallDistance;
        double wallDerivative = wallError - wallPrevError;
        wallTotalError += wallError;

        double wallCorrection = wallError * wallKP + wallDerivative * wallKD + wallTotalError * wallKI;
        if (wallCorrection > 20) wallCorrection = 20;
        if (wallCorrection < -20) wallCorrection = -20;
        wallPrevError = wallError;

        // =====================
        // WALL ENABLE/DISABLE
        // =====================
        bool wallEnabled = true;

        if (wallOffStart1 != -1 && pos > wallOffStart1) wallEnabled = false;
        if (wallOnAgain1  != -1 && pos > wallOnAgain1)  wallEnabled = true;
        if (wallOffStart2 != -1 && pos > wallOffStart2) wallEnabled = false;
        if (wallOnAgain2  != -1 && pos > wallOnAgain2)  wallEnabled = true;

        // =====================
        // COMBINE CORRECTIONS
        // =====================
        double totalCorrection = headingCorrection;
        if (wallEnabled) {
            double directedWallCorrection = (desiredValue > 0) ? wallCorrection : -wallCorrection;
            if (sensorSide == 0) {
                totalCorrection += directedWallCorrection;
            } else {
                totalCorrection -= directedWallCorrection;
            }
        }

        chasMove(speed + totalCorrection, speed - totalCorrection);

        prevError = error;

        if (chainValue != 0) {
            if (desiredValue > 0 && currentValue >= desiredValue) {
                enableDrivePID = false;
            }
            else if (desiredValue < 0 && currentValue <= desiredValue) {
                enableDrivePID = false;
            }
        }
        else {
            if (fabs(error) < errorThreshold) {
                count++;
            }
            if (count > settleCount) {
                enableDrivePID = false;
            }
        }

        if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0){
            con.print(0, 0, "error: %.2f          ", error);
        } else if (time % 100 == 0 && time % 150 != 0){
            con.print(1, 0, "IMU: %.2f          ", imu.get_heading());
        } else if (time % 150 == 0){
            con.print(2, 0, "distance: %d            ", distanceSensorL.get());
        }

        delay(10);
        time += 10;
    }

    chasBrake();
}

//--------------------------------------------------------------------------------------------
// TURNING
//--------------------------------------------------------------------------------------------

void turnPID(double desiredValue, int topSpeed = 127, int timeout = 5000, int errorThreshold = 1, int settleCount = 50, int p_point = 0)
{   
    bool enableTurnPID = true;
    double prevError = 0;
    double totalError = 0;
    int count = 0;
    int time = 0;

    double kP = turnKP;
    double kI = turnKI;
    double kD = turnKD;
    double maxI = turnMAXI;
    int integralThreshold = 30;


     switch (p_point)
{
    case 0: break;
    case 1: kP = turnKP1; kI = turnKI1; kD = turnKD1; break;
    case 2: kP = turnKP2; kI = turnKI2; kD = turnKD2; break;
    case 3: kP = turnKP3; kI = turnKI3; kD = turnKD3; break;
    case 4: kP = turnKP4; kI = turnKI4; kD = turnKD4; break;
    case 5: kP = turnKP5; kI = turnKI5; kD = turnKD5; break;
    case 6: kP = turnKP6; kI = turnKI6; kD = turnKD6; break;
    case 7: kP = turnKP7; kI = turnKI7; kD = turnKD7; break;
    case 8: kP = turnKP8; kI = turnKI8; kD = turnKD8; break;
    case 9: kP = turnKP9; kI = turnKI9; kD = turnKD9; break;
    case 10: kP = turnKP10; kI = turnKI10; kD = turnKD10; break;
    case 11: kP = turnKP11; kI = turnKI11; kD = turnKD11; break;
    default: break;
}

    con.clear();

    while (enableTurnPID)
    {
        if (time > timeout) {
            enableTurnPID = false;
        }

        // proportional
        double error = fmod((desiredValue - imu.get_heading() + 540), 360) - 180;

        // derivative
        double derivative = error - prevError;

        // integral
        if (fabs(error) < integralThreshold) {
            totalError += error;
        }

        if (error > 0) {
            if (totalError > maxI) {
                totalError = maxI;
            }
        }
        else {
            if (totalError < -maxI) {
                totalError = -maxI;
            }
        }

        double speed = (error * kP + derivative * kD + totalError * kI);

        if (speed > topSpeed) {
            speed = topSpeed;
        }
        else if (speed < -topSpeed) {
            speed = -topSpeed;
        }

        // left spins opposite to right for turning
        chasMove(speed, -speed);

        prevError = error;

        if (fabs(error) < errorThreshold) {
            count++;
        }
        if (count > settleCount) {
            enableTurnPID = false;
        }
        
        if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0){
            con.print(0, 0, "error: %.2f        ", -error);
        } else if (time % 100 == 0 && time % 150 != 0){
            con.print(1, 0, "IMU: %.2f          ", imu.get_heading());
        } else if (time % 150 == 0){
            con.print(2, 0, "time: %d            ", time);
        }

        delay(10);
        time += 10;
    }

    chasBrake();
    universal_target_heading = desiredValue;
}


//--------------------------------------------------------------------------------------------
// ARC LEFT
//--------------------------------------------------------------------------------------------

void driveArcL(double theta, double radius, int timeout = 5000, int speed = 100, 
               int errorThreshold = 1, int settleCount = 50, int chainValue = 0)
{
    double pi = 3.14159265359;

    double ltarget = (theta / 360.0) * 2 * pi * radius;
    double rtarget = (theta / 360.0) * 2 * pi * (radius + 360);

    // extend targets if chaining
    double ltargetPID = (chainValue != 0) ? ltarget + chainValue : ltarget;
    double rtargetPID = (chainValue != 0) ? rtarget + chainValue : rtarget;

    double speedProp = ltarget / rtarget;
    if (speedProp < .35) speedProp = .35;

    double kP = arcKP;
    double kI = arcKI;
    double kD = arcKD;
    double maxI = arcMAXI;
    double arcHeadingKP = 1;
    int integralThreshold = 150;

    double prevError = 0;
    double totalError = 0;

    resetEncoders();
    con.clear();

    double initialHeading = imu.get_heading();

    bool enableArcPID = true;
    int count = 0;
    int time = 0;

    while (enableArcPID)
    {
        if (time > timeout) enableArcPID = false;

        double encoderAvgL = (LF.get_position() + LM.get_position() + LB.get_position()) / 3.0;
        double encoderAvgR = (RF.get_position() + RM.get_position() + RB.get_position()) / 3.0;

        double right_error = rtargetPID - encoderAvgR;
        double left_error  = ltargetPID - encoderAvgL;

        double leftcorrect = (encoderAvgL * 360.0) / (2.0 * pi * radius);

        double currentIMUValue = imu.get_heading();
        if (currentIMUValue > 180) currentIMUValue -= 360;

        double expectedHeading = initialHeading - leftcorrect;
        if (expectedHeading > 180) expectedHeading -= 360;

        if ((expectedHeading < 0) && (currentIMUValue > 0)) {
            if ((currentIMUValue - expectedHeading) >= 180) {
                expectedHeading += 360;
                currentIMUValue = imu.get_heading();
            }
        } else if ((expectedHeading > 0) && (currentIMUValue < 0)) {
            if ((expectedHeading - currentIMUValue) >= 180) {
                currentIMUValue = imu.get_heading();
            }
        }

        double headingError = expectedHeading - currentIMUValue;
        double headingCorrection = headingError * arcHeadingKP;

        double derivative = right_error - prevError;

        if (fabs(right_error) < integralThreshold) totalError += right_error;
        if (totalError >  maxI) totalError =  maxI;
        if (totalError < -maxI) totalError = -maxI;

        double maxVoltage = 127.0 * (speed / 100.0);
        double speedR = right_error * kP + derivative * kD + totalError * kI;
        if (speedR >  maxVoltage) speedR =  maxVoltage;
        if (speedR < -maxVoltage) speedR = -maxVoltage;

        chasMove(speedProp * speedR + headingCorrection,
                 speedR            - headingCorrection);

        prevError = right_error;

        // exit conditions
        if (chainValue != 0) {
            if (encoderAvgR >= rtarget) enableArcPID = false;
        } else {
            if ((fabs(left_error) < errorThreshold) && (fabs(right_error) < errorThreshold)) count++;
            if (count > settleCount) enableArcPID = false;
        }

        if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0){
            con.print(0, 0, "error: %.5f    ", right_error);
        } else if (time % 100 == 0 && time % 150 != 0){
            con.print(1, 0, "imu: %.3f          ", imu.get_heading());
        } else if (time % 150 == 0){
            con.print(2, 0, "time: %d           ", time);
        
        }
        delay(10);
        time += 10;
    }

    chasBrake();
    universal_target_heading -= theta;
}

//--------------------------------------------------------------------------------------------
// ARC RIGHT
//--------------------------------------------------------------------------------------------

void driveArcR(double theta, double radius, int timeout = 5000, int speed = 100, 
               int errorThreshold = 1, int settleCount = 50, int chainValue = 0)
{
    double pi = 3.14159265359;

    double rtarget = (theta / 360.0) * 2 * pi * radius;
    double ltarget = (theta / 360.0) * 2 * pi * (radius + 360);

    // extend targets if chaining
    double ltargetPID = (chainValue != 0) ? ltarget + chainValue : ltarget;
    double rtargetPID = (chainValue != 0) ? rtarget + chainValue : rtarget;

    double speedProp = rtarget / ltarget;
    if (speedProp < .35) speedProp = .35;

    double kP = arcKP;
    double kI = arcKI;
    double kD = arcKD;
    double maxI = arcMAXI;
    double arcHeadingKP = 1;
    int integralThreshold = 150;

    double prevError = 0;
    double totalError = 0;

    resetEncoders();
    con.clear();

    double initialHeading = imu.get_heading();

    bool enableArcPID = true;
    int count = 0;
    int time = 0;

    while (enableArcPID)
    {
        if (time > timeout) enableArcPID = false;

        double encoderAvgL = (LF.get_position() + LM.get_position() + LB.get_position()) / 3.0;
        double encoderAvgR = (RF.get_position() + RM.get_position() + RB.get_position()) / 3.0;

        double right_error = rtargetPID - encoderAvgR;
        double left_error  = ltargetPID - encoderAvgL;

        double rightcorrect = (encoderAvgR * 360.0) / (2.0 * pi * radius);

        double currentIMUValue = imu.get_heading();
        if (currentIMUValue > 180) currentIMUValue -= 360;

        double expectedHeading = initialHeading + rightcorrect;
        if (expectedHeading > 180) expectedHeading -= 360;

        if ((expectedHeading < 0) && (currentIMUValue > 0)) {
            if ((currentIMUValue - expectedHeading) >= 180) {
                expectedHeading += 360;
                currentIMUValue = imu.get_heading();
            }
        } else if ((expectedHeading > 0) && (currentIMUValue < 0)) {
            if ((expectedHeading - currentIMUValue) >= 180) {
                currentIMUValue = imu.get_heading();
            }
        }

        double headingError = expectedHeading - currentIMUValue;
        double headingCorrection = headingError * arcHeadingKP;

        double derivative = left_error - prevError;

        if (fabs(left_error) < integralThreshold) totalError += left_error;
        if (totalError >  maxI) totalError =  maxI;
        if (totalError < -maxI) totalError = -maxI;

        double maxVoltage = 127.0 * (speed / 100.0);
        double speedL = left_error * kP + derivative * kD + totalError * kI;
        if (speedL >  maxVoltage) speedL =  maxVoltage;
        if (speedL < -maxVoltage) speedL = -maxVoltage;

        chasMove(speedL            + headingCorrection,
                 speedProp * speedL - headingCorrection);

        prevError = left_error;

        // exit conditions
        if (chainValue != 0) {
            if (encoderAvgL >= ltarget) enableArcPID = false;
        } else {
            if ((fabs(left_error) < errorThreshold) && (fabs(right_error) < errorThreshold)) count++;
            if (count > settleCount) enableArcPID = false;
        }

        if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0){
            con.print(0, 0, "error: %.5f    ", left_error);
        } else if (time % 100 == 0 && time % 150 != 0){
            con.print(1, 0, "imu: %.3f          ", imu.get_heading());
        } else if (time % 150 == 0){
            con.print(2, 0, "time: %d           ", time);
        
        }
        delay(10);
        time += 10;
    }

    chasBrake();
    universal_target_heading += theta;
}