#include "main.h"
#include "api.h"
#include "auton.h"
#include "pid.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "odometry.h"

using namespace pros;
using namespace std;


double driveKP = 0;
double driveKI = 0;
double driveKD = 0;
double driveMAXI = 500;


double HCKP = 0;
double HCKI = 0;
double HCKD = 0;
double HCMAXI = 500;

double wallKP = 0;
double wallKI = 0;
double wallKD = 0;


void chasMove(int left, int right) { //voltage to each chassis motor
    LF.move(left);
    LM.move(left);
    LB.move(left);
    RF.move(right);
    RM.move(right);
    RB.move(right);
}



//CALCPID
double calcPID(int error, double kP=HCKP, double kI=HCKI, double kD=HCKD, double totalError=0, double prevError=0, double integralThreshold=30, double maxI=HCMAXI) {
  // calculate integral
  if (abs(error) < integralThreshold)
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

void drivePID(int desiredValue, int maxSpeed, int timeout = 5000)
{
  desiredValue = desiredValue * -1;
  bool enableDrivePID = true;
  int prevError = 0;
  double totalError = 0;
  int count = 0;


  double kP = driveKP;
  double kI = driveKI;
  double kD = driveKD;
  double maxI = driveMAXI;
  
  int integralThreshold = 150;
  int percent = 

  int time = 0;


  chassisRight.setPosition(0, degrees);
  chassisLeft.setPosition(0, degrees);
  LF.set







  // inertial.tare_heading();


  double initialValue = ((360 + (imu.get_heading() * -1)));


  if (initialValue > 180){
    initialValue = initialValue - 360;
  }


  while (enableDrivePID)
  {






    if (time > timeout){
      enableDrivePID = false;
    }


    // get position of all motors:
    int chassisRightPos = RF.get_position()
    int chassisLeftPos = LF.get_position();


    double currentIMUValue = ((360 + (imu.heading() * -1)));
    if (currentIMUValue > 180){
      currentIMUValue = currentIMUValue - 360;
    }


    if ((initialValue < 0) && (currentIMUValue > 0)){
      if ((currentIMUValue - initialValue) >= 180){
        initialValue = initialValue + 360;
        currentIMUValue = ((360 + (imu.heading() * -1)));
      }
    }


    else if ((initialValue > 0) && (currentIMUValue < 0)) {
      if ((initialValue - currentIMUValue) >= 180){
        currentIMUValue = ((360 + (imu.get_heading() * -1))) ;
      }
    }


    double headingError = initialValue -currentIMUValue;
    double headingCorrection = calcPID(headingError);






    // get avg of motors:
    int currentValue = (chassisRightPos + chassisLeftPos) / 2;


    // proportional
    int error = desiredValue - currentValue;


    // derivative
    int derivative = error - prevError;


    // integral
    if (abs(error) < integralThreshold)
    {
      totalError += error;
    }


    if (error > 0){
     if (totalError > maxI){
       totalError = maxI;
     }
     else if(totalError < maxI){
       //dont change anything
     }
   }
   else{
     if (totalError > -maxI){
       //dont change anything
     }
     else if(totalError < -maxI){
       totalError = -maxI;
     }
   }


    double speed = (error * kP + derivative * kD + totalError * kI);




    if (speed> maxSpeed){
      speed = maxSpeed;
    }
    else if (speed < -maxSpeed){
      speed = -maxSpeed;
    }


    chassisLeft.setVelocity(speed - headingCorrection , percent);
    chassisRight.setVelocity(speed + headingCorrection , percent);


    chassisLeft.spin(forward);
    chassisRight.spin(forward);



    prevError = error;


    if (abs(error) < 15)
    {
      count++;
    }


    if (count > 50)
    {
      enableDrivePID = false;
    }
    delay(2);
  }


  chassisRight.stop();
 chassisLeft.stop();


}

void driveWALLPID(int desiredValue, int maxSpeed, int wallDistanceTarget,
                  int wallOffStart1 = -1, int wallOnAgain1 = -1, 
                  int wallOffStart2 = -1, int wallOnAgain2 = -1,
                  int timeout = 5000)
{
    desiredValue = desiredValue * -1;

    bool enableDrivePID = true;

    int prevError = 0;
    double totalError = 0;

    int count = 0;
    int time = 0;

    // Wall PID variables
    double wallPrevError = 0;
    double wallTotalError = 0;

    chassisRight.setPosition(0, degrees);
    chassisLeft.setPosition(0, degrees);

    double initialValue = ((360 + (imu.get_heading() * -1)));
    if (initialValue > 180){
        initialValue = initialValue - 360;
    }

    while (enableDrivePID)
    {
        if (time > timeout){
            enableDrivePID = false;
        }

        // Motor positions
        int chassisRightPos = chassisRight.position(degrees);
        int chassisLeftPos = chassisLeft.position(degrees);

        // Average encoder position
        int currentValue = (chassisRightPos + chassisLeftPos) / 2;
        int pos = abs(currentValue);

        // Gyro reading
        double currentIMUValue = ((360 + (imu.get_heading() * -1)));
        if (currentIMUValue > 180){
            currentIMUValue = currentIMUValue - 360;
        }
        double headingError = initialValue - currentIMUValue;
        double headingCorrection = calcPID(headingError);

        // =====================
        // DRIVE PID
        // =====================
        int error = desiredValue - currentValue;
        int derivative = error - prevError;

        if (abs(error) < 150){  // integral threshold
            totalError += error;
        }
        if (totalError > driveMAXI) totalError = driveMAXI;
        if (totalError < -driveMAXI) totalError = -driveMAXI;

        double speed = error * driveKP + derivative * driveKD + totalError * driveKI;
        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < -maxSpeed) speed = -maxSpeed;

        

        // =====================
        // WALL FOLLOWING PID
        // =====================
        double wallDistance = distance.get(); // always measure
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

        if (wallOffStart1 != -1 && pos > wallOffStart1){
            wallEnabled = false;
        }
        if (wallOnAgain1 != -1 && pos > wallOnAgain1){
            wallEnabled = true;
        }
        if (wallOffStart2 != -1 && pos > wallOffStart2){
            wallEnabled = false;
        }
        if (wallOnAgain2 != -1 && pos > wallOnAgain2){
            wallEnabled = true;
        }

        // =====================
        // COMBINE CORRECTIONS
        // =====================
        double correctionDirection = (desiredValue > 0) ? -1.0 : 1.0;
double totalCorrection = headingCorrection;
if (wallEnabled){
    totalCorrection += wallCorrection * correctionDirection;
}

        // =====================
        // APPLY MOTOR SPEEDS
        // =====================
        chassisLeft.setVelocity(speed - totalCorrection, percent);
        chassisRight.setVelocity(speed + totalCorrection, percent);

        LF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        RF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        LM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        RM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        LB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        RB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

        chassisLeft.spin(forward);
        chassisRight.spin(forward);

        prevError = error;

        if (abs(error) < 50){
            count++;
        }
        if (count > 50){
            enableDrivePID = false;
        }

        delay(2);
        time += 2;
    }

    chasMove(0,0);

}
 



