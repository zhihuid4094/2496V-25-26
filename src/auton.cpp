#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "derrickPID.h"


using namespace std;
using namespace pros;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or t
 * he VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
  
  //INDEX
    //AUTON 0: SKILLS
    //AUTON 1: blue right
    //AUTON 2: blue left
    //AUTON 3:red right
    //AUTON 4: red left
    //AUTON 5: elims

//---------------------------------------------------------------------------------------------------------------------------------------
//drivePID(Target, Maxspeed, Timeout, Error Width, Time Within Error, When To Start Slow Down, Min Speed, Kp Selector, chainValue) 

//            drivePID(1000, 127, 5000, 15, 50, 700, 30, 0, 0);       (9 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//drivePIDW(Target, Maxspeed, Timeout, Distance from Wall, -1, -1, -1, -1, Error Width,
//           Time Within Error, 0 = Left Sensor/1 = Right Sensor, When To Start Slow Down, Min Speed, Kp Selector, chainValue) 

//            drivePIDW(1000, 127, 5000, 360, -1, -1, -1, -1, 15, 50, 0, 700, 30, 0, 0);       (15 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//turnPID(Target, MaxSpeed, Timeout, Error Width, Time Within Error, Kp Selector)

//            turnPID(90, 127, 5000, 15, 50, 0);     (6 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//driveArcL(Target, radius, timeout, max Speed, Error Width, Time in Error, chainValue)

//            driveArcL(90, 500, 5000, 100, 15, 50, 0)      (7 Parameters)  
//            driveArcR(90, 500, 5000, 10, 15, 50, 0)      (7 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
                      
  if (atn == 0) {
    //skills
  } 
 

  else if (atn == 1) {
    //3+4 left
     pros::delay(2000);

     //drivePIDW(1000, 50, 5000, 360, -1, -1, -1, -1, 15, 4900, 0, 1000, 50, 0, 0);
    //turnPID(90, 80, 5000, .1, 4500, 0);
    
    

    //drivePID(1000, 50, 5000, 5, 50, 1000, 50, 0, 0); 


    //driveArcR(90, 400, 5000, 50, 15, 4900, 0);
    //drivePID(1000, 500, 5000, 15, 50, 1000, 100, 0, 0); 
  }


  else if (atn == 2) { 
   //3+4 right
  } 

  else if (atn == 3 ){
   //7 left
  } 


  else if(atn == 4) {
    //7 right
  } 

  else if (atn == 5){
    //4 Fast Left
  } 
  

  else if (atn == 6){
    //4 Fast Right
  } 
  

  else if (atn == 7){
    //sawp
     
  } 

  else if (atn == 8){
    //misc
  }
}

