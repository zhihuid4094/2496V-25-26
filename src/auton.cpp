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

  //KEY (LOOK HERE IF YOU ARE LOST ON WHAT PARAMETERS MEAN)  
//---------------------------------------------------------------------------------------------------------------------------------------
//drivePID(Target, Maxspeed, Timeout, Error Width, Time Within Error, When To Start Slow Down, Min Speed, chainValue, Trigger Distance, Speed) 
//
//            drivePID(1000, 127, 5000, 15, 50, 700, 30, 0, -1, 0);       (10 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//drivePIDW(Target, Maxspeed, Timeout, Distance from Wall, -1, -1, -1, -1, Error Width,
//           Time Within Error, 0 = Left Sensor/1 = Right Sensor, When To Start Slow Down, Min Speed, chainValue, Trigger Distance, Speed) 
//
//            drivePIDW(1000, 127, 5000, 360, -1, -1, -1, -1, 15, 50, 0, 700, 30, 0, -1, 0);       (16 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//turnPID(Target, MaxSpeed, Timeout, Error Width, Time Within Error)
//
//            turnPID(90, 127, 5000, 15, 50);     (5 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//driveArcL(Target, radius, timeout, max Speed, Error Width, Time in Error, chainValue)
//
//            driveArcL(90, 500, 5000, 100, 15, 50, 0)      (7 Parameters)  
//            driveArcR(90, 500, 5000, 10, 15, 50, 0)      (7 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//chasSlow(speed, time), for like reseting in goal position
//
//            chasSlow(-50, 400);
//---------------------------------------------------------------------------------------------------------------------------------------
//Copy and Paste from Here
//  drivePID(-1000, 50, 3000, 15, 50, 700, 40, 0, -1, 0);
//  drivePIDW(-1000, 50, 3000, 400, -1, -1, -1, -1, 15, 50, 0, 700, 40, 0, -1, 0); 
//  turnPID(90, 50, 3000, 1, 50);
//  driveArcR(90, 400, 5000, 100, 15, 50, 0);
//---------------------------------------------------------------------------------------------------------------------------------------

                      
  if (atn == 0) {
    //skills


    // drivePID(300, 80, 1000, 8, 50, 250, 50, 250, -1, 0);
    //driveArcR(90, 100, 5000, 60, 5, 4500, 0);
    // drivePID(500, 80, 1000, 8, 50, 250, 50, 0, -1, 0);
    

turnPID(170, 75, 1000, 1.6, 30, 0);    // ~170 turn
turnPID(165, 75, 1000, 1.6, 30, 0);    // ~5 turn
turnPID(0, 75, 1000, 1.6, 30, 0);      // ~165 turn
turnPID(-170, 75, 1000, 1.6, 30, 0);   // ~170 turn
turnPID(-155, 75, 1000, 1.6, 30, 0);   // ~15 turn
turnPID(-130, 75, 1000, 1.6, 30, 0);   // ~25 turn
turnPID(-90, 75, 1000, 1.6, 30, 0);    // ~40 turn
turnPID(-150, 75, 1000, 1.6, 30, 0);   // ~60 turn
turnPID(-85, 75, 1000, 1.6, 30, 0);    // ~65 turn
turnPID(85, 75, 1000, 1.6, 30, 0);     // ~170 turn
turnPID(95, 75, 1000, 1.6, 30, 0);     // ~10 turn
turnPID(75, 75, 1000, 1.6, 30, 0);     // ~20 turn
turnPID(170, 75, 1000, 1.6, 30, 0);    // ~95 turn
turnPID(50, 75, 1000, 1.6, 30, 0);     // ~120 turn
turnPID(45, 75, 1000, 1.6, 30, 0);     // ~5 turn
turnPID(-45, 75, 1000, 1.6, 30, 0);    // ~90 turn
turnPID(130, 75, 1000, 1.6, 30, 0);    // ~175 turn
turnPID(120, 75, 1000, 1.6, 30, 0);    // ~10 turn
turnPID(-60, 75, 1000, 1.6, 30, 0);    // ~180 turn
turnPID(-45, 75, 1000, 1.6, 30, 0);    // ~15 turn
turnPID(30, 75, 1000, 1.6, 30, 0);     // ~75 turn
turnPID(15, 75, 1000, 1.6, 30, 0);     // ~15 turn
turnPID(-165, 75, 1000, 1.6, 30, 0);   // ~180 turn
turnPID(0, 75, 1000, 1.6, 30, 0);      // ~165 turn
  

     
  } 
 

  else if (atn == 1) {
    //3+4 left
    intake.move(127);
    drivePID(800, 50, 1500, 10, 50, 500, 50, 200, -1, 0);
    driveArcL(42, 1100, 1500, 30, 15, 50, 0);
    turnPID(3, 75, 1000, 1.6, 30, 5); 
    drivePID(-1300, 127, 1000, 10, 50, 700, 50, 200, -1, 0);
    driveArcL(-135, 160, 1000, 60, 15, 50, 0);
    turnPID(-140, 75, 100, 1.6, 30, 0); 
    lift.set_value(true);
    drivePIDW(-1600, 70, 2500, 460, -1, -1, -1, -1, 15, 50, 1, 699, 127, 0, -1, 0); 
    chasSlow(-100, 300);
    blocker.set_value(false);
        leverPID(750, 95, 2000, 100, 10, 550, 50);
        pros::delay(500);
        blocker.set_value(true);
        leverPID(-550, 127, 2000, 100, 10, 400, 50);
        Lever.move(-20);
        pros::delay(800);
        Lever.move(0);

    






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
    drivePID(600, 80, 3000, 15, 50, 999, 80, 500, -1, 0);
    driveArcR(95, 100, 3000, 100, 15, 50, 300);
    drivePID(900, 100, 3000, 15, 50, 999, 100, 500, -1, 0);
    turnPID(158, 100, 400, 1, 50, 0);
    drivePIDW(-1000, 100, 3000, 460, -1, -1, -1, -1, 15, 50, 0, 999, 100, 0, -1, 0); 

  } 
  

  else if (atn == 7){
    //sawp

  } 

  else if (atn == 8){
    //misc

  }
}

