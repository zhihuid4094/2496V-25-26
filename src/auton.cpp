#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"
#include "odometry.h"


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

//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111


  if (atn == 0) {
    RunpidStraightNTo(1000,2000,10,50,1300,0,0,5000,0,48);
  } 
    //blue right
  else if (atn == 1) {

      Fintake.move(127);
      Mintake.move(127);
      blocker.set_value(true)
      RunpidStraightNTo(100,600,20,100,400,0,0,4000,30,39);//the 600 is distance in mm and 30 is angle
      delay(100);
      TurnVolpidNTo(100, 45, 1, 2000,34);
      delay(50);
      RunpidStraightNTo(100,850,20,100,400,0,0,4000,75,39);
      delay(50);
      Fintake.move(0);
      Mintake.move(0);
      RunpidStraightNTo(100,-850,20,100,400,0,0,4000,0,39);
      delay(50);
      TurnVolpidNTo(100, -20, 1, 2000,34);
      delay(50);
      RunpidStraightNTo(100,200,20,100,400,0,0,4000,0,39);
       for(int i = 0; i <= 500; i+=10){
          Fintake.move(-127);
          Mintake.move(-127);
          delay(10);
        }
      RunpidStraightNTo(100,-2000,20,100,400,0,0,4000,0,39);
      delay(50);
      Fintake.move(127);
      Mintake.move(127);
      scraper.set_value(true);
      TurnVolpidNTo(100, 179, 1, 2000,34);
      delay(50);
      RunpidStraightNTo(100,200,20,100,400,0,0,4000,0,39);
      delay(50);
      RunpidStraightNTo(100,-200,20,100,400,0,0,4000,0,39);
      delay(50);
      RunpidStraightNTo(100,200,20,100,400,0,0,4000,0,39);
      delay(50);
      RunpidStraightNTo(100,-200,20,100,400,0,0,4000,0,39);
      delay(50);
      scraper.set_value(false);
      TurnVolpidNTo(100, 0, 1, 2000,34);
      linkage.set_value(true);
      RunpidStraightNTo(100,200,20,100,400,0,0,4000,0,39);
      delay(50);
  }

  else if (atn == 2) { 


  }
    else if (atn == 3) { 
      
  } 
  else if(atn == 4) {

  } 
  
  else if (atn == 5){

  } 
  
  else if (atn == 6){

  } 
  
  else if (atn == 7){

  } 
  
  else if (atn == 8){

  }
}
