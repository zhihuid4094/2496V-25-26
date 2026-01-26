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

//skills////////////////////////////////////////////////
  if (atn == 0) {
  
  } 
 //blue right//////
  else if (atn == 1) {

  }
//blue left///////
  else if (atn == 2) { 

  }
//red right//////
  else if (atn == 3) { 

  } 
//red left///////
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
      //derrick was here
}
