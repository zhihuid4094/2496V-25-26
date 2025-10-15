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
    //skills auton
      RunpidStraightNTo(100,1000,20,100,400,0,0,1200,0,39);
      delay(100);
      TurnVolpidNTo(100,87, 1, 1000,30);
      Fintake.move(127);
      Mintake.move(127);
      scraper.set_value(true);
      RunpidStraightNTo(50,1200,20,100,1000,0,0,1000,0,39);
      delay(600);
      RunpidStraightNTo(50,200,20,100,1000,0,0,1000,0,39);
      delay(20);
      RunpidStraightNTo(100,-600,20,100,400,0,0,500,0,39);
      delay(50);
      scraper.set_value(false);
      TurnVolpidNTo(100, 175, 1, 800,31);
      delay(100);
      RunpidStraightNTo(100,500,20,100,400,0,0,500,0,39);
      blocker.set_value(true);
      delay(1200);
      RunpidStraightNTo(100,-600,20,100,400,0,0,500,0,39);
      delay(20);
      TurnVolpidNTo(100, -60, 1, 800,31);
      delay(50);
      RunpidStraightNTo(100,800,20,100,400,0,0,1000,45,39);
      delay(20);
      RunpidStraightNTo(100,2500,20,100,400,0,0,3000,0,39);
      delay(40);
      TurnVolpidNTo(100, 60, 1, 800,31);
      delay(20);
      RunpidStraightNTo(100,500,20,100,400,0,0,1000,-70,39);
      scraper.set_value(true);
      delay(20);
      RunpidStraightNTo(100,400,20,100,400,0,0,1000,0,39);

  } 
 
  else if (atn == 1) {
    //blue right
      blocker.set_value(true);
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,500,10,100,400,0,0,4000,-40,39);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,400,5,100,400,0,0,1000,0,39);
      linkage.set_value(true);
      scraper.set_value(true);
      Fintake.move(0);
      Mintake.move(0);
      TurnVolpidNTo(100, 87, 1, 800,34);
      blocker.set_value(false);
      RunpidStraightNTo(50,300,20,100,400,0,0,4000,0,39);
      Fintake.move(127);
      Mintake.move(127);
      delay(1400);
      RunpidStraightNTo(100,-1250,20,100,400,0,0,4000,0,39);
      linkage.set_value(false);
      delay(50);
      TurnVolpidNTo(100,128, 1, 1000,34);
      delay(50);
      blocker.set_value(true);
      RunpidStraightNTo(100,750,20,100,400,0,0,1000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, 178, 1, 1000,19);
      delay(50);
      RunpidStraightNTo(100,600,20,100,400,0,0,1000,0,39);
      blocker.set_value(false);
  //blue elims
      // blocker.set_value(true);
      // Fintake.move(127);
      // Mintake.move(127);
      // RunpidTo(80,1600,10,30,1000,0,0,4000,-50,450,0,2);//1000, -40
      // scraper.set_value(true);
  }

  else if (atn == 2) { 
    //Blue left
      blocker.set_value(true);
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,600,10,100,400,0,0,4000,40,39);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,450,5,100,400,0,0,1000,0,39);
      Fintake.move(0);
      Mintake.move(0);
      TurnVolpidNTo(100, -85, 1, 800,34);
      blocker.set_value(false);
      delay(10);
      RunpidStraightNTo(50,550,20,100,400,0,0,4000,0,39);
      Fintake.move(-127);
      Mintake.move(-127);
      delay(1400);
      RunpidStraightNTo(100,-1550,20,100,400,0,0,4000,0,39);
      scraper.set_value(true);
      Fintake.move(127);
      Mintake.move(127);
      linkage.set_value(false);
      delay(50);
      TurnVolpidNTo(100,-133, 1, 1000,34);
      delay(50);
      blocker.set_value(true);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -180, 1, 800,33);
      delay(50);
      RunpidStraightNTo(100,600,20,100,400,0,0,1000,0,39);
      blocker.set_value(false);
      colorSorter.set_value(true);

  }
    else if (atn == 3) { 
      blocker.set_value(true);
      blocker.set_value(false);
      RunpidStraightNTo(100,1000,20,100,400,0,0,1200,0,39);
      delay(100);
      TurnVolpidNTo(100,87, 1, 1000,30);
      blocker.set_value(true);
      Fintake.move(127);
      Mintake.move(127);
      scraper.set_value(true);
      RunpidStraightNTo(50,1500,20,100,1000,0,0,1000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-600,20,100,400,0,0,500,0,39);
      delay(50);
      scraper.set_value(false);
      TurnVolpidNTo(100, 177, 1, 800,31);
      delay(100);
      RunpidStraightNTo(100,500,20,100,400,0,0,500,0,39);
      delay(1200);
      RunpidStraightNTo(100,-600,20,100,400,0,0,500,0,39);
      delay(20);
      TurnVolpidNTo(100, -60, 1, 800,31);
      delay(50);
      RunpidStraightNTo(100,800,20,100,400,0,0,1000,50,39);
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
