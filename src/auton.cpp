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
    //TurnVolpidN_Global(int max_speed, double angle, double how_much_error_you_tolerate, int break out time);
    //TurnVolpidNTo(int max_speed, double angle, double how_much_error_you_tolerate, int break out time,int which pid case);
      RunpidStraightNTo(100,990,20,100,400,0,0,1200,0,39);
      delay(100);
      TurnVolpidNTo(100,91, 1, 1000,30);
      Fintake.move(127);
      Mintake.move(127);
      //run into loader and scraper
      scraper.set_value(true);
      delay(400);
      RunpidStraightNTo(50,1150,20,100,1000,0,0,1000,0,39);
      delay(600);
      RunpidStraightNTo(50,200,20,100,1000,0,0,1000,0,39);
      delay(100);
      RunpidStraightNTo(100,-700,20,100,400,0,0,500,0,39);
      delay(200);
      scraper.set_value(false);
      //turn to the goal
      delay(200);
      TurnVolpidNTo(100, -175, 1, 800,31);
      delay(200);
      RunpidStraightNTo(100,650,20,100,400,0,0,800,0,39);
      //run into goal
      blocker.set_value(true);
      delay(1600);
      //drive out
      RunpidStraightNTo(100,-650,20,100,400,0,0,500,0,39);
      delay(90);
      TurnVolpidNTo(100, -60, 1, 800,31);
      delay(150);
      //big straight 
      RunpidStraightNTo(100,550,20,100,400,0,0,1000,0,39);
      delay(400);
      TurnVolpidNTo(100, 58, 1, 800,31);
      delay(200);
      RunpidStraightNTo(100,-1200,20,60,400,0,0,2000,0,39);
      delay(200);
      RunpidStraightNTo(100,3600,20,60,400,0,0,4000,0,39);
      delay(150);
      TurnVolpidNTo(100, 60, 1, 800,31);
      delay(300);
      //turn into loader
      //drive to the second loader
      RunpidStraightNTo(100,485,20,100,400,0,0,1000,0,39);
      delay(90);
      TurnVolpidNTo(100, -56, 1, 800,31);
      delay(90);
      // //second loader
      scraper.set_value(true);
      blocker.set_value(false);
      delay(200);

      // //runs into second loader
      RunpidStraightNTo(60,700,20,60,400,0,0,2000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-400,20,100,400,0,0,1000,0,39);
      delay(50);
      scraper.set_value(false);
      // //turn to the goal
      TurnVolpidNTo(100, 180, 1, 800,31);
      delay(100);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      delay(1200);
      RunpidStraightNTo(100,-200,20,100,400,0,0,500,0,39);
      TurnVolpidNTo(100, 90, 1, 800,31);
      delay(70);
      RunpidStraightNTo(100,-600,20,70,400,0,0,1000,0,39);
      delay(100);
      // //100 ready to drive
      RunpidStraightNTo(100,4100,20,70,400,0,0,5000,0,39);
      delay(100);
      TurnVolpidNTo(100, 90, 1, 800,31);
      scraper.set_value(true);
      blocker.set_value(false);
      delay(50);
      RunpidStraightNTo(100,700,20,70,400,0,0,1000,0,39);
      delay(800);
      RunpidStraightNTo(100,50,20,70,400,0,0,1000,0,39);
      scraper.set_value(false);
      delay(200);
      TurnVolpidNTo(100, -178, 1, 800,31);
      delay(200);
      RunpidStraightNTo(100,700,20,70,400,0,0,1000,0,39);
      blocker.set_value(true);
      delay(1200);
      RunpidStraightNTo(100,-200,20,70,400,0,0,1000,0,39);
      TurnVolpidNTo(100, 90, 1, 800,31);
      RunpidStraightNTo(100,-400,20,70,400,0,0,1000,0,39);
      TurnVolpidNTo(100, 90, 1, 800,31);
      RunpidStraightNTo(100,-4000,20,70,400,0,0,5000,0,39);
      RunpidStraightNTo(100,100,20,70,400,0,0,5000,0,39);
      TurnVolpidNTo(100, -90, 1, 800,31);
      RunpidStraightNTo(100,-500,20,70,400,0,0,5000,0,39);
      




  } 
 
  else if (atn == 1) {
    //red left
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,500,10,100,400,0,0,4000,-40,39);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,400,5,100,400,0,0,1000,0,39);
      linkage.set_value(true);
      scraper.set_value(true);
      Fintake.move(0);
      Mintake.move(0);
      TurnVolpidNTo(100, 88, 1, 800,34);
      blocker.set_value(true);
      RunpidStraightNTo(50,350,20,100,400,0,0,4000,0,39);
      Fintake.move(127);
      Mintake.move(127);
      delay(500);
      blocker.set_value(false);
      RunpidStraightNTo(100,-1200,20,100,400,0,0,4000,0,39);
      linkage.set_value(false);
      delay(50);
      TurnVolpidNTo(100,123, 1, 1000,34);
      delay(50);
      RunpidStraightNTo(100,750,20,100,400,0,0,1000,0,39);
      delay(1000);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -175, 1, 1000,19);
      delay(50);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      delay(200);
      colorSorter.set_value(true);
      delay(1200);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,170,39);
      delay(200);
      descore.set_value(true);
      TurnVolpidNTo(100, -95, 1, 1000,19);
      delay(100);
      RunpidStraightNTo(100,1000,20,100,400,0,0,1000,0,39);
  //blue elims
      // blocker.set_value(true);
      // Fintake.move(127);
      // Mintake.move(127);
      // RunpidTo(80,1600,10,30,1000,0,0,4000,-50,450,0,2);//1000, -40
      // scraper.set_value(true);
  }

  else if (atn == 2) { 
    //Blue right safe
      Fintake.move(127);
      Mintake.move(127);
      // //elims
      // RunpidStraightNTo(100,650,10,100,400,0,0,4000,20,39);
      // RunpidStraightNTo(100,1400,10,100,400,0,0,4000,40,39);
      // scraper.set_value(true);
      // RunpidStraightNTo(100,-200,10,100,400,0,0,4000,0,39);
      // scraper.set_value(false);
      // RunpidStraightNTo(100,-800,10,100,400,0,0,4000,-10,39);
      // delay(200);
      // Fintake.move(-127);
      // Mintake.move(-127);
      

      //safe odd
      RunpidStraightNTo(100,500,10,100,400,0,0,4000,0,39);
      TurnVolpidNTo(100, 50, 1, 800,34);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,500,5,30,400,0,0,1000,0,39);
      RunpidStraightNTo(30,-80,5,30,400,0,0,1000,0,39);
      Fintake.move(0);
      Mintake.move(0);
      TurnVolpidNTo(100, -90, 1, 800,34);
      blocker.set_value(true);
      delay(10);
      RunpidStraightNTo(50,620,20,100,400,0,0,4000,0,39);
      Fintake.move(-60);
      Mintake.move(-127);
      delay(1000);
      blocker.set_value(false);
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,-1730,20,100,400,0,0,4000,0,39);
      scraper.set_value(true);
      linkage.set_value(false);
      delay(50);
      TurnVolpidNTo(100,-133, 1, 1000,34);
      delay(50);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -179, 1, 800,33);
      delay(50);
      RunpidStraightNTo(100,650,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      colorSorter.set_value(true);

  }
    else if (atn == 3) { 
      //solo awp - mid end
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
    //Full awp tuned on blue
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,500,10,100,400,0,0,4000,0,39);
      TurnVolpidNTo(100, 50, 1, 800,34);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,500,5,30,400,0,0,1000,0,39);
      RunpidStraightNTo(30,-80,5,30,400,0,0,1000,0,39);
      Fintake.move(0);
      Mintake.move(0);
      TurnVolpidNTo(100, -90, 1, 800,34);
      blocker.set_value(true);
      RunpidStraightNTo(50,630,20,100,400,0,0,4000,0,39);
      scraper.set_value(true);
      Fintake.move(-60);
      Mintake.move(-127);
      delay(600);
      blocker.set_value(false);
      Fintake.move(127);
      Mintake.move(127);
      scraper.set_value(false);
      RunpidStraightNTo(100,-500,20,100,400,0,0,4000,0,39);
      Fintake.move(127);
      Mintake.move(127);
      TurnVolpidNTo(100, -50, 1, 800,34);
      RunpidStraightNTo(100,1490,20,70,400,0,0,4000,0,39);
      TurnVolpidNTo(100, 138, 1, 800,34);
      linkage.set_value(true);
      scraper.set_value(true);
      RunpidStraightNTo(100,380,20,60,400,0,0,4000,0,39);
      blocker.set_value(true);
      delay(500);
      blocker.set_value(false);
      scraper.set_value(false);
      RunpidStraightNTo(100,-1300,20,70,400,0,0,4000,0,39);
      TurnVolpidNTo(100, 128, 1, 800,34);
      linkage.set_value(false);
      scraper.set_value(true);
      RunpidStraightNTo(100,600,20,70,400,0,0,4000,0,39);
      blocker.set_value(false);
      RunpidStraightNTo(100,-400,20,70,400,0,0,4000,0,39);
      scraper.set_value(false);

      TurnVolpidNTo(100, 178, 1, 800,34);
      RunpidStraightNTo(100,500,20,70,400,0,0,4000,0,39);
      blocker.set_value(false);
  } 
  
  else if (atn == 5){
  //blue right elims with push
      RunpidStraightNTo(100,650,10,100,400,0,0,4000,20,39);
      RunpidStraightNTo(100,1400,10,100,400,0,0,4000,40,39);
      scraper.set_value(true);
      RunpidStraightNTo(100,-200,10,100,400,0,0,4000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -30, 1, 800,34);
      RunpidStraightNTo(100,-400,10,100,400,0,0,4000,0,39);//tune this value for the connection between the angle facing towards middle goal
      TurnVolpidNTo(100, -90, 1, 800,34);
      blocker.set_value(true);
      delay(10);
      RunpidStraightNTo(50,620,20,100,400,0,0,4000,0,39);
      Fintake.move(-60);
      Mintake.move(-127);
      delay(800);
      blocker.set_value(false);
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,-1730,20,100,400,0,0,4000,0,39);
      scraper.set_value(true);
      linkage.set_value(false);
      delay(50);
      TurnVolpidNTo(100,-133, 1, 1000,34);
      delay(50);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -179, 1, 800,33);
      delay(50);
      RunpidStraightNTo(100,650,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      colorSorter.set_value(true);
      delay(1000);
      //push for zone
      RunpidStraightNTo(100,-400,20,100,400,0,0,1000,0,39);
      descore.set_value(true);
      TurnVolpidNTo(100, 60, 1, 800,33);
      RunpidStraightNTo(100,400,20,100,400,0,0,1000,45,39);
      RunpidStraightNTo(100,400,20,100,400,0,0,1000,0,39);
  } 
  
  else if (atn == 6){

  } 
  
  else if (atn == 7){

  } 
  
  else if (atn == 8){

  }
}
