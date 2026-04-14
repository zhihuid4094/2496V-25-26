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
    //6+3 rush

    //version 1
    intake.move(127);
    drivePID(800, 53, 1500, 10, 50, 500, 50, 200, -1, 0);
    scraper.set_value(true);
    driveArcL(23, 200, 1500, 50, 15, 50, 200);
    scraper.set_value(false);
    drivePID(650, 100, 1500, 10, 50, 350, 20, 200, -1, 0);
    turnPID(6, 75, 1000, 1.6, 30, 3); 
    drivePID(-800, 127, 1000, 10, 50, 700, 50, -200, -1, 0);
    lift.set_value(true);
    driveArcL(-135, 80, 1000, 50, 15, 50, 0);
    turnPID(210, 75, 1000, 1.6, 30, 3); 
    drivePIDW(-600, 127, 1000, 460, -1, -1, -1, -1, 15, 50, 1, 599, 65, 0, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 800, 100, 10, 550, 50);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    drivePIDW(1350, 60, 1500, 480, -1, -1, -1, -1, 15, 50, 1, 675, 30, 0, -1, 0);
    Lever.move(0);
    drivePIDW(-125, 127, 5000, 480, -1, -1, -1, -1, 15, 50, 1, 300, 60, -200, -1, 0);
    lift.set_value(false); 
    turnPID(-104, 75, 800, 1.6, 30, 0);
     drivePID(-2300, 127, 2000, 10, 50, 600, 20, 0, -1, 0);
     LF.move(-10);
    LM.move(-10);
    LB.move(-10);
    RF.move(-10);
    RM.move(-10);
    RB.move(-10);
    blocker.set_value(false);
    leverPID(750, 85, 1000, 200, 10, 550, 50);
    drivePID(1150, 127, 1000, 50, 50, 350, 20, 0, -1, 0);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-40);
    lift.set_value(true);
    turnPID(210, 75, 1000, 1.6, 10, 0); 
    Lever.move(0);
    drivePID(-800, 127, 800, 10, 50, 350, 20, -200, -1, 0);
    turnPID(235, 75, 1000, 1.6, 10, 0); 



    //version 2
    // intake.move(127);
    // drivePID(800, 53, 1500, 10, 50, 500, 50, 200, -1, 0);
    // scraper.set_value(true);
    // driveArcL(23, 200, 1500, 50, 15, 50, 200);
    // scraper.set_value(false);
    // drivePID(650, 100, 1500, 10, 50, 350, 20, 200, -1, 0);
    // drivePID(-200, 127, 500, 10, 50, 399, 127, -200, -1, 0);
    // turnPID(25, 75, 700, 1.6, 30, 3); 
    // // 3 to chain, 0 to not chain
    // drivePID(-300, 127, 1000, 10, 50, 700, 50, -200, -1, 0);
    // lift.set_value(true);
    // driveArcL(-135, 140, 1000, 50, 15, 50, 0);
    // turnPID(210, 75, 1000, 1.6, 30, 3); 
    // //3 should be fine for chain
    // drivePIDW(-600, 127, 1000, 460, -1, -1, -1, -1, 15, 50, 1, 599, 65, 0, -1, 0); 
    // LF.move(-80);
    // LM.move(-80);
    // LB.move(-80);
    // RF.move(-80);
    // RM.move(-80);
    // RB.move(-80);
    // blocker.set_value(false);
    // leverPID(750, 85, 800, 100, 10, 550, 50);
    // pros::delay(1);
    // blocker.set_value(true);
    // Lever.move(-65);
    // scraper.set_value(true);
    // drivePIDW(1350, 60, 1900, 480, -1, -1, -1, -1, 15, 50, 1, 675, 30, 0, -1, 0);
    // Lever.move(0);
    // drivePIDW(-125, 127, 2000, 480, -1, -1, -1, -1, 15, 50, 1, 300, 60, -50, -1, 0);
    // driveArcL(-45, 140, 1000, 40, 15, 50, -100);
    // lift.set_value(false); 
    // drivePID(-2100, 127, 1200, 10, 50, 600, 20, 0, -1, 0);
    // LF.move(-10);
    // LM.move(-10);
    // LB.move(-10);
    // RF.move(-10);
    // RM.move(-10);
    // RB.move(-10);
    // blocker.set_value(false);
    // leverPID(750, 85, 1000, 200, 10, 550, 50);
    // drivePID(1150, 127, 1000, 50, 50, 350, 20, 0, -1, 0);
    // pros::delay(100);
    // blocker.set_value(true);
    // Lever.move(-40);
    // lift.set_value(true);
    // turnPID(210, 75, 1000, 4, 10, 0); 
    // Lever.move(0);
    // drivePID(-750, 127, 800, 10, 50, 350, 100, -200, -1, 0);
    // turnPID(235, 75, 1000, 4, 10, 0); 


  }


  else if (atn == 2) { 
   //3+4 right

    intake.move(127);
    drivePID(800, 53, 1500, 10, 50, 500, 50, 200, -1, 0);
    scraper.set_value(true);
    driveArcR(22, 200, 1500, 50, 15, 50, 200);
    scraper.set_value(false);
    drivePID(660, 100, 1500, 10, 50, 350, 20, 200, -1, 0);
    drivePID(-200, 127, 500, 10, 50, 399, 127, -200, -1, 0);
    turnPID(-25, 75, 700, 1.6, 30, -3); 
    //movement above -3 should make it actaully chain, see if it helps make it faster ig
    drivePID(-300, 127, 1000, 10, 50, 700, 50, -200, -1, 0);
    lift.set_value(true);
    driveArcR(-135, 140, 1000, 50, 15, 50, 0);
    turnPID(150, 75, 1000, 1.6, 30, 0);  
    drivePIDW(-600, 127, 1000, 435, -1, -1, -1, -1, 15, 50, 0, 599, 65, 0, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 800, 100, 10, 550, 50);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    drivePIDW(1350, 60, 1900, 455, -1, -1, -1, -1, 15, 50, 0, 675, 30, 0, -1, 0);
    Lever.move(0);
    drivePIDW(-100, 127, 500, 455, -1, -1, -1, -1, 15, 50, 0, 300, 60, -200, -1, 0); 
    scraper.set_value(false);
    turnPID(284, 75, 800, 1.6, 30, 0);
    drivePID(2100, 127, 1700, 40, 0, 600, 20, 0, -1, 0);
    intake.move(-127);
    pros::delay(500);
    drivePID(-1200, 127, 1000, 50, 50, 350, 20, 0, -1, 0);
    turnPID(-30, 75, 1000, 1.6, 10, 0); 
    Lever.move(0);
    drivePID(750, 127, 800, 10, 50, 350, 20, 200, -1, 0);
    turnPID(-50, 75, 1000, 1.6, 10, 0); 

    





    
  } 

  else if (atn == 3 ){
   //7 left
    
  } 


  else if(atn == 4) {
    //7 right
    intake.move(127);
    lift.set_value(true);
    descore.set_value(true);
    drivePID(650, 127, 1500, 10, 50, 200, 80, 200, -1, 0);
    scraper.set_value(true);
    turnPID(90, 75, 1000, 1.6, 30, -3); 
    scraper.set_value(false);
    drivePID(800, 200, 1500, 10, 50, 350, 100, 200, -1, 0);
    turnPID(143, 75, 400, 1.6, 30, 0); 
    //4 ball variation
    drivePIDW(-600, 127, 500, 440, -1, -1, -1, -1, 15, 50, 0, 599, 65, 0, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 1000, 100, 10, 550, 50);
    scraper.set_value(true);
    Lever.move(-65);
    drivePIDW(1000, 100, 1000, 40, -1, -1, -1, -1, 15, 50, 0, 599, 40, 0, -1, 0); 
    delay(200);
    drivePIDW(-600, 127, 500, 440, -1, -1, -1, -1, 15, 50, 0, 599, 65, 0, -1, 0);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 600, 100, 10, 550, 50);
    // turnPID(-130, 75, 300, 1.6, 30, -3); 
    // drivePID(130, 100, 500, 10, 50, 350, 100, 200, -1, 0);
    // turnPID(-30, 75, 500, 1.6, 30, 3); 
    // descore.set_value(false);
    // drivePID(1300, 127, 800, 10, 50, 350, 127, -200, -1, 0);
    // turnPID(-50, 75, 1000, 1.6, 10, 0); 
    // blocker.set_value(true);
    // Lever.move(-65);
    // pros::delay(1000);
    // Lever.move(0);
  } 

  else if (atn == 5){
    //4 Fast right
    intake.move(127);
    lift.set_value(true);
    descore.set_value(true);
    drivePID(650, 127, 1500, 10, 50, 200, 80, 200, -1, 0);
    scraper.set_value(true);
    turnPID(90, 75, 1000, 1.6, 30, -3); 
    scraper.set_value(false);
    drivePID(800, 200, 1500, 10, 50, 350, 100, 200, -1, 0);
    turnPID(143, 75, 400, 1.6, 30, 0); 
    //4 ball variation
    drivePIDW(-600, 127, 500, 440, -1, -1, -1, -1, 15, 50, 0, 599, 65, 0, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 600, 100, 10, 550, 50);
    turnPID(-130, 75, 300, 1.6, 30, -3); 
    drivePID(130, 100, 500, 10, 50, 350, 100, 200, -1, 0);
    turnPID(-30, 75, 500, 1.6, 30, 3); 
    descore.set_value(false);
    drivePID(1300, 127, 800, 10, 50, 350, 127, -200, -1, 0);
    turnPID(-50, 75, 1000, 1.6, 10, 0); 
    blocker.set_value(true);
    Lever.move(-65);
    pros::delay(1000);
    Lever.move(0);
  
  }
  else if (atn == 6){
    //4 Fast Right


    intake.move(127);
    lift.set_value(true);
    descore.set_value(true);
    drivePID(600, 127, 1500, 10, 50, 200, 80, 200, -1, 0);
    scraper.set_value(true);
    turnPID(-90, 75, 1000, 1.6, 30, -3); 
    scraper.set_value(false);
    drivePID(800, 200, 1500, 10, 50, 350, 100, 200, -1, 0);
    turnPID(210, 75, 1000, 1.6, 30, 0); 
    //4 ball variation
    drivePIDW(-600, 127, 500, 460, -1, -1, -1, -1, 15, 50, 1, 599, 65, 0, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 600, 100, 10, 550, 50);
    turnPID(170, 75, 300, 1.6, 30, -3); 
    drivePID(230, 100, 500, 10, 50, 350, 100, 200, -1, 0);
    turnPID(208, 75, 500, 1.6, 30, 3); 
    descore.set_value(false);
    drivePID(-750, 127, 800, 10, 50, 350, 127, -200, -1, 0);
    turnPID(235, 75, 1000, 1.6, 10, 0); 
    blocker.set_value(true);
    Lever.move(-65);
    pros::delay(1000);
    Lever.move(0);
    //7 ball variation
    // scraper.set_value(true);
    // drivePIDW(1150, 127, 1400, 480, -1, -1, -1, -1, 15, 50, 1, 300, 65, 0, -1, 0); 
    // drivePIDW(-1200, 127, 1500, 460, -1, -1, -1, -1, 15, 50, 1, 599, 65, 0, -1, 0); 
    // scraper.set_value(false);
    // LF.move(-80);
    // LM.move(-80);
    // LB.move(-80);
    // RF.move(-80);
    // RM.move(-80);
    // RB.move(-80);
    // blocker.set_value(false);
    // leverPID(750, 85, 600, 100, 10, 550, 50);
    // turnPID(170, 75, 300, 1.6, 30, 3); 
    // drivePID(230, 100, 500, 10, 50, 350, 100, 200, -1, 0);
    // turnPID(210, 75, 500, 1.6, 30, 3); 
    // descore.set_value(false);
    // drivePID(-800, 127, 800, 10, 50, 350, 20, -200, -1, 0);
    // turnPID(235, 75, 1000, 4, 10, 0); 
    // blocker.set_value(true);
    // Lever.move(-65);
    // pros::delay(1000);
    // Lever.move(0);



  } 
  

  else if (atn == 7){
    //sawp

    // updateddddd 
    intake.move(127);
    drivePIDW(140, 127, 500, 457, -1, -1, -1, -1, 15, 50, 0, 260, 65, 50, -1, 0); 
    drivePIDW(-1650, 127, 1000, 457, 800, -1, -1, -1, 15, 50, 0, 800, 65, 0, -1, 0); 
    turnPID(-90, 75, 700, 1.6, 30, -2); 
    scraper.set_value(true);
    lift.set_value(true);
    drivePIDW(800, 40, 1000, 472, -1, -1, -1, -1, 15, 50, 0, 400, 60, 0, -1, 0);
    pros::delay(200);
    drivePIDW(-1200, 127, 800, 435, -1, -1, -1, -1, 15, 50, 0, 600, 60, 0, -1, 0);
    scraper.set_value(false);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 800, 100, 10, 550, 50);
    LF.move(30);
    LM.move(30);
    LB.move(30);
    RF.move(-100);
    RM.move(-100);
    RB.move(-100);
    pros::delay(200);
    blocker.set_value(true);
    Lever.move(-60);
    turnPID(10, 75, 700, 1, 30, 0); 
    Lever.move(0);
    driveArcL(10, 1500, 1000, 50, 15, 50, 50);
    drivePIDW(1600, 127, 1500, 1069, 1600, -1, -1, -1, 15, 50, 0, 1300, 60, 200, -1, 0);
    scraper.set_value(true);
    turnPID(-40, 75, 700, 1.6, 30, -3);
    scraper.set_value(false);
    drivePID(900, 127, 1000, 15, 50, 500, 50, 0, -1, 0);
    turnPID(-90, 75, 700, 1.6, 30, -2); 
    drivePIDW(-600, 127, 1000, 460, -1, -1, -1, -1, 15, 50, 1, 599, 65, 0, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 800, 100, 10, 550, 50);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    drivePIDW(1350, 60, 1500, 480, -1, -1, -1, -1, 15, 50, 1, 675, 30, 0, -1, 0);
    Lever.move(0);
    drivePIDW(-180, 60, 500, 480, -1, -1, -1, -1, 15, 50, 1, 100, 50, -50, -1, 0);
    turnPID(-45, 75, 700, 1.6, 30, 2); 
    scraper.set_value(false);
    lift.set_value(false); 
    drivePID(-2300, 127, 1200, 10, 50, 600, 20, 0, -1, 0);
    LF.move(-10);
    LM.move(-10);
    LB.move(-10);
    RF.move(-10);
    RM.move(-10);
    RB.move(-10);
    blocker.set_value(false);
    leverPID(750, 127, 600, 200, 10, 550, 50);
    // Lever.move(-40);
    // drivePID(1150, 127, 1000, 50, 50, 350, 20, 0, -1, 0);
    // blocker.set_value(true);
    // lift.set_value(true);
    // Lever.move(0);
    // turnPID(-90, 75, 1000, 2, 10, 0); 
    // drivePID(-750, 127, 800, 10, 50, 350, 100, -200, -1, 0);
    // turnPID(-70, 75, 1000, 2, 10, 0); 



    //att version
    // intake.move(127);
    // drivePID(-500, 127, 600, 10, 50, 500, 50, 200, -1, 0);
    // drivePID(1360, 127, 2000, 10, 50, 500, 50, 200, -1, 0);
    // delay(200);
    // turnPID(84, 127, 1000, 1.0, 30, 3); 
    // scraper.set_value(true);
    // lift.set_value(true);
    // drivePIDW(500, 30, 1900, 480, -1, -1, -1, -1, 15, 50, 1, 675, 30, 0, -1, 0);
    // delay(100);
    // drivePIDW(-1200, 60, 1200, 435, -1, -1, -1, -1, 15, 50, 0, 675, 30, 0, -1, 0);
    // scraper.set_value(false);
    // LF.move(-80);
    // LM.move(-80);
    // LB.move(-80);
    // RF.move(-80);
    // RM.move(-80);
    // RB.move(-80);
    // blocker.set_value(false);
    // LF.move(0);
    // LM.move(0);
    // LB.move(0);
    // RF.move(0);
    // RM.move(0);
    // RB.move(0);
    // leverPID(750, 85, 600, 100, 10, 550, 50);
    // Lever.move(-65);
    // turnPID(-160, 50, 1000, 1.0, 30, 3); 
    // driveArcL(56, 2000, 2500, 50, 15, 50, 0);
    // lift.set_value(false);
    // drivePID(-1000, 60, 1500, 10, 50, 500, 50, 200, -1, 0);
    // LF.move(-10);
    // LM.move(-10);
    // LB.move(-10);
    // RF.move(-10);
    // RM.move(-10);
    // RB.move(-10);
    // blocker.set_value(false);
    // leverPID(750, 85, 1000, 200, 10, 550, 50);
    // drivePID(1000, 60, 1500, 10, 50, 500, 50, 200, -1, 0);
    // driveArcR(20, 300, 1000, 100, 15, 50, 0);
    // turnPID(-130, 127, 1000, 1.0, 30, 3); 
    // driveArcL(30,25, 200, 50, 15, 50, 200);
    
    // drivePIDW(1500, 60, 1200, 1130, -1, -1, -1, -1, 15, 50, 0, 675, 30, 0, -1, 0);
    // drivePID(1000, 60, 1500, 10, 50, 500, 50, 200, -1, 0);



  } 

  else if (atn == 8){
    //misc

  }
}

