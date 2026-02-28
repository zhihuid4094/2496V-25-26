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
    //97 skills
    descore.set_value(true);
    park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    //arc to pickup balls 
    RunpidStraightNTo(100,200,40,60,400,0,0,2000,0,39);
    RunpidStraightNTo(100,250,30,60,400,0,0,2000,-40,39);
    RunpidStraightNTo(100,380,40,60,400,0,0,2000,0,39);
    lift.set_value(true);
    TurnVolpidNTo(100, -131, 1, 500,36);
    delay(100);
    RunpidStraightNTo(100,-490,40,60,400,0,0,2000,0,39);
    //score middle goal 
    pto.set_value(false);
    delay(1200);
    RunpidStraightNTo(100,1590,30,30,400,0,0,4000,0,39);//1600
     lift.set_value(false);
     pto.set_value(true);
     delay(100);
    TurnVolpidNTo(100,179 , 1, 300,37);
    scraper.set_value(true);
    delay(100);
    RunpidStraightNTo(60,470,20,60,400,0,0,1000,0,39);//
    delay(400);
    RunpidStraightNTo(100,30,100,60,400,0,0,1000,0,39);
    delay(100);
    RunpidStraightNTo(100,-500,100,10,400,0,0,1200,0,39);
    delay(25);
    scraper.set_value(false);
    TurnVolpidNTo(100, -90, 1, 500,37);
    Lintake.move(0);
      Rintake.move(0);
    RunpidStraightNTo(100,600,100,10,400,0,0,1200,0,39);
    delay(50);
    RunpidStraightNTo(100,-20,100,10,400,0,0,200,0,39);
    delay(50);
    TurnVolpidNTo(100, -10, 1, 500,37);
    RunpidStraightNTo(100,2800,100,60,400,0,0,4000,0,39);
    TurnVolpidNTo(100, -40, 1, 500,37);
    RunpidStraightNTo(100,-550,100,30,400,0,0,4000,0,39);
    TurnVolpidNTo(100, 0, 1, 500,37);
    Lintake.move(127);
      Rintake.move(-127);
    RunpidStraightNTo(100,-500,100,30,400,0,0,800,0,39);
    //score other side first loader balls
    pto.set_value(false);
    delay(1500);
    scraper.set_value(true);
    pto.set_value(true);
    TurnVolpidNTo(100, 3, 1, 500,37);
     RunpidStraightNTo(60,1000,20,60,400,0,0,1500,0,39);
    delay(400);
    RunpidStraightNTo(60,100,20,60,400,0,0,1500,0,39);
    delay(300);
    RunpidStraightNTo(80,-1100,80,60,400,0,0,4000,0,39);
    //intake second and scored second loader
    // RunpidStraightNTo(100,5000,100,30,400,0,0,2000,0,39);
    scraper.set_value(false);
    pto.set_value(false);
    delay(900);
    TurnVolpidNTo(100,30, 1, 500,37);
    pto.set_value(true);
    RunpidStraightNTo(100,1080,100,60,400,0,0,4000,0,39);
    TurnVolpidNTo(100,85, 1, 500,37);
      RunpidStraightNTo(100,650,100,60,400,0,0,1000,0,39);
  RunpidStraightNTo(100,-300,100,30,400,0,0,2000,0,39);
  // scraper.set_value(true);
  RunpidStraightNTo(100,2000,100,30,400,0,0,2000,0,39);
  RunpidStraightNTo(100,-200,100,30,400,0,0,500,0,39);
  RunpidStraightNTo(100,3000,100,30,400,0,0,4500,0,39);
  scraper.set_value(false);
  RunpidStraightNTo(100,1500,100,30,400,0,0,2000,0,39);
  TurnVolpidNTo(100,180, 1, 500,37);
  delay(50);
  TurnVolpidNTo(100,0, 1, 500,37);
  delay(50);
  RunpidStraightNTo(100,300,100,30,400,0,0,600,0,39);
  RunpidStraightNTo(100,-780,100,30,400,0,0,2000,0,39);
  delay(50);
  TurnVolpidNTo(100,46, 1, 500,37);
    delay(50);
    RunpidStraightNTo(100,-1100,40,30,400,0,0,2300,0,39);
    lift.set_value(true);
    for(int i = 0; i <= 200; i++){
      Lintake.move(-127);
      Rintake.move(127);
      delay(1);
    }
    Lintake.move(60);
    Rintake.move(-60);
    //score middle goal
    delay(5000);
    RunpidStraightNTo(100,1610,30,30,400,0,0,4000,0,39);
    delay(50);
    scraper.set_value(true);
    lift.set_value(false);
    pto.set_value(true);
    TurnVolpidNTo(100,0, 1, 500,37);
    delay(100);
    //loader number 3
    RunpidStraightNTo(60,470,20,60,400,0,0,1000,0,39);//
    delay(400);
    RunpidStraightNTo(100,30,100,60,400,0,0,1000,0,39);
    delay(100);
    RunpidStraightNTo(100,-500,100,10,400,0,0,1200,0,39);
    delay(25);
    scraper.set_value(false);
    TurnVolpidNTo(100, 90, 1, 500,37);
    //carry balls over
    Lintake.move(0);
    Rintake.move(0);
    RunpidStraightNTo(100,600,100,10,400,0,0,1200,0,39);
    delay(50);
    RunpidStraightNTo(100,-20,100,10,400,0,0,200,0,39);
    delay(50);
    TurnVolpidNTo(100, 170, 1, 500,37);
    RunpidStraightNTo(100,2800,100,60,400,0,0,4000,0,39);
    TurnVolpidNTo(100, 140, 1, 500,37);
    RunpidStraightNTo(100,-550,100,30,400,0,0,4000,0,39);
    TurnVolpidNTo(100, 180, 1, 500,37);
    Lintake.move(127);
    Rintake.move(-127);
    RunpidStraightNTo(100,-500,100,30,400,0,0,800,0,39);
    pto.set_value(false);
    delay(1500);
    scraper.set_value(true);
    pto.set_value(true);
    TurnVolpidNTo(100, -177, 1, 500,37);
    //loader number 4
    RunpidStraightNTo(60,1000,20,60,400,0,0,1500,0,39);
    delay(400);
    RunpidStraightNTo(60,100,20,60,400,0,0,1500,0,39);
    delay(300);
    RunpidStraightNTo(80,-1100,80,60,400,0,0,4000,0,39);
    scraper.set_value(false);
    pto.set_value(false);
    delay(1500);
    TurnVolpidNTo(100,30, 1, 500,37);
    pto.set_value(true);
    RunpidStraightNTo(100,1080,100,60,400,0,0,4000,0,39);
    TurnVolpidNTo(100,85, 1, 500,37);
    RunpidStraightNTo(100,650,100,60,400,0,0,1000,0,39);
    RunpidStraightNTo(100,-300,100,30,400,0,0,2000,0,39);
    RunpidStraightNTo(100,2000,100,30,400,0,0,2000,0,39);
    Lintake.move(127);
    Rintake.move(-127);

  


    // RunpidStraightNTo(100,730,100,60,400,0,0,4000,0,39);
    // TurnVolpidNTo(100, 78, 1, 500,37);
    // RunpidStraightNTo(100,5000,100,60,400,0,0,5000,0,39);

    //75 skills
  //     park.set_value(true);
  //   pto.set_value(true);
  //   Lintake.move(127);
  //   Rintake.move(-127);
  // RunpidStraightNTo(100,950,100,10,400,0,0,2000,0,39);
  // delay(100);
  // TurnVolpidNTo(100, 90, 1, 500,37);
  // delay(25);
  // scraper.set_value(true);
  // delay(100);
  // RunpidStraightNTo(40,580,40,10,800,0,0,1000,0,39);
  // //1st scraper
  // delay(900);
  // RunpidStraightNTo(40,200,40,10,800,0,0,200,0,39);
  // delay(100);
  // RunpidStraightNTo(100,-500,100,10,400,0,0,1200,0,39);
  // delay(25);
  // scraper.set_value(false);
  // TurnVolpidNTo(100, 0, 1, 500,37);
  // Lintake.move(0);
  //   Rintake.move(0);
  // RunpidStraightNTo(100,400,100,10,400,0,0,1200,0,39);
  // delay(50);
  // TurnVolpidNTo(100, -90, 1, 500,37);
  // RunpidStraightNTo(100,2700,100,60,400,0,0,4000,0,39);
  // TurnVolpidNTo(100, -25, 1, 500,37);
  // RunpidStraightNTo(100,-420,100,30,400,0,0,4000,0,39);
  // TurnVolpidNTo(100, -90, 1, 500,37);
  // Lintake.move(127);
  //   Rintake.move(-127);
  // RunpidStraightNTo(100,-500,100,30,400,0,0,800,0,39);
  // pto.set_value(false);
  // //1st scoring
  // delay(1900);
  // TurnVolpidNTo(100, -92, 1, 500,37);
  // scraper.set_value(true);
  // delay(50);
  // pto.set_value(true);
  // RunpidStraightNTo(40,1380,40,30,400,0,0,1700,0,39);
  // //2nd loader
  // delay(800);
  // RunpidStraightNTo(40,100,40,10,800,0,0,200,0,39);
  // delay(100);
  // TurnVolpidNTo(100, -87, 1, 500,37);
  // RunpidStraightNTo(100,-1300,100,30,400,0,0,1500,0,39);
  // scraper.set_value(false);
  // pto.set_value(false);
  // //2nd scoring
  // delay(1900);
  // RunpidStraightNTo(100,350,100,30,400,0,0,4000,0,39);
  // TurnVolpidNTo(100, -180, 1, 500,37);
  // RunpidStraightNTo(100,3280,100,60,400,0,0,4000,0,39);
  // delay(100);
  // TurnVolpidNTo(100, -88, 1, 500,37);
  // scraper.set_value(true);
  // pto.set_value(true);
  // RunpidStraightNTo(40,1300,40,10,400,0,0,1700,0,39);
  // //3rd loader
  // delay(800);
  // RunpidStraightNTo(40,100,40,10,800,0,0,200,0,39);
  // delay(100);
  // scraper.set_value(false);
  // TurnVolpidNTo(100, -92, 1, 500,37);
  // RunpidStraightNTo(100,-500,100,10,400,0,0,1700,0,39);
  // TurnVolpidNTo(100, -180, 1, 500,37);
  // Lintake.move(0);
  // Rintake.move(0);
  // RunpidStraightNTo(40,650,40,10,400,0,0,1300,0,39);
  // RunpidStraightNTo(40,-100,40,10,400,0,0,500,0,39);
  // TurnVolpidNTo(100, 90, 1, 500,37);
  // RunpidStraightNTo(100,2800,100,60,400,0,0,4000,0,39);
  // TurnVolpidNTo(100, 145, 1, 500,37);
  // RunpidStraightNTo(100,-460,100,60,400,0,0,800,0,39);
  // TurnVolpidNTo(100, 90, 1, 500,37);
  // RunpidStraightNTo(100,-500,100,60,400,0,0,800,0,39);
  // Lintake.move(127);
  //   Rintake.move(-127);
  // pto.set_value(false);
  // //3rd scoring
  // delay(1900);
  // scraper.set_value(true);
  // pto.set_value(true);
  // TurnVolpidNTo(100, 86, 1, 500,37);
  // RunpidStraightNTo(40,1390,40,30,400,0,0,1700,0,39);
  // //4th loader
  // delay(800);
  // RunpidStraightNTo(40,100,40,10,800,0,0,200,0,39);
  // delay(100);
  // TurnVolpidNTo(100, 91, 1, 500,37);
  // RunpidStraightNTo(100,-1300,100,60,400,0,0,2000,0,39);
  // pto.set_value(false);
  // //4th scoring
  // delay(1900);
  // scraper.set_value(false);
  // RunpidStraightNTo(100,550,100,60,400,0,0,2000,0,39);
  // TurnVolpidNTo(100, -45, 1, 500,37);
  // pto.set_value(true);
  // RunpidStraightNTo(40,1600,40,30,400,0,0,2000,0,39);
  // delay(300);
  // TurnVolpidNTo(100, 124, 1, 500,37);
  // lift.set_value(true);
  // RunpidStraightNTo(100,-670,100,60,400,0,0,2000,0,39);
  // pto.set_value(false);
  // //mid goal
  // delay(1000);
  // RunpidStraightNTo(100,800,100,60,400,0,0,2000,0,39);
  // TurnVolpidNTo(100, 90, 1, 500,37);
  // //park
  // lift.set_value(false);
  // RunpidStraightNTo(100,400,100,60,400,0,0,2000,0,39);
  // RunpidStraightNTo(100,400,100,60,400,0,0,2000,-120,39);
  // RunpidStraightNTo(100,800,100,60,400,0,0,1000,0,39);
  // RunpidStraightNTo(100,-200,100,30,400,0,0,2000,0,39);
  // scraper.set_value(true);
  // RunpidStraightNTo(100,1500,100,30,400,0,0,2000,0,39);
  // scraper.set_value(false);
  // Lintake.move(127);
  // Rintake.move(-127);










    
  } 
 

  else if (atn == 1) {
    //left mid 3+4
    //field 1&2
    // park.set_value(true);
    // pto.set_value(true);
    // Lintake.move(127);
    // Rintake.move(-127);
    // RunpidStraightNTo(100,400,40,60,400,0,0,2000,0,39);
    // RunpidStraightNTo(100,250,30,60,400,0,0,2000,-40,39);
    // RunpidStraightNTo(100,200,40,60,400,0,0,2000,0,39);
    // lift.set_value(true);
    // TurnVolpidNTo(100, -131, 1, 500,36);
    // RunpidStraightNTo(100,-490,40,60,400,0,0,2000,0,39);
    // pto.set_value(false);
    // delay(1200);
    // pto.set_value(true);
    // lift.set_value(false);
    // RunpidStraightNTo(100,1570,30,30,400,0,0,4000,0,39);//1600
    // TurnVolpidNTo(100,179 , 1, 300,37);
    // scraper.set_value(true);
    // delay(100);
    // RunpidStraightNTo(60,500,20,60,400,0,0,1000,0,39);//
    // delay(100);

    // RunpidStraightNTo(100,150,100,60,400,0,0,800,0,39);//200
    // delay(100);
    // RunpidStraightNTo(100,-1100,40,60,400,0,0,2000,0,39);
    // pto.set_value(false);
    // delay(1100);
    // Lintake.move(0);
    // Rintake.move(0);
    // imu.set_rotation(0);
    // descore.set_value(false);
    // RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    // TurnVolpidNTo(100, 60, 1, 500,36);
    // RunpidStraightNTo(100,-380,40,60,400,0,0,500,0,39);//-250, -30
    // TurnVolpidNTo(100, 0, 1, 500,36);
    // RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
    // TurnVolpidNTo(100, 15, 1, 500,36);

    //3+4 long goal first
    //field 1
     park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    //intake 3 balls
    RunpidStraightNTo(100,400,40,60,400,0,0,2000,0,39);
    RunpidStraightNTo(100,250,30,60,400,0,0,2000,-40,39);
    RunpidStraightNTo(100,200,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 230, 1, 500,36);
    //drive backwards before turn to goal
    RunpidStraightNTo(100,960,40,60,400,0,0,2000,0,39);
    pros::delay(300);
    TurnVolpidNTo(100, -179, 1, 500,36);
    RunpidStraightNTo(100,-750,40,60,400,0,0,1000,0,39);
    pto.set_value(false); //scoring
    delay(800);
    TurnVolpidNTo(100, 183, 1, 500,36);
    scraper.set_value(true);
    RunpidStraightNTo(50,1500,40,40,400,0,0,1500,0,39);
    delay(100);
    pto.set_value(true);
    RunpidStraightNTo(70,-300,40,60,400,0,0,1000,0,39);
    scraper.set_value(false);
    TurnVolpidNTo(100, -134, 1, 500,36);
    //turning to midle goal
    lift.set_value(true);
    RunpidStraightNTo(70,-1660,40,60,400,0,0,1500,0,39);
    for(int i = 0; i <= 20; i++){
      Lintake.move(-127);
      Rintake.move(127);
      delay(1);
    }
    Lintake.move(127);
      Rintake.move(-127);
    pto.set_value(false);
    delay(500); //mid 
    RunpidStraightNTo(70,300,40,60,400,0,0,2000,0,39);
    RunpidStraightNTo(100,-400,40,60,400,0,0,2000,0,39); //dih tap 
    RunpidStraightNTo(70,1095,40,60,400,0,0,2000,0,39); //drive out 
    lift.set_value(false);
    TurnVolpidNTo(100, 183, 1, 500,36); ////descore up and ready
    RunpidStraightNTo(70,-1000,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, -160, 1, 500,36);

    //  descore.set_value(true);
    //  RunpidStraightNTo(100,800,40,60,400,0,0,500,0,39);
    //  TurnVolpidNTo(100, 30, 1, 500,36);
    // RunpidStraightNTo(100,-470,40,60,400,0,0,2000,0,39);
    //field 1 tbt///////////////////////////////////////////////////
    // park.set_value(true);
    // pto.set_value(true);
    // Lintake.move(127);
    // Rintake.move(-127);
    // RunpidStraightNTo(100,300,40,60,400,0,0,2000,0,39);
    // RunpidStraightNTo(100,250,30,60,400,0,0,2000,-40,39);
    // RunpidStraightNTo(100,220,40,60,400,0,0,2000,0,39);
    // lift.set_value(true);
    // TurnVolpidNTo(100, -130, 1, 500,36);
    // RunpidStraightNTo(100,-450,40,60,400,0,0,2000,0,39);
    // pto.set_value(false);
    // delay(500);
    // pto.set_value(true);
    // lift.set_value(false);
    // RunpidStraightNTo(100,1470,30,60,400,0,0,2000,0,39);
    // TurnVolpidNTo(100, 179, 1, 2000,37);
    // scraper.set_value(true);
    // delay(300);
    // RunpidStraightNTo(60,500,20,60,400,0,0,1000,0,39);
    // delay(100);
    // RunpidStraightNTo(100,320,100,60,400,0,0,800,0,39);
    // delay(300);
    // RunpidStraightNTo(100,-1100,40,60,400,0,0,2000,0,39);
    // pto.set_value(false);
    // delay(1500);
    // imu.set_rotation(0);
    // descore.set_value(false);
    // RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    // TurnVolpidNTo(100, 60, 1, 2000,36);
    // RunpidStraightNTo(100,-300,40,60,400,0,0,500,0,39);//-250, -30
    // TurnVolpidNTo(100, 0, 1, 500,36);
    // RunpidStraightNTo(100,-900,40,60,400,0,0,2000,0,39);
  }
  else if (atn == 2) { 
    //right low 3+4
    park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    RunpidStraightNTo(100,580,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 45, 1, 1000,36);
    RunpidStraightNTo(100,370,40,60,400,0,0,2000,0,39);
    RunpidStraightNTo(100,-45,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, -44, 1, 1000,36);
    RunpidStraightNTo(100,530,40,60,400,0,0,2000,0,39);
    Lintake.move(-127);
    Rintake.move(127);
    delay(800);
    pto.set_value(true);
    lift.set_value(false);
    Lintake.move(127);
    Rintake.move(-127);
    RunpidStraightNTo(100,-1820,70,40,400,0,0,2500,0,39);
    delay(50);
    TurnVolpidNTo(100, -179, 1, 1000,37);
    scraper.set_value(true);
    delay(50);
    RunpidStraightNTo(40,500,40,10,400,0,0,700,0,39);
    RunpidStraightNTo(100,200,100,10,400,0,0,300,0,39);
    TurnVolpidNTo(100, 180, 1, 1000,37);
    RunpidStraightNTo(100,-1100,40,10,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1000);
    Lintake.move(0);
    Rintake.move(0);
    imu.set_rotation(0);
    descore.set_value(false);
    RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    TurnVolpidNTo(100, 60, 1, 500,36);
    RunpidStraightNTo(100,-370,40,60,400,0,0,500,0,39);//-250, -30
    TurnVolpidNTo(100, 0, 1, 500,36);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 15, 1, 500,36);
  }
//9 ball blue right //////
  else if (atn == 3) { 
  park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    lift.set_value(true);
    RunpidStraightNTo(60,800,20,60,400,0,0,2000,0,39);
    scraper.set_value(true);
    RunpidStraightNTo(60,200,20,60,400,0,0,2000,0,39);
    RunpidStraightNTo(100,300,30,60,400,0,0,2000,-33,39);
    scraper.set_value(false);
    RunpidStraightNTo(100,480,40,60,400,0,0,2000,0,39);
    delay(200);
    RunpidStraightNTo(80,-600,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, -110, 1, 500,36);
    RunpidStraightNTo(100,1250,30,60,400,0,0,2000,0,39);
    lift.set_value(false);
    TurnVolpidNTo(100, -153, 1, 2000,37);
    scraper.set_value(true);
    delay(300);
    RunpidStraightNTo(60,300,20,60,400,0,0,1000,0,39);
    lift.set_value(false);
    delay(100);
    RunpidStraightNTo(100,300,100,60,400,0,0,800,0,39);
    delay(300);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1500);
    imu.set_rotation(0);
    descore.set_value(true);
    RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    TurnVolpidNTo(100, 60, 1, 2000,36);
    RunpidStraightNTo(100,-300,40,60,400,0,0,500,0,39);//-250, -30
    TurnVolpidNTo(100, 0, 1, 500,36);
    RunpidStraightNTo(100,-900,40,60,400,0,0,2000,0,39);
  } 
//red left///////
  else if(atn == 4) {
  //14 bal 
  park.set_value(true);
  pto.set_value(true);
  Lintake.move(127);
  Rintake.move(-127);
  RunpidStraightNTo(100,970,100,10,400,0,0,2000,0,39);
  delay(50);
  TurnVolpidNTo(100, 88, 1, 500,36);
  delay(25);
  scraper.set_value(true);
  delay(50);
  RunpidStraightNTo(100,460,90,60,400,0,0,1000,0,39);
  delay(300);
  RunpidStraightNTo(100,-1000,100,60,400,0,0,1200,0,39);
  pto.set_value(false);
  RunpidStraightNTo(100,-400,100,60,400,0,0,400,0,39);
  scraper.set_value(false);
  delay(500);
  TurnVolpidNTo(100, -163, 1, 900,30);
  delay(50);
  pto.set_value(true);
  RunpidStraightNTo(100,2100,80,50,400,0,0,3000,-14,39);//1945,-10
   delay(25);
    TurnVolpidNTo(100, 136.5, 1, 500,36);
    delay(25);
    lift.set_value(true);
    RunpidStraightNTo(100,-500,100,60,400,0,0,2000,0,39);
    pto.set_value(false);
    RunpidStraightNTo(100,-160,100,60,400,0,0,2000,0,39);
    delay(500);                                         
    pto.set_value(true);
    lift.set_value(false);
    TurnVolpidNTo(100, 138, 1, 500,36);
    RunpidStraightNTo(100,1500,100,60,400,0,0,2200,0,39);
    delay(50);
    TurnVolpidNTo(100, 88, 1, 300,37);
    RunpidStraightNTo(100,-800,100,30,400,0,0,1000,0,39);
    pto.set_value(false);
    delay(400);
    scraper.set_value(true);
    delay(50);
    pto.set_value(true);
    RunpidStraightNTo(100,1100,90,30,200,0,0,1400,0,39);
    delay(400);
    RunpidStraightNTo(100,-1100,100,30,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1500);

  } 
  else if (atn == 5){
//blue 7 ball left
    // park.set_value(true);
    // pto.set_value(true);
    // Lintake.move(127);
    // Rintake.move(-127);
    // RunpidStraightNTo(100,400,100,60,400,0,0,2000,0,39);
    // RunpidStraightNTo(100,250,100,60,400,0,0,2000,-40,39);
    // RunpidStraightNTo(60,270,60,60,400,0,0,2000,0,39);
    // TurnVolpidNTo(100, -130, 1, 600,37);
    // RunpidStraightNTo(100,950,100,60,400,0,0,1000,0,39);
    // TurnVolpidNTo(100, 179, 1, 800,37);
    // scraper.set_value(true);
    // delay(150);
    // RunpidStraightNTo(60,500,20,60,400,0,0,800,0,39);
    // delay(50);
    // RunpidStraightNTo(100,300,100,60,400,0,0,500,0,39);
    // RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
    // pto.set_value(false);
    // delay(1700);
    // imu.set_rotation(0);
    // descore.set_value(false);
    // Lintake.move(0);
    // Rintake.move(0);
    // RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    // TurnVolpidNTo(100, 60, 1, 500,36);
    // RunpidStraightNTo(100,-380,40,60,400,0,0,500,0,39);//-250, -30
    // TurnVolpidNTo(100, 0, 1, 500,36);
    // RunpidStraightNTo(100,-900,40,60,400,0,0,2000,0,39);
    // TurnVolpidNTo(100, 15, 1, 500,36);

//7 ball right RRRRRRR
     park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    // RunpidStraightNTo(100,500,40,60,400,0,0,2000,0,39);
    // TurnVolpidNTo(100, 45, 1, 300,37);
    RunpidStraightNTo(100,600,60,60,400,0,0,2000,0,39);
    RunpidStraightNTo(60,400,60,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 117, 1, 600,37);
    //long straight distane to goal
    RunpidStraightNTo(100,950,100,40,400,0,0,1000,0,39);
    delay(50);
    TurnVolpidNTo(100,163,1, 1000,37);
    scraper.set_value(true);
    delay(150);
    RunpidStraightNTo(60,520,20,60,400,0,0,800,0,39);
    delay(15);
    RunpidStraightNTo(100,300,100,60,400,0,0,500,0,39);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1700);
    imu.set_rotation(0);
    descore.set_value(false);
    Lintake.move(0);
    Rintake.move(0);
    RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    TurnVolpidNTo(100, 60, 1, 500,36);
    RunpidStraightNTo(100,-370,40,60,400,0,0,500,0,39);//-250, -30
    TurnVolpidNTo(100, 0, 1, 500,36);
    RunpidStraightNTo(100,-900,40,60,400,0,0,2000,0,39);
    delay(50);
    TurnVolpidNTo(100, 30, 1, 500,36);
    
    

  } 
  
  else if (atn == 6){

    //sawp
    park.set_value(true);
    RunpidStraightNTo(100,-1280,30,60,400,0,0,2000,0,39);
    pto.set_value(false);
		Lintake.move(75);
    Rintake.move(-75);
    delay(200);
    Lintake.move(127);
    Rintake.move(-127);
    RunpidStraightNTo(100,250,40,60,400,0,0,3000,-110,39);//350,-75
    pto.set_value(true);
    RunpidStraightNTo(60,400,20,60,400,0,0,2000,0,39);
    delay(300);
    TurnVolpidNTo(100, 160, 1, 2000,37);//165
    RunpidStraightNTo(100,600,40,60,400,0,0,2000,0,39);
    Lintake.move(-90);
    Rintake.move(90);
    scraper.set_value(true);
    delay(635);
    Lintake.move(127);
    Rintake.move(-127);
    scraper.set_value(false);
    RunpidStraightNTo(100,-1800,40,60,400,0,0,2500,0,39);
    delay(200);
    TurnVolpidNTo(100, 26, 1, 2000,37);
    Lintake.move(127);
    Rintake.move(-127);
    scraper.set_value(true);
    delay(350);
    RunpidStraightNTo(100,560,40,60,400,0,0,2000,0,39);
    delay(200);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,800,0,39);
    RunpidStraightNTo(100,-300,40,60,400,0,0,200,0,39);
    pto.set_value(false);
    delay(1000);
    descore.set_value(false);
    RunpidStraightNTo(100,200,40,60,400,0,0,500,0,39);
    TurnVolpidNTo(100,-20, 1, 500,37);
    delay(50);
    RunpidStraightNTo(100,500,40,60,400,0,0,500,0,39);//-250, -30
    TurnVolpidNTo(100, 26, 1, 500,37);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,1000,0,39);
  






  


  

  
 
  } 
  
  else if (atn == 7){
    
  } 
  
  else if (atn == 8){
    //4 ball rush left. LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
    //  park.set_value(true);
    // pto.set_value(true);
    // Lintake.move(127);
    // Rintake.move(-127);
    // RunpidStraightNTo(100,400,40,60,400,0,0,2000,0,39);
    // RunpidStraightNTo(100,250,30,60,400,0,0,2000,-40,39);
    // RunpidStraightNTo(100,210,40,60,400,0,0,2000,0,39);
    // TurnVolpidNTo(100, -133, 1, 500,36);
    // delay(25);
    // RunpidStraightNTo(100,1105,30,60,400,0,0,2000,0,39);//1600
    // delay(50);
    // TurnVolpidNTo(100, 179, 1, 300,37);
    // delay(25);
    // RunpidStraightNTo(100,-1100,40,60,400,0,0,1500,0,39);
    // pto.set_value(false);
    // delay(1000);
    // Lintake.move(0);
    // Rintake.move(0);
    // imu.set_rotation(0);
    // descore.set_value(false);
    // RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    // TurnVolpidNTo(100, 60, 1, 500,36);
    // RunpidStraightNTo(100,-380,40,60,400,0,0,500,0,39);//-250, -30
    // TurnVolpidNTo(100, 0, 1, 500,36);
    // RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
    // scraper.set_value(true);
    //  TurnVolpidNTo(100, 15, 1, 500,36);



     //4 ball right RRRRRRRRRRRRRRRRRRRRRRRRR
     park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    // RunpidStraightNTo(100,500,40,60,400,0,0,2000,0,39);
    // TurnVolpidNTo(100, 45, 1, 300,37);
    RunpidStraightNTo(100,600,100,60,400,0,0,2000,0,39);
    RunpidStraightNTo(40,400,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 115, 1, 600,37);
    //long straight distane to goal
    RunpidStraightNTo(100,970,100,40,400,0,0,1000,0,39);
    delay(50);
    TurnVolpidNTo(100, 163, 1, 300,37);
    delay(25);
    RunpidStraightNTo(100,-900,100,60,400,0,0,1200,0,39);
    pto.set_value(false);
    RunpidStraightNTo(100,-200,100,60,400,0,0,1200,0,39);
    delay(1000);
    Lintake.move(0);
    Rintake.move(0);
    imu.set_rotation(0);
    descore.set_value(false);
    RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    TurnVolpidNTo(100, 60, 1, 500,36);
    RunpidStraightNTo(100,-380,40,60,400,0,0,500,0,39);//-250, -30
    TurnVolpidNTo(100, 0, 1, 500,36);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
    scraper.set_value(true);
     TurnVolpidNTo(100, 30, 1, 500,36);




  }
      //derrick was here
}



//park code
// RunpidStraightNTo(100,400,100,60,400,0,0,2000,0,39);
//     RunpidStraightNTo(100,400,100,60,400,0,0,2000,-120,39);
//     RunpidStraightNTo(100,800,100,60,400,0,0,1000,0,39);
//     RunpidStraightNTo(100,-200,100,30,400,0,0,2000,0,39);
//     scraper.set_value(true);
//     RunpidStraightNTo(100,1200,100,30,400,0,0,2000,0,39);
//     scraper.set_value(false);
//      Lintake.move(127);
//     Rintake.move(-127);
