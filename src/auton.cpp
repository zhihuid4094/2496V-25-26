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

  if (atn == 0) {//sawp mid
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
 //left mid 3+4

  else if (atn == 1) {
    //field 3
    park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    RunpidStraightNTo(100,400,40,60,400,0,0,2000,0,39);
    RunpidStraightNTo(100,250,30,60,400,0,0,2000,-40,39);
    RunpidStraightNTo(100,220,40,60,400,0,0,2000,0,39);
    lift.set_value(true);
    TurnVolpidNTo(100, -133, 1, 500,36);
    RunpidStraightNTo(100,-470,40,60,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1000);
    pto.set_value(true);
    lift.set_value(false);
    RunpidStraightNTo(100,1600,30,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 179, 1, 300,37);
    scraper.set_value(true);
    delay(100);
    RunpidStraightNTo(60,500,20,60,400,0,0,1000,0,39);
    delay(100);
    RunpidStraightNTo(100,200,100,60,400,0,0,800,0,39);
    delay(100);
    RunpidStraightNTo(100,-1100,40,60,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(800);
    Lintake.move(0);
    Rintake.move(0);
    imu.set_rotation(0);
    descore.set_value(false);
    RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    TurnVolpidNTo(100, 60, 1, 500,36);
    RunpidStraightNTo(100,-340,40,60,400,0,0,500,0,39);//-250, -30
    TurnVolpidNTo(100, 0, 1, 500,36);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);
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
//blue 7 ball long///////
  else if (atn == 2) { 
    park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    RunpidStraightNTo(100,400,40,60,400,0,0,2000,0,39);
    RunpidStraightNTo(100,250,30,60,400,0,0,2000,-40,39);
    RunpidStraightNTo(100,220,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, -130, 1, 2000,37);
    RunpidStraightNTo(100,800,30,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 179, 1, 2000,37);
    scraper.set_value(true);
    delay(300);
    RunpidStraightNTo(60,500,20,60,400,0,0,1000,0,39);
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
  //14 bal field 2
  park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
  RunpidStraightNTo(100,1000,100,10,400,0,0,2000,0,39);
  delay(50);
  TurnVolpidNTo(100, 91, 1, 500,36);
  delay(25);
  scraper.set_value(true);
  delay(100);
  RunpidStraightNTo(100,520,100,60,400,0,0,1000,0,39);
  delay(150);
  RunpidStraightNTo(100,-1000,100,60,400,0,0,1200,0,39);
  pto.set_value(false);
  RunpidStraightNTo(100,-300,100,60,400,0,0,400,0,39);
  scraper.set_value(false);
  delay(50);
  RunpidStraightNTo(100,400,100,10,400,0,0,1500,0,39);
  delay(25);
  TurnVolpidNTo(100, -145, 1, 700,36);
  delay(25);
  RunpidStraightNTo(80,960,80,10,400,0,0,2000,0,39);
  pto.set_value(true);
  TurnVolpidNTo(100, 179, 1, 500,36);
  delay(100);
  RunpidStraightNTo(100,1480,80,50,400,0,0,2500,0,39);
   delay(25);
    TurnVolpidNTo(100, 138, 1, 500,36);
    delay(25);
    RunpidStraightNTo(100,-570,100,60,400,0,0,2000,0,39);
    lift.set_value(true);
    pto.set_value(false);
    delay(450);                                         
    pto.set_value(true);
    lift.set_value(false);
    TurnVolpidNTo(100, 138, 1, 500,36);
    RunpidStraightNTo(100,1600,100,60,400,0,0,2000,0,39);
    delay(50);
    TurnVolpidNTo(100, 92, 1, 300,37);
    scraper.set_value(true);
    delay(100);
    RunpidStraightNTo(100,610,100,30,400,0,0,1000,0,39);
    delay(300);
    RunpidStraightNTo(100,-1100,100,30,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1500);

    //14 ball field 1
  //   park.set_value(true);
  //   pto.set_value(true);
  //   Lintake.move(127);
  //   Rintake.move(-127);
  // RunpidStraightNTo(100,1000,100,10,400,0,0,2000,0,39);
  // delay(50);
  // TurnVolpidNTo(100, 89, 1, 500,36);
  // delay(25);
  // scraper.set_value(true);
  // delay(100);
  // RunpidStraightNTo(100,480,100,60,400,0,0,1000,0,39);
  // delay(150);
  // RunpidStraightNTo(100,-1000,100,60,400,0,0,1500,0,39);
  // pto.set_value(false);
  // RunpidStraightNTo(100,-300,100,60,400,0,0,1500,0,39);
  // scraper.set_value(false);
  // delay(200);
  // RunpidStraightNTo(100,400,100,10,400,0,0,1500,0,39);
  // delay(25);
  // TurnVolpidNTo(100, -150, 1, 700,36);
  // delay(25);
  // RunpidStraightNTo(80,920,80,10,400,0,0,2000,0,39);
  // pto.set_value(true);
  // TurnVolpidNTo(100, 179, 1, 500,36);
  // delay(100);
  // RunpidStraightNTo(100,1580,80,50,400,0,0,2500,0,39);
  // lift.set_value(true);
  //   TurnVolpidNTo(100, 138, 1, 500,36);
  //   delay(50);
  //   RunpidStraightNTo(100,-570,100,60,400,0,0,2000,0,39);
  //   pto.set_value(false);
  //   delay(300);                                         
  //   pto.set_value(true);
  //   lift.set_value(false);
  //   TurnVolpidNTo(100, 138, 1, 500,36);
  //   RunpidStraightNTo(100,1470,100,60,400,0,0,2000,0,39);
  //   delay(50);
  //   TurnVolpidNTo(100, 92, 1, 300,37);
  //   scraper.set_value(true);
  //   delay(100);
  //   RunpidStraightNTo(100,610,100,30,400,0,0,1000,0,39);
  //   delay(300);
  //   RunpidStraightNTo(100,-1100,100,30,400,0,0,2000,0,39);
  //   pto.set_value(false);
  //   delay(1500);


  } 
  else if (atn == 5){
    //right low 3+4
    park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
    RunpidStraightNTo(100,470,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, 45, 1, 1000,36);
    RunpidStraightNTo(100,350,40,60,400,0,0,2000,0,39);
    TurnVolpidNTo(100, -45, 1, 1000,36);
    RunpidStraightNTo(100,510,40,60,400,0,0,2000,0,39);
    Lintake.move(-127);
    Rintake.move(127);
    delay(500);
    pto.set_value(true);
    lift.set_value(false);
    RunpidStraightNTo(100,-1560,60,40,400,0,0,2000,0,39);
    delay(100);
    Lintake.move(127);
    Rintake.move(-127);
    TurnVolpidNTo(100, -179, 1, 1000,37);
    scraper.set_value(true);
    delay(100);
    RunpidStraightNTo(60,500,20,10,400,0,0,1000,0,39);
    delay(50);
    RunpidStraightNTo(100,200,100,10,400,0,0,800,0,39);
    delay(50);
    RunpidStraightNTo(100,-1100,40,10,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1000);
    Lintake.move(0);
    Rintake.move(0);
    imu.set_rotation(0);
    descore.set_value(false);
    RunpidStraightNTo(100,400,40,60,400,0,0,500,0,39);
    TurnVolpidNTo(100, 60, 1, 500,36);
    RunpidStraightNTo(100,-320,40,60,400,0,0,500,0,39);//-250, -30
    TurnVolpidNTo(100, 0, 1, 500,36);
    RunpidStraightNTo(100,-1000,40,60,400,0,0,2000,0,39);

  } 
  
  else if (atn == 6){
    //skills
      park.set_value(true);
    pto.set_value(true);
    Lintake.move(127);
    Rintake.move(-127);
  RunpidStraightNTo(100,1000,100,10,400,0,0,2000,0,39);
  delay(50);
  TurnVolpidNTo(100, 91, 1, 500,36);
  delay(25);
  scraper.set_value(true);
  delay(100);
  RunpidStraightNTo(100,520,100,60,400,0,0,1000,0,39);
  delay(150);
  RunpidStraightNTo(100,-1000,100,60,400,0,0,1200,0,39);
  pto.set_value(false);
  RunpidStraightNTo(100,-300,100,60,400,0,0,400,0,39);
  scraper.set_value(false);
  delay(50);
  RunpidStraightNTo(100,400,100,10,400,0,0,1500,0,39);
  delay(25);
  TurnVolpidNTo(100, 180, 1, 700,36);
  RunpidStraightNTo(100,2500,100,10,400,0,0,1500,0,39);
    TurnVolpidNTo(100, 92, 1, 300,37);
    scraper.set_value(true);
    delay(100);
    RunpidStraightNTo(100,610,100,30,400,0,0,1000,0,39);
    delay(300);
    RunpidStraightNTo(100,-1100,100,30,400,0,0,2000,0,39);
    pto.set_value(false);
    delay(1500);
    RunpidStraightNTo(100,400,100,30,400,0,0,1000,0,39);
  } 
  
  else if (atn == 7){
 
  } 
  
  else if (atn == 8){
  
  }
      //derrick was here
}
