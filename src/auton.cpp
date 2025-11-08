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
    //test
      // RunpidStraightNTo(100,-500,20,100,400,0,0,1200,0,39);
      // delay(100);
      // TurnVolpidNTo(100,90, 1, 1000,30);
      // RunpidStraightNTo(100,200,20,100,400,0,0,1200,0,39);

      RunpidStraightNTo(100,1000,10,100,400,0,0,1200,0,39);
      delay(200);
      TurnVolpidNTo(100,93, 1, 1000,30);
      Fintake.move(127);
      Mintake.move(127);
      //run into 1st loader and scraper
      scraper.set_value(true);
      delay(400);
      RunpidStraightNTo(50,600,20,100,1000,0,0,1000,0,39);
      delay(600);
      RunpidStraightNTo(50,-10,20,100,1000,0,0,1000,0,39);
      delay(400);
      RunpidStraightNTo(100,-700,20,100,400,0,0,500,0,39);
      delay(200);
      scraper.set_value(false);
      delay(200);
      TurnVolpidNTo(100, -84, 1, 800,30); 
      //turn to the 1st goal
      delay(200);
      RunpidStraightNTo(100,620,20,100,400,0,0,800,0,39);
      RunpidStraightNTo(100,-10,20,100,400,0,0,800,0,39);
      blocker.set_value(true);
      //score 1st goal
      delay(1600);
      RunpidStraightNTo(100,-800,20,100,400,0,0,500,0,39);
      blocker.set_value(false);
      delay(400);
      TurnVolpidNTo(100, 0, 1, 800,30);
      delay(150);
      Fintake.move(-127);
      Mintake.move(-127);
      //1st long horizontal
      RunpidStraightNToHC(100,-3110,20,100,400,0,0,4000,0,40);
      Fintake.move(127);
      Mintake.move(127);
      delay(200);
      TurnVolpidNTo(100, 90, 1, 800,30);
      //scrape 2nd loader
      scraper.set_value(true);
      RunpidStraightNTo(100,720,20,100,400,0,0,1200,0,39);
      RunpidStraightNTo(100,-20,20,100,400,0,0,800,0,39);
      delay(1200);
      RunpidStraightNTo(100,-700,20,100,400,0,0,500,0,39);
      delay(200);
      scraper.set_value(false);
      delay(200);
      TurnVolpidNTo(100, -88, 1, 800,30);
      delay(200);
      RunpidStraightNTo(100,600,20,100,400,0,0,800,0,39);
      //score 2nd goal
      blocker.set_value(true);
      delay(1600);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      delay(400);
      TurnVolpidNTo(100, 30, 1, 800,30);
      delay(60);
      RunpidStraightNTo(100,-550,20,100,400,0,0,1000,0,39);
      delay(400);
      TurnVolpidNTo(100, 93, 1, 800,30);
      //2nd long vertical
      RunpidStraightNTo(100,-2500,20,100,400,0,0,4000,0,40);
      delay(400);
      TurnVolpidNTo(100, -30, 1, 800,30);
      delay(200);
      RunpidStraightNTo(70,480,20,70,400,0,0,2000,0,39);
      delay(200);
      TurnVolpidNTo(100, -90, 1, 800,30);
      blocker.set_value(false);
      //3rd loader
      scraper.set_value(true);
      RunpidStraightNTo(100,555,20,100,400,0,0,1200,0,39);
      delay(500);
      RunpidStraightNTo(100,100,20,100,400,0,0,1200,0,39);
      delay(200);
      RunpidStraightNTo(100,-1200,20,100,400,0,0,500,0,39);
      delay(200);
      scraper.set_value(false);
      delay(200);
      TurnVolpidNTo(100, -179, 1, 800,30);
      delay(200);
      //3rd long horizontal
      RunpidStraightNToHC(100,-3135,10,100,400,0,0,4000,0,40);
      Fintake.move(127);
      Mintake.move(127);
      delay(200);
      TurnVolpidNTo(100, 90, 1, 800,30);
      RunpidStraightNTo(100,480,20,100,400,0,0,1200,0,39);
      //score 3rd goal (opp side)
      blocker.set_value(true);
      delay(1200);
      RunpidStraightNTo(100,-400,20,100,400,0,0,1200,0,39);
      delay(200);
      TurnVolpidNTo(100, -93, 1, 800,30);
      blocker.set_value(false);
      //scrape 4th goal
      scraper.set_value(true);
      RunpidStraightNTo(100,700,20,100,400,0,0,800,0,39);
      delay(600);
      RunpidStraightNTo(100,100,20,100,400,0,0,1200,0,39);
      delay(200);
      RunpidStraightNTo(100,-800,20,100,400,0,0,500,0,39);
      blocker.set_value(false);
      scraper.set_value(false);
      delay(400);
      TurnVolpidNTo(100, 180, 1, 800,30);
      delay(200);
      //4th long horizontal
      RunpidStraightNTo(100,4150,20,100,400,0,0,4000,0,39);
      delay(200);
      RunpidStraightNTo(70,-520,20,70,400,0,0,4000,0,39);
      delay(200);
      TurnVolpidNTo(100, 90, 1, 800,30);
      delay(200);
      RunpidStraightNTo(100,520,20,100,400,0,0,700,0,39);
      RunpidStraightNTo(100,-10,20,100,400,0,0,1200,0,39);
      //score 4th goal
      blocker.set_value(true);
      delay(1600);
      RunpidStraightNTo(100,-400,20,100,400,0,0,1200,0,39);
      TurnVolpidNTo(100, 30, 1, 800,30);
      RunpidStraightNTo(100,400,20,100,400,0,0,1200,0,39);
      TurnVolpidNTo(100, 90, 1, 800,30);
      //attempt to park
      RunpidStraightNTo(100,2900,20,100,400,0,0,4000,0,39);
      RunpidStraightNTo(100,600,20,100,400,0,0,2000,-90,39);
      RunpidStraightNTo(100,100,20,100,400,0,0,4000,0,39);
      scraper.set_value(true);
      RunpidStraightNTo(100,1000,20,100,400,0,0,2000,0,39);
      scraper.set_value(false);







      //old
      // delay(200);
      // RunpidStraightNTo(100,-1200,20,60,400,0,0,2000,0,39);
      // delay(200);
      // RunpidStraightNTo(100,3600,20,60,400,0,0,4000,0,39);
      // delay(150);
      // TurnVolpidNTo(100, 60, 1, 800,31);
      // delay(300);
      // //turn into loader
      // //drive to the second loader
      // RunpidStraightNTo(100,500,20,100,400,0,0,1000,0,39);
      // delay(90);
      // TurnVolpidNTo(100, -56, 1, 800,31);
      // delay(90);
      // //second loader
      // scraper.set_value(true);
      // blocker.set_value(false);
      // delay(200);

      // // //runs into second loader
      // RunpidStraightNTo(60,700,20,60,400,0,0,2000,0,39);
      // delay(1200);
      // RunpidStraightNTo(100,-400,20,100,400,0,0,1000,0,39);
      // delay(50);
      // scraper.set_value(false);
      // // //turn to the goal
      // TurnVolpidNTo(100, 178, 1, 800,31);
      // delay(100);
      // RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      // blocker.set_value(true);
      // delay(1200);
      // RunpidStraightNTo(100,-200,20,100,400,0,0,500,0,39);
      // TurnVolpidNTo(100, 90, 1, 800,31);
      // delay(70);
      // RunpidStraightNTo(100,-650,20,70,400,0,0,1000,0,39);
      // delay(100);
      // // //100 ready to drive
      // RunpidStraightNTo(100,4100,20,70,400,0,0,5000,0,39);
      // delay(100);
      // TurnVolpidNTo(100, 90, 1, 800,31);
      // scraper.set_value(true);
      // blocker.set_value(false);
      // delay(50);
      // RunpidStraightNTo(100,700,20,70,400,0,0,1000,0,39);
      // delay(800);
      // RunpidStraightNTo(100,50,20,70,400,0,0,1000,0,39);
      // scraper.set_value(false);
      // delay(200);
      // TurnVolpidNTo(100, -178, 1, 800,31);
      // delay(200);
      // RunpidStraightNTo(100,700,20,70,400,0,0,1000,0,39);
      // blocker.set_value(true);
      // delay(1200);
      // RunpidStraightNTo(100,-200,20,70,400,0,0,1000,0,39);
      // TurnVolpidNTo(100, -90, 1, 800,31);
      // RunpidStraightNTo(100,400,20,70,400,0,0,1000,0,39);
      // TurnVolpidNTo(100, 90, 1, 800,31);
      // RunpidStraightNTo(100,5000,20,70,400,0,0,5000,0,39);
      // RunpidStraightNTo(100,200,20,70,400,0,0,5000,0,39);
      // TurnVolpidNTo(100, -90, 1, 800,31);
      // RunpidStraightNTo(100,500,20,70,400,0,0,5000,0,39);
      // scraper.set_value(true);
      // RunpidStraightNTo(100,700,20,70,400,0,0,5000,0,39);

      




  } 
 
  else if (atn == 1) {
    //red left
      Fintake.move(127);
      Mintake.move(127);
      //derrick was here
      RunpidStraightNTo(100,550,10,100,400,0,0,4000,-40,39);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,350,5,75,400,0,0,1000,0,39);
      RunpidStraightNTo(30,-120,5,50,400,0,0,1000,0,39);
      linkage.set_value(true);
      Fintake.move(0);
      Mintake.move(0);
      //pid constants edited onb the next line (ur welcome- derrick)
      TurnVolpidNTo(100, 47, 1, 800,34);
      scraper.set_value(true);
      blocker.set_value(true);
      RunpidStraightNTo(50,320,20,100,400,0,0,4000,0,39);
      //middle goal scoring
      Fintake.move(127);
      Mintake.move(127);
      delay(420);
      blocker.set_value(false);
      delay(200);
      scraper.set_value(false);
      RunpidStraightNTo(100,-1345,20,100,400,0,0,4000,0,39);
      linkage.set_value(false);
      delay(100);
      TurnVolpidNTo(100,177, 1, 1000,34);
      //scraper 
      scraper.set_value(true);
      delay(200);
      RunpidStraightNTo(100,500,20,30,400,0,0,2000,0,39);
      delay(700);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -3, 1, 1000,19);
      RunpidStraightNTo(100,615,20,100,400,0,0,1000,0,39); //value that drives into goal
      blocker.set_value(true);
      delay(700);
      RunpidStraightNTo(100,-400,20,100,400,0,0,1000,0,39);
      colorSorter.set_value(false);
      TurnVolpidNTo(100, -50, 1, 1000,19);
      //run into goal
      RunpidStraightNTo(100,200,20,100,400,0,0,1000,0,39);
      descore.set_value(true);
      RunpidStraightNTo(100,800,20,100,400,0,0,1000,20,39);
      RunpidStraightNTo(100,800,20,100,400,0,0,1000,0,39);
      
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
      RunpidStraightNTo(30,280,10,30,400,0,0,1000,0,39);
      RunpidStraightNTo(100,450,10,100,400,0,0,1000,48,39);
      RunpidStraightNTo(100,1100,10,100,400,0,0,4000,0,39);
      RunpidStraightNTo(30,250,10,30,400,0,0,1000,0,39);
      scraper.set_value(true);
      delay(500);
      scraper.set_value(false);
      RunpidStraightNTo(100,-780,10,100,400,0,0,4000,0,39);
      TurnVolpidNTo(100, -45, 1, 800,34);
      delay(100);
      Fintake.move(-127);
      Mintake.move(-127);
      RunpidStraightNTo(100,800,10,100,400,0,0,1000,0,39);
      scraper.set_value(true);
      delay(600);
      scraper.set_value(false);
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,-1700,10,100,400,0,0,2000,0,39);
      linkage.set_value(false);
      delay(100);
      TurnVolpidNTo(100,-179, 1, 1000,34);
      scraper.set_value(true);
      delay(200);
      RunpidStraightNTo(100,550,20,30,400,0,0,2000,0,39);
      delay(700);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, 0, 1, 1000,19);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      Fintake.move(-127);
      Mintake.move(-127);
      

      //safe odd
      // RunpidStraightNTo(100,500,10,100,400,0,0,4000,0,39);
      // TurnVolpidNTo(100, 50, 1, 800,34);//the 600 is distance in mm and 30 is angle
      // RunpidStraightNTo(30,500,5,30,400,0,0,1000,0,39);
      // RunpidStraightNTo(30,-40,5,30,400,0,0,1000,0,39);
      // Fintake.move(0);
      // Mintake.move(0);
      // TurnVolpidNTo(100, -48, 1, 800,34);
      // blocker.set_value(true);
      // delay(10);
      // RunpidStraightNTo(50,620,20,100,400,0,0,4000,0,39);    
      // scraper.set_value(true);
      // Fintake.move(-127);
      // Mintake.move(-127);
      // delay(725);
      // scraper.set_value(false);
      // blocker.set_value(false);
      // Fintake.move(127);
      // Mintake.move(127);
      // RunpidStraightNTo(100,-1600,20,100,400,0,0,4000,0,39);
      // scraper.set_value(true);
      // linkage.set_value(false);
      // delay(50);
      // TurnVolpidNTo(100,-179, 1, 1000,34);
      // delay(50);
      // RunpidStraightNTo(100,600,20,100,400,0,0,1000,0,39); //was 700
      // delay(1200);
      // RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      // scraper.set_value(false);
      // TurnVolpidNTo(100, 3, 1, 800,33);
      // delay(50);
      // RunpidStraightNTo(100,650,20,100,400,0,0,1000,0,39);
      // blocker.set_value(true);
      // colorSorter.set_value(true);

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
      RunpidStraightNTo(100,1000,10,100,400,0,0,2000,0,39);
      Fintake.move(-127);
      Mintake.move(-127);
      delay(300);
      RunpidStraightNTo(100,-50,10,100,400,0,0,100,0,39);
      TurnVolpidNTo(100, 0, 1, 800,34);
      delay(50);
      RunpidStraightNTo(100,-750,10,100,400,0,0,1000,0,39);
      TurnVolpidNTo(100, -63, 1, 800,34);
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(70,1350,10,30,400,0,0,2000,0,39);
      TurnVolpidNTo(100, 46, 1, 800,34);
      Fintake.move(0);
      Mintake.move(0);
      linkage.set_value(true);
      scraper.set_value(true);
      RunpidStraightNTo(100,450,10,100,400,0,0,500,0,39);
      Fintake.move(-127);
      Mintake.move(-127);
      delay(100);
      Fintake.move(90);
      Mintake.move(127);
      blocker.set_value(true);
      //derrick was hereeeeeee
      delay(200);
      blocker.set_value(false);
      delay(600);
      scraper.set_value(false);
      linkage.set_value(false);
      RunpidStraightNTo(100,-1290,10,100,400,0,0,2000,0,39);
      Fintake.move(127);
      Mintake.move(127);
      delay(100);
      TurnVolpidNTo(100,-179, 1, 1000,34);
      scraper.set_value(true);
      delay(50);
      RunpidStraightNTo(100,620,20,100,400,0,0,600,0,39);
      delay(700);
      RunpidStraightNTo(100,-300,20,100,400,0,0,500,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -3, 1, 1000,19);
      RunpidStraightNTo(100,500,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);




      // TurnVolpidNTo(100, 50, 1, 800,34);//the 600 is distance in mm and 30 is angle
      // RunpidStraightNTo(30,500,5,30,400,0,0,1000,0,39);
      // RunpidStraightNTo(30,-80,5,30,400,0,0,1000,0,39);
      // Fintake.move(0);
      // Mintake.move(0);
      // TurnVolpidNTo(100, -90, 1, 800,34);
      // blocker.set_value(true);
      // RunpidStraightNTo(50,630,20,100,400,0,0,4000,0,39);
      // scraper.set_value(true);
      // Fintake.move(-60);
      // Mintake.move(-127);
      // delay(700);
      // blocker.set_value(false);
      // Fintake.move(127);
      // Mintake.move(127);
      // scraper.set_value(false);
      // RunpidStraightNTo(100,-500,20,100,400,0,0,4000,0,39);
      // Fintake.move(127);
      // Mintake.move(127);
      // TurnVolpidNTo(100, -50, 1, 800,34);
      // RunpidStraightNTo(100,1490,20,70,400,0,0,4000,0,39);
      // TurnVolpidNTo(100, 138, 1, 800,34);
      // linkage.set_value(true);
      // scraper.set_value(true);
      // RunpidStraightNTo(100,380,20,60,400,0,0,4000,0,39);
      // blocker.set_value(true);
      // delay(500);
      // blocker.set_value(false);
      // scraper.set_value(false);
      // RunpidStraightNTo(100,-1300,20,70,400,0,0,4000,0,39);
      // linkage.set_value(false);
      // TurnVolpidNTo(100, 128, 1, 800,34);
      // scraper.set_value(true);
      // RunpidStraightNTo(100,600,20,70,400,0,0,4000,0,39);
      // blocker.set_value(false);
      // RunpidStraightNTo(100,-400,20,70,400,0,0,4000,0,39);
      // scraper.set_value(false);

      // TurnVolpidNTo(100, 178, 1, 800,34);
      // RunpidStraightNTo(100,500,20,70,400,0,0,4000,0,39);
      // blocker.set_value(false);
  } 
  
  else if (atn == 5){
  //red left elims with push
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,300,10,100,400,0,0,500,0,39);
      RunpidStraightNTo(100,250,10,100,400,0,0,4000,-60,39);
      RunpidStraightNTo(100,1100,10,100,400,0,0,4000,-0,39);
      RunpidStraightNTo(30,500,10,30,400,0,0,200,-0,39);
      delay(200);
      RunpidStraightNTo(100,-680,10,100,400,0,0,4000,0,39);
      TurnVolpidNTo(100, 45, 1, 800,34);
      scraper.set_value(true);
      linkage.set_value(true);
      RunpidStraightNTo(100,350,10,100,400,0,0,4000,0,39);
      blocker.set_value(true);
      delay(500);
      blocker.set_value(false);
      scraper.set_value(false);
      RunpidStraightNTo(100,-1470,10,100,400,0,0,4000,0,39);
      linkage.set_value(false);
      delay(100);
      TurnVolpidNTo(100,-179, 1, 1000,34);
      scraper.set_value(true);
      delay(200);
      RunpidStraightNTo(100,430,20,30,400,0,0,2000,0,39);
      delay(700);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, -5, 1, 1000,19);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      // scraper.set_value(false);
      // RunpidStraightNTo(100,-400,10,100,400,0,0,4000,0,39);//tune this value for the connection between the angle facing towards middle goal
      // TurnVolpidNTo(100, -90, 1, 800,34);
      // blocker.set_value(true);
      // delay(10);
      // RunpidStraightNTo(50,620,20,100,400,0,0,4000,0,39);
      // Fintake.move(-60);
      // Mintake.move(-127);
      // delay(800);
      // blocker.set_value(false);
      // Fintake.move(127);
      // Mintake.move(127);
      // RunpidStraightNTo(100,-1730,20,100,400,0,0,4000,0,39);
      // scraper.set_value(true);
      // linkage.set_value(false);
      // delay(50);
      // TurnVolpidNTo(100,-133, 1, 1000,34);
      // delay(50);
      // RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      // delay(1200);
      // RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      // scraper.set_value(false);
      // TurnVolpidNTo(100, -179, 1, 800,33);
      // delay(50);
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
      Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,500,10,100,400,0,0,500,0,39);

  } 
  
  else if (atn == 7){
    //7 goal long
       Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,500,10,100,400,0,0,4000,-40,39);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,380,5,100,400,0,0,1000,0,39);
      TurnVolpidNTo(100,90, 1, 1000,34);
      RunpidStraightNTo(100,770,20,100,400,0,0,4000,0,39);
      linkage.set_value(false);
      delay(200);
      TurnVolpidNTo(100,50, 1, 1000,34);
      delay(50);
      RunpidStraightNTo(100,750,20,30,400,0,0,2000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, 174, 1, 1000,19);
      delay(50);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      delay(200);
      colorSorter.set_value(true);
      delay(1200);
      RunpidStraightNTo(100,-400,20,100,400,0,0,1000,0,39);
      descore.set_value(true);
      delay(200);
      TurnVolpidNTo(100, -40, 1, 1000,19);
      RunpidStraightNTo(100,200,20,100,400,0,0,1000,0,39);
      RunpidStraightNTo(100,800,20,100,400,0,0,1000,40,39);
      RunpidStraightNTo(100,400,20,100,400,0,0,1000,0,39);
  } 
  
  else if (atn == 8){
    //elims(run for not )
     Fintake.move(127);
      Mintake.move(127);
      RunpidStraightNTo(100,500,10,100,400,0,0,4000,-40,39);//the 600 is distance in mm and 30 is angle
      RunpidStraightNTo(30,380,5,100,400,0,0,1000,0,39);
      linkage.set_value(true);
      scraper.set_value(true);
      Fintake.move(0);
      Mintake.move(0);
      TurnVolpidNTo(100, 80, 1, 800,34);
      blocker.set_value(true);
      RunpidStraightNTo(50,280,20,100,400,0,0,4000,0,39);
      Fintake.move(127);
      Mintake.move(127);
      delay(1200);
      blocker.set_value(false);
      RunpidStraightNTo(100,-1470,20,100,400,0,0,4000,0,39);
      linkage.set_value(false);
      delay(200);
      TurnVolpidNTo(100,130, 1, 1000,34);
      delay(50);
      RunpidStraightNTo(100,750,20,30,400,0,0,2000,0,39);
      delay(1200);
      RunpidStraightNTo(100,-300,20,100,400,0,0,1000,0,39);
      scraper.set_value(false);
      TurnVolpidNTo(100, 174, 1, 1000,19);
      delay(50);
      RunpidStraightNTo(100,700,20,100,400,0,0,1000,0,39);
      blocker.set_value(true);
      delay(200);
      colorSorter.set_value(true);
      delay(1200);
      RunpidStraightNTo(100,-400,20,100,400,0,0,1000,0,39);
      descore.set_value(true);
      delay(200);
      TurnVolpidNTo(100, -40, 1, 1000,19);
      RunpidStraightNTo(100,200,20,100,400,0,0,1000,0,39);
      RunpidStraightNTo(100,800,20,100,400,0,0,1000,40,39);
      RunpidStraightNTo(100,400,20,100,400,0,0,1000,0,39);
  }
}
