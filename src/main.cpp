// basically run all of the code here (PID, auton, etc.)

#include "main.h"
#include "api.h"
#include "auton.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"

using namespace pros;
using namespace std;



// void resetEncoders() { //we can't add this to main.h because main.h doesn't
// refer to robot.h (where LF, LB, etc. are located) 	LF.tare_position(); //or
// set_zero_position(0) or set_zero_position(LF.get_position()); (sets current
// encoder position to 0) 	LB.tare_position(); 	RF.tare_position();
// 	RB.tare_position();
// }

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  OpticalC.set_led_pwm(100);
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");
  //imu.tare_position();
  ODOMY.reset_position();
  ODOMX.reset_position();
  //pros::Task kalman(kalmanTask, nullptr, "Kalman Filter Task");


  // pros::lcd::register_btn1_cb(on_center_button);
  // optical.set_led_pwm(100);


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */



int atn = 5;
int ballColor = 2;
int color = 0;
int pressed = 0;
string autstr;
float errorp;
bool mogoToggle = false;
bool scraperToggle = false;
bool blockerToggle = false;
bool colorSorterToggle = false;
bool linkageToggle = false;

bool parkToggle = false;
bool ptoToggle = false;
bool liftToggle = false;


 
void competition_initialize() {

  
    while(true) {
      // if(selec.get_value() == true) {
      //   atn ++;  
      //   delay(350);
      // }

      if(selec.get_value() == true) {
        pressed ++;  
      } else {
        pressed = 0;
      }

      if (pressed == 1){
        atn++;
      }

      if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
        atn++;
      }
      if(atn>8){
        atn = 0;
      }

 


      //resetEncoders();
      
      if (atn == 0) {
        autstr = "skills";
        con.print(0, 0, "Aut 0: %s", autstr);
      }
      else if (atn == 1) {
        autstr = "3+4 left";
        con.print(0, 0, "Aut 1: %s", autstr);
      }
      else if (atn == 2) {
        autstr = "7 ball long";
        con.print(0, 0, "Aut 2: %s", autstr);
      }
      else if (atn == 3) {
       autstr = "9 ball long";
        con.print(0, 0, "Aut 3: %s", autstr);
      }
      else if (atn == 4) {
       autstr = "14 ball SAWP";
        con.print(0, 0, "Aut 4: %s", autstr);
      }
      else if (atn == 5) {
       autstr = "ELIMS BLUE RIGHT PUSH";
        con.print(0, 0, "Aut 5: %s", autstr);
      }
      else if (atn == 6) {
       autstr = "7 ball SAWP";
        con.print(0, 0, "Aut 6: %s", autstr);
      } 
      else if (atn == 7) {
        autstr = "BLUE GOAL SAFE";
         con.print(0, 0, "Aut 6: %s", autstr);
       } 
       else if (atn == 8) {
        autstr = "RED GOAL SAFE";
         con.print(0, 0, "Aut 6: %s", autstr);
       } 
      else if (atn == 9) {
       atn = 0;
      }
  
      con.clear();
    }
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
  int cycle = 0;
  int time = 0;
  bool NEWL1 = false;
  bool NEWL2 = false;
  bool NEWR2 = false;
  bool NEWR1 = false;
  bool arcToggle = false;
  bool tankToggle = true;
  double maxRPM = 0;
  double motorTotal = 0;
  double avgRPM = 0;
  double liftAngle = 0; 
  double rotoAngle = 0;
  float xvelo = 0;
  int macro = 1;
  bool macroControl = false;
  bool hookControl = false;
  bool descoreToggle = true;


  imu.tare_heading();



	while (true) {

    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
      scraperToggle  = !scraperToggle;
      if(scraperToggle == true){
        scraper.set_value(false);
      }
      else{
        scraper.set_value(true);
      }
    }
    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
      descoreToggle  = !descoreToggle;
      if(descoreToggle == true){
        descore.set_value(false);
      }
      else{
        descore.set_value(true);
      }
    }
    //derrae
     if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
      liftToggle = !liftToggle;
      if(liftToggle == true){
        lift.set_value(false);
      }
      else{
        lift.set_value(true);

      }
    }
    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
      parkToggle = !parkToggle;
      if(parkToggle == true){
        park.set_value(false);
      }
      else{
        park.set_value(true);
      }
    }
    //(a)b is descore, y is string blocker, right is color sort (b)is linkage
    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
      drive_to_point(0, 20, 50, 100, 80, 1, 0, 800, 1, 1, 10, 0,0);
      // TurnVolpidNTo(100, 100, 1, 2000,30);
      // RunpidStraightNToHC(100,800,20,60,400,0,0,2000,0,39);
    }
//TurnVolpidNTo(100, 180, 1, 2000,30);/34 for 120
//RunpidStraightNTo(100,1000,20,60,400,0,0,2000,0,39);
    if (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {//score mode
      pto.set_value(true);
			Lintake.move(127);
      Rintake.move(-127);
      Lintake.tare_position();
      Rintake.tare_position();
		} 
    else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){//store mode
      pto.set_value(false);
      Lintake.move(127);
      Rintake.move(-127);
      Lintake.tare_position();
      Rintake.tare_position();
    }
    else if (con.get_digital(E_CONTROLLER_DIGITAL_L2)) {//outake mode
      Lintake.move(-127);
      Rintake.move(127);
      Lintake.tare_position();
      Rintake.tare_position();
    }
    else if (con.get_digital(E_CONTROLLER_DIGITAL_UP)) {//middle goal mode
      Lintake.move(60);
      Rintake.move(-60);
      Lintake.tare_position();
      Rintake.tare_position();

    }
    else {
      Lintake.move(0);
      Rintake.move(0);
    }

  pros::c::imu_accel_s_t accel = imu.get_accel();

  // if(abs(accel.x)>0.04){
  // xvelo += accel.x;
  // }
  xvelo += accel.x-0.032;

    OpticalC.set_led_pwm(100);

    //TEST2.move_velocity(300);
    if(TEST.get_actual_velocity() > maxRPM){
      maxRPM = TEST.get_actual_velocity();
    }

    

    motorTotal += TEST.get_actual_velocity();
    cycle++;
    avgRPM = motorTotal/cycle;



  
		//chassis arcade drive
		int power = con.get_analog(ANALOG_LEFT_Y); //power is defined as forward or backward
		int RX = con.get_analog(ANALOG_RIGHT_X); //turn is defined as left (positive) or right (negative)



    int turn = int(RX); // Normal Rates
		// int turn = int(abs(RX) * RX / 127); //X Squared Rates
    // int turn = int(pow(RX, 3) / pow(127, 2)); //X Cubed Rates

		int left = power + turn;
		int right = power - turn;

    // //switch between arcade and tank
    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
      arcToggle = !arcToggle;
      tankToggle = !tankToggle;
    }


   
    if (tankToggle) {
      LF.move(con.get_analog(ANALOG_LEFT_Y));
      LM.move(con.get_analog(ANALOG_LEFT_Y));
      LB.move(con.get_analog(ANALOG_LEFT_Y));
      RF.move(con.get_analog(ANALOG_RIGHT_Y));
      RM.move(con.get_analog(ANALOG_RIGHT_Y));
      RB.move(con.get_analog(ANALOG_RIGHT_Y));
    }
    if (arcToggle) {
      LF.move(left);
      LM.move(left);
      LB.move(left);
      RF.move(right);
      RM.move(right);
      RB.move(right);
    }




    //auton selector
    if (selec.get_value() == true) { 
      atn++;
      delay(350);
    }
      // brain was here
    // switch(atn){
    //   case 0:
    //     autstr = "Auton 0";
    //     break;
    //   case 1:
    //     autstr = "Auton 1";
    //     break;
    //   case 2:
    //     autstr = "Auton 2";
    //     break;
    //   case 3:
    //     autstr = "Auton 3";
    //     break;
    //   case 4:
    //     autstr = "Auton 4";
    //     break;
    //   case 5:
    //     autstr = "Auton 5";
    //     break;
    //   case 6:
    //     autstr = "Auton 6";
    //     break;
    //   case 7:
    //     atn = 0;
    //     break;

    // }
    
    if (atn == 0) {
      autstr = "SKILLS";
    }
    if (atn == 1) {
      autstr = "3+4 left";
    }
    else if (atn == 2) {
      autstr = "3+4 right";
    }
    else if (atn == 3) {
      autstr = "9 ball right";
    }
    else if (atn == 4) {
      autstr = "14 ball";
    }
    else if (atn == 5) {
      autstr = "7 ball right";
    } 
    else if (atn == 6) {
      autstr = "sawp";
    }
    else if (atn == 7) {
      autstr = "BLUE GOAL SAFE";
    }
    else if (atn == 8) {
      autstr = "4 ball rush";
    }
    else if (atn == 9) {
      atn = 0;
    }

      //printing stuff
		double chasstempC = ((RF.get_temperature() + RB.get_temperature() + LF.get_temperature() + LB.get_temperature())/4);
    double intaketempc = ((Lintake.get_temperature() + Rintake.get_temperature())/2);
    if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0){
      con.print(0, 0, "AUTON: %s           ", autstr);
      
    } else if (time % 100 == 0 && time % 150 != 0){
        con.print(1, 0, "imu: %f         ", imu.get_rotation());
        
    } 
    else if (time % 150 == 0){
      con.print(2, 0, "C:%i int:%i ", int(chasstempC), int(intaketempc)); 
    } 

	  	time += 1;
		  delay(1);
	  }
  }
  
    