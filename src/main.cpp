// basically run all of the code here (PID, auton, etc.)

#include "main.h"
#include "api.h"
#include "auton.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"


using namespace pros;
using namespace std;


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
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");
  //imu.tare_position();




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
//did i fix the compiler



int atn = 0;
string autstr;
bool scraperToggle = false;
bool liftToggle = false;


 
void competition_initialize() {
    if (atn == 0)      { autstr = "skills";       con.print(0, 0, "Aut 0: %s", autstr); }
    else if (atn == 1) { autstr = "3+4 left";     con.print(0, 0, "Aut 1: %s", autstr); }
    else if (atn == 2) { autstr = "3+4 right";    con.print(0, 0, "Aut 2: %s", autstr); }
    else if (atn == 3) { autstr = "7 left";       con.print(0, 0, "Aut 3: %s", autstr); }
    else if (atn == 4) { autstr = "7 right";      con.print(0, 0, "Aut 4: %s", autstr); }
    else if (atn == 5) { autstr = "4 Fast Left";  con.print(0, 0, "Aut 5: %s", autstr); }
    else if (atn == 6) { autstr = "4 Fast Right"; con.print(0, 0, "Aut 6: %s", autstr); }
    else if (atn == 7) { autstr = "sawp";         con.print(0, 0, "Aut 7: %s", autstr); }
    else if (atn == 8) { autstr = "misc";         con.print(0, 0, "Aut 8: %s", autstr); }
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
  int time = 0;
  bool arcToggle = false;
  bool tankToggle = true;
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

    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
      descoreToggle  = !descoreToggle;
      if(descoreToggle == true){
        descore.set_value(false);
      }
      else{
        descore.set_value(true);
      }
    }
    
     if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
      liftToggle = !liftToggle;
      if(liftToggle == true){
        lift.set_value(false);
        descore.set_value(false);
      }
      else{
        lift.set_value(true);
        descore.set_value(false);

      }
    }
    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
  

    }
   
    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
      
    }

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
      // LF.move(int(abs(con.get_analog(ANALOG_LEFT_Y)) * con.get_analog(ANALOG_LEFT_Y) / 127));
      // LM.move(int(abs(con.get_analog(ANALOG_LEFT_Y)) * con.get_analog(ANALOG_LEFT_Y) / 127));
      // LB.move(int(abs(con.get_analog(ANALOG_LEFT_Y)) * con.get_analog(ANALOG_LEFT_Y) / 127));
      // RF.move(int(abs(con.get_analog(ANALOG_RIGHT_Y)) * con.get_analog(ANALOG_RIGHT_Y) / 127));
      // RM.move(int(abs(con.get_analog(ANALOG_RIGHT_Y)) * con.get_analog(ANALOG_RIGHT_Y) / 127));
      // RB.move(int(abs(con.get_analog(ANALOG_RIGHT_Y)) * con.get_analog(ANALOG_RIGHT_Y) / 127));
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




    if (atn == 0)      { autstr = "skills";       con.print(0, 0, "Aut 0: %s", autstr); }
    else if (atn == 1) { autstr = "3+4 left";     con.print(0, 0, "Aut 1: %s", autstr); }
    else if (atn == 2) { autstr = "3+4 right";    con.print(0, 0, "Aut 2: %s", autstr); }
    else if (atn == 3) { autstr = "7 left";       con.print(0, 0, "Aut 3: %s", autstr); }
    else if (atn == 4) { autstr = "7 right";      con.print(0, 0, "Aut 4: %s", autstr); }
    else if (atn == 5) { autstr = "4 Fast Left";  con.print(0, 0, "Aut 5: %s", autstr); }
    else if (atn == 6) { autstr = "4 Fast Right"; con.print(0, 0, "Aut 6: %s", autstr); }
    else if (atn == 7) { autstr = "sawp";         con.print(0, 0, "Aut 7: %s", autstr); }
    else if (atn == 8) { autstr = "misc";         con.print(0, 0, "Aut 8: %s", autstr); }




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
  
    