// basically run all of the code here (PID, auton, etc.)

#include "main.h"
#include "api.h"
#include "auton.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "derrickPID.h"


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

//--------------------------------------------------------------------------------------------------------------------------------------
//SELECT AUTON HERE
int atn = 7;
//--------------------------------------------------------------------------------------------------------------------------------------
string autstr;


 
void competition_initialize() {
    if (atn == 0)      { autstr = "skills";       con.print(0, 0, "Aut 0: %s", autstr); }
    else if (atn == 1) { autstr = "6+3 rush";     con.print(0, 0, "Aut 1: %s", autstr); }
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
  con.clear();
  int time = 0;
  bool arcToggle = false;
  bool tankToggle = true;
  bool liftToggle = false;
  bool descoreToggle = false;
  bool blockerToggle = false;
  bool scraperToggle = false;
  LF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  LM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  LB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  RF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  RM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  RB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  imu.tare_heading();

  while (true) {

    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
        liftToggle = !liftToggle;
      lift.set_value(liftToggle);
      descore.set_value(false);
    }

    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
      descoreToggle = !descoreToggle;
      descore.set_value(descoreToggle);
    }

    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
      // scraperToggle = !scraperToggle;
      // scraper.set_value(scraperToggle);
      drivePIDW(1500, 60, 1200, 1130, -1, -1, -1, -1, 15, 50, 1, 675, 30, 0, -1, 0);
    }
    

    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
    pros::Task leverTask([](){
        blocker.set_value(false);
        leverPID(750, 85, 2000, 100, 10, 550, 50);
        pros::delay(500);
        blocker.set_value(true);
        leverPID(-550, 127, 2000, 100, 10, 400, 50);
        Lever.move(-20);
        pros::delay(800);
        Lever.move(0);
        pros::Task::current().remove();
    });
}


    if (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {//score mode
			intake.move(127);
      intake.tare_position();

		} 
    else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){//store mode
      intake.move(-127);
      intake.tare_position();
    }

    else if (con.get_digital(E_CONTROLLER_DIGITAL_UP)) {//middle goal mode
      intake.move(-60);
      intake.tare_position();

    }

    else {
      intake.move(0);
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




       // set autstr once per loop without printing
    if      (atn == 0) autstr = "skills";
    else if (atn == 1) autstr = "3+4 left";
    else if (atn == 2) autstr = "3+4 right";
    else if (atn == 3) autstr = "7 left";
    else if (atn == 4) autstr = "7 right";
    else if (atn == 5) autstr = "4 Fast Left";
    else if (atn == 6) autstr = "4 Fast Right";
    else if (atn == 7) autstr = "sawp";
    else if (atn == 8) autstr = "misc";

    double chasstempC = ((RF.get_temperature() + RB.get_temperature() + RM.get_temperature() + LF.get_temperature() + LB.get_temperature() + LM.get_temperature()) / 6);
    double intaketempc = intake.get_temperature();

      if (time % 101 == 0) {
        con.print(0, 0, "AUTON: %s           ", autstr);
    }
    if (time % 103 == 0) {
        con.print(1, 0, "imu: %.2f           ", imu.get_rotation());
    } 
    if (time % 107 == 0) {
        con.print(2, 0, "C:%i int:%i         ", int(chasstempC), int(intaketempc)); 
    }
    time += 1;
    delay(1);
  }
}