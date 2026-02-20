#include "main.h"
#include "robot.h"

int screenchoose() {

    pros::lcd::initialize();

    while (!initializing || SCREEN_ON) {

        if (BSCREEN_ON) {

            // ---- LCD (Brain replacement) ----
            pros::lcd::set_text(0, "L1: " + std::to_string(LF.get_position()));
            pros::lcd::set_text(1, "L2: " + std::to_string(LM.get_position()));
            pros::lcd::set_text(2, "L3: " + std::to_string(LB.get_position()));
            pros::lcd::set_text(3, "R1: " + std::to_string(RF.get_position()));
            pros::lcd::set_text(4, "R2: " + std::to_string(RM.get_position()));
            pros::lcd::set_text(5, "R3: " + std::to_string(RB.get_position()));
        }

        if (CSCREEN_ON) {

            // ---- Controller Screen ----
            pros::Controller con(pros::E_CONTROLLER_MASTER);

            con.clear();

            con.print(0, 0, "Drg: %.2f", return_angle);
            con.print(0, 10, "BT: %d", pros::battery::get_capacity());

            con.print(1, 0, "Base: %.0f", LF.get_position());
            con.print(2, 0, "X: %.2f", X_position);
            con.print(2, 8, "Y: %.2f", Y_position);

        }

        pros::delay(20);
    }

    return 0;
}