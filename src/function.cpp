#include "vex.h"
#include "robot.h"

using namespace vex;
using namespace pros;
using namespace std;

using signature = vision::signature;
using code = vision::code;

void set_location(float X_value,float Y_value){
X_position=X_value;
Y_position=Y_value;
}

float compute(float error){
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float accumulated_error = 0;
  float previous_error = 0;
  float output = 0;
  if (fabs(error) < 100){
    accumulated_error+=error;
  }
  if ((error>0 && previous_error<0)||(error<0 && previous_error>0)){ 
    accumulated_error = 0; 
  }
  output = kp*error + ki*accumulated_error + kd*(error-previous_error);

  previous_error=error;

  return output;
}

//getting the x position
float get_X_position(){
  return(X_position);
}

// getting the y position
float get_Y_position(){
  return(Y_position);
}

//making sure angle is within -180 and 180
float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { 
        angle += 360; 
    }
    if(angle >= 180) { 
        angle -= 360; 
    }
  }
  return(angle);
}

//making sure angle is within -90 and 90
float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { 
        angle += 180; 
    }
    if(angle >= 90) { 
        angle -= 180;
     }
  }
  return(angle);
}

//angle to radians
float to_rad(float angle_deg){
  return(angle_deg/(180.0/M_PI));
}

//radians to angle
float to_deg(float angle_rad){
  return(angle_rad*(180.0/M_PI));
}

//getting the sign
int sgn(double number) {

  if (number < 0)
    return -1;
  else if (number == 0)
    return 0;
  else
    return 1;
}

//
int setMotorPct(int Value) {
  if (Value < 0) {
    Value = -PutPct[-Value];
  } else if (Value > 0) {
    Value = PutPct[Value];
  }
  return 0;
}