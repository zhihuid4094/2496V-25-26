#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
#include "odometry.h"

PID::PID(float error, float kp, float kd, float starti):
error(error),
kp(kp),
ki(ki),
kd(kd),
starti(starti)
{};

PID::PID(float error, float kp, float ki, float kd, float starti, 
float settle_error, float settle_time, float timeout, float update_period) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  settle_time(settle_time),
  timeout(timeout),
  update_period(update_period)
{};

float PID::compute(float error){
  if (fabs(error) < starti){
    accumulated_error+=error;
  }

  if ((error>0 && previous_error<0)||(error<0 && previous_error>0)){ 
    accumulated_error = 0; 
  }

  output = kp*error + ki*accumulated_error + kd*(error-previous_error);

  previous_error=error;

  if(fabs(error)<settle_error){
    time_spent_settled+=10;
  } else {
    time_spent_settled = 0;
  }

  time_spent_running+=10;

  return output;
}

bool PID::is_settled(){
  if (time_spent_running>timeout && timeout != 0){
    return(true);
  } // If timeout does equal 0, the move will never actually time out. Setting timeout to 0 is the 
    // equivalent of setting it to infinity.
  if (time_spent_settled>settle_time){
    return(true);
  }
  return(false);
}