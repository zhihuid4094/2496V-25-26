#ifndef DERRICKPIDH
#define DERRICKPIDH

#include "api.h"
#include "main.h"
#include "robot.h"

// UNIVERSAL HEADING
extern double universal_target_heading;

// Drive PID constants
extern double driveKP;
extern double driveKI;
extern double driveKD;
extern double driveMAXI;

// Heading correction constants
extern double HCKP;
extern double HCKI;
extern double HCKD;
extern double HCMAXI;

// Wall PID constants
extern double wallKP;
extern double wallKI;
extern double wallKD;

// Turn PID constants
extern double turnKP;
extern double turnKI;
extern double turnKD;
extern double turnMAXI;

// Utility functions
extern void chasMove(int left, int right);
extern void resetEncoders();
extern void chasBrake();
extern void chasSlow(int speed, int ms);

// PID calc
extern double calcPID(int error, double kP, double kI, double kD, double totalError, double prevError, double integralThreshold, double maxI);

// Drive functions
extern void drivePID(int desiredValue, int maxSpeed, int timeout, int errorThreshold, int settleCount, int dec_point, int minSpeed, int chainValue, int triggerDist, int triggerSpeed);
extern void drivePIDW(int desiredValue, int maxSpeed, int timeout, int wallDistanceTarget, int wallOffStart1, int wallOnAgain1, int wallOffStart2, int wallOnAgain2, int errorThreshold, int settleCount, int sensorSide, int dec_point, int minSpeed, int chainValue,  int triggerDist, int triggerSpeed);

// Turn functions
extern void turnPID(double desiredValue, int topSpeed, int timeout, double errorThreshold, int settleCount, double chainValue);

// Arc functions
extern void driveArcL(double theta, double radius, int timeout, int speed, int errorThreshold, int settleCount, int chainValue);
extern void driveArcR(double theta, double radius, int timeout, int speed, int errorThreshold, int settleCount, int chainValue);

// Lever
extern void leverPID(int derivedValue, int maxSpeed, int timeout, int errorThreshold, int settleCount, int dec_point, int minSpeed);

#endif