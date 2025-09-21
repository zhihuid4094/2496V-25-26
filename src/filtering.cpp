#include "main.h"
#include "api.h"
#include "auton.h"
#include "pid.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "odometry.h"
#include "pure_pursuit.h"

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"



// class KalmanFilter2D {
// public:
//     Vector4d x;     // [x_pos, x_vel, y_pos, y_vel]
//     Matrix4d P;     // Covariance
//     Matrix4d A;     // State transition
//     Matrix4d Q;     // Process noise
//     Matrix4d R;     // Measurement noise
//     double dt;

//     KalmanFilter2D(double time_step) : dt(time_step) {
//         x << 0, 0, 0, 0;
//         P = Matrix4d::Identity() * 0.1;

//         A << 1, dt, 0, 0,
//              0, 1,  0, 0,
//              0, 0,  1, dt,
//              0, 0,  0, 1;

//         Q = Matrix4d::Identity() * 0.05;
//         R = Matrix4d::Identity() * 0.1;
//     }

//     void predict() {
//         x = A * x;
//         P = A * P * A.transpose() + Q;
//     }

//     void update(const Vector4d& z) {
//         Vector4d y = z - x;
//         Matrix4d S = P + R;
//         Matrix4d K = P * S.inverse();
//         x = x + K * y;
//         P = (Matrix4d::Identity() - K) * P;
//     }

//     Vector2d getPosition() {
//         return Vector2d(x(0), x(2)); // [x_pos, y_pos]
//     }
// };