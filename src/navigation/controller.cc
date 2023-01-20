// //========================================================================
// /*!
// \file    controller.cc
// \brief   Starter code for controller.
// \author  Nihar, Jared, Eric, Daniel
// */
// //========================================================================

// #include "glog/logging.h"
// #include "shared/math/math_util.h"
// #include "shared/util/timer.h"
// #include "controller.h"

// using Eigen::Vector2f;
// using std::string;
// using std::vector;

// using namespace math_util;

// namespace controller {

// Controller::Controller() {

// }

// // Return the velocity for upcoming time-step
// void Controller::CalculateValues(double vCurrent, double vMax, double aMax, double distanceTravelled, double goalDist, double *results) {
//   // Minimum distance required to decelerate = vCurrent/(2*aMax)
//   // Time required to deaccelerate = vCurrent/aMax
//   double distanceLeft;

//   const double dt = 0.05; // 1/20th of a second, plug into equations to calculate distance travelled in this time frame
  
//   distanceLeft = goalDist - distanceTravelled; // subtracts global distance travelled from our goal distance

//     //acceleration
//   if ((vCurrent < vMax) && (distanceLeft-(vCurrent+aMax*dt) >= vMax*vMax/(2*aMax))) {
//     // Use kinematic equation to check if we have enough distanceleft to keep increasing velocity
//     Acceleration(vMax, vCurrent, aMax, dt, distanceLeft, results);
//   }
//     //cruise
//   else if((vMax == vCurrent) && (distanceLeft-vMax*dt >= vMax*vMax/(2*aMax))){
//     Cruise(vMax, vCurrent, aMax, dt, distanceLeft, results)
//   }
//     //deceleration
//   else {
//     Deceleration(vMax, vCurrent, aMax, dt, distanceLeft, results)
//   }

//       // Clear previous visualizations.

//       // Add timestamps to all messages.
//       //   local_viz_msg_.header.stamp = ros::Time::now();
//       //   global_viz_msg_.header.stamp = ros::Time::now();
//       //   drive_msg_.header.stamp = ros::Time::now();
//       // Publish messages.
//       //   viz_pub_.publish(local_viz_msg_);
//       //   viz_pub_.publish(global_viz_msg_);
//       //   drive_pub_.publish(drive_msg_);
//       ros::spinOnce();
// }

// // Return 1D array w/ two doubles
// // array[0] = velocity, array[1] = curature (0 for now)
// void Controller::Acceleration(double vMax, double vCurrent, double aMax, double dt, double distanceLeft, double results) {
//     // Kinematic equation?
//     //double results[2];
//     double controlVelocity = vCurrent + aMax*dt;
    
//     results = controlVelocity;
//     // results[1] = 0;
//     //return results;
// }

// void Controller::Cruise(double vMax, double vCurrent, double aMax, double dt, double distanceLeft, double results) {
//     //double results[2];
//     double controlVelocity = vMax;

//     results = controlVelocity
//     // results[1] = 0; //curvature
//     //return results;
// }

// // Calculate the deceleration and Return the velocity needed for deceleration
// void Controller::Deceleration(double vMax, double vCurrent, double aMax, double aCurrent, double dt, double distanceLeft, double goalDist, double results) {
//     d = vCurrent*vCurrent/(2*(distanceLeft));
//     //double results[2];
//     double controlVelocity = vCurrent - d*dt;
//     results = controlVelocity
//     // results[1] = 0; //curvature
//     //return results;
// }

// }  // namespace controller
