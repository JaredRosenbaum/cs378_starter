//========================================================================
/*!
\file    controller.cc
\brief   Starter code for controller.
\author  Nihar, Jared, Eric, Daniel
*/
//========================================================================

#include "glog/logging.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "controller.h"
#include <iostream>

using std::string;
using std::vector;

using namespace math_util;

namespace {
  const float dt = 0.05; // 1/20th of a second, plug into equations to calculate distance travelled in this time frame
  const float vMax = 1.0;
  const float aMax = 3.0;
  float goalDist;
}  // namespace

namespace navigation {

// Return the control velocity that will be used for the upcoming time-step.
float Controller::Run(float vCurrent, float distanceTraveled, float cp1_distance, float free_path_length) {
  float distanceLeft;
  float controlVelocity;

  // Robot checks to see if the distance to an obstacle is less than the assigned goal distance.
  // If so, it overwrites the assigned goal distance with its collision avoidance model.
  goalDist = cp1_distance;
  if (free_path_length >= 10.0){
    free_path_length = 10000000.0; //... workaround for when goal dist > 10
  }
  if (free_path_length < goalDist - distanceTraveled){
    distanceLeft = free_path_length - 0.5;
  }
  else {
    distanceLeft = goalDist - distanceTraveled; // subtracts global distance travelled from our goal distance
  }

  // ACCELERATION CASE
  // Use kinematic equation to check if we have enough distance left to accelerate one more time-step
  if ((vCurrent < vMax) && (distanceLeft-(vCurrent+aMax*dt) >= vMax*vMax/(2*aMax))) {
    controlVelocity = vCurrent + aMax*dt;
  }

  // CRUISE CASE
  // Use kinematic equation to check if we have enough distance left to cruise one more time-step
  else if((vMax == vCurrent) && (distanceLeft-vMax*dt >= vMax*vMax/(2*aMax))){
    controlVelocity = vMax;
  }
  // DECELERATION CASE
  // Not enough distance left, calculate deceleration rate
  else {
    float d = vCurrent*vCurrent/(2*(distanceLeft));
    controlVelocity = vCurrent - d*dt;
  }

  // In case of overshoot, stop!
  if (distanceLeft <= 0) {
    controlVelocity = 0.0;
  }

  return controlVelocity;
}

}  // namespace navigation
