//========================================================================
/*!
\file    controller.cc
\brief   Starter code for controller.
\author  Nihar, Jared, Eric, Daniel
*/
//========================================================================

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "glog/logging.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "controller.h"
#include <iostream>
#include <math.h>

using Eigen::Vector2f;
using std::string;
using std::vector;

using namespace math_util;

namespace {
  const float dt = 0.05; // 1/20th of a second, plug into equations to calculate distance travelled in this time frame
  const float vMax = 1.0;
  const float aMax = 3.0;
  float goalDist;

  // Free path length variables
  float r, r1, r2, r_dist, theta, theta_new;

  // Clearance calculation variables
  float max_clearance = 0.5;
  float c, f;
}  // namespace

namespace navigation {

// Return the control velocity that will be used for the upcoming time-step.
float Controller::Run(float vCurrent, float distanceTraveled, float cp1_distance, float free_path_length) {
  float distanceLeft;
  float controlVelocity;
  goalDist = cp1_distance;
  float car_margin = 0.5;

  // TODO Daniel: This would not do anything. The LiDAR never gives a reading > 10.0
  // REPONSE Jared: The purpose of this is to not overwrite the goal distance with 10 when the lidar = 10.0.
  // cont. The >= is not necessary, it could also be == 10.0, but it serves the same purpose and can catch weird errors this way.
  // If you remove this you'll see the error that occurs. 
  if (free_path_length >= 9.0){
    free_path_length = 10000000.0; // ... workaround for when goal dist > 10
  }

  // OBSTACLE CASE
  // We detected a free path length closer than out actual end goal.
  if (free_path_length < goalDist - distanceTraveled){
    distanceLeft = free_path_length - car_margin;  // adjust for front of car + margin
  }


  
  // GOAL CASE
  // We will reach the end goal before colliding with any obstacle.
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

// Calculate free path length between the car and object.
float Controller::FreePathLength(std::vector<Eigen::Vector2f> point_cloud_, float cp3_curvature) {
  Vector2f p(0.0, 0.0);
  float f_min = 9.0;
  r = cp3_curvature;

  // Moving in a straight line
  if (abs(r) < 0.01) {
    // Loop through pointcloud
    for (int i = 0; i < (int)point_cloud_.size(); i++) {
      p = point_cloud_[i];

      // Make all coordinates positive
      if (p.y() < 0) {
        p.y() = p.y()*(-1);
      }

      // Update minimum free path length
      if (p.y() < 0.2405 && p.x() < f_min) {
        f_min = p.x();
      }
    } // At this point, min_dist is the closest point in front of the car.
  }

  // Moving along an arc
  else {
    r = abs(r);   // absolute value to handle both left/right turns

    // Calculate radius for swept volume
    r1 = 1/r - 0.2405;
    r2 = sqrt(pow(1/r + 0.2405, 2) + pow(0.5, 2));
    
    // Loop through pointcloud
    for (int i = 0; i < (int)point_cloud_.size(); i++) {
      p = point_cloud_[i];

      // Ignore points which are on the other side of the center of curvature
      if(abs(p.y()) < 1/r){
        // Project car towards point
        // Left turn
        if (cp3_curvature > 0.0) {
          r_dist = sqrt(pow(p.x(), 2) + pow(1/r - p.y(), 2));
          theta = atan2(p.x(), abs(1/r - p.y()));
        }
        // Right turn
        else {
          r_dist = sqrt(pow(p.x(), 2) + pow(-1/r - p.y(), 2));
          theta = atan2(p.x(), abs(-1/r - p.y()));
        }

        // Check if its an obstacle
        if (r_dist >= r1 && r_dist <= r2 && theta > 0) {
          f = 1/r * (theta - atan2(0.5, 1/r - 0.2405));
          
          // Update minimum free path length
          if (f < f_min) {
            f_min = f+0.5;
            theta_new = theta;
          }
        }
      }
    }
  }

  return f_min; //This f_min needs to account for the length of the car and already has the safety margin, add 0.5
}

// Calculate minimum clearance along free path length and curvature.
float Controller::Clearance(std::vector<Eigen::Vector2f> point_cloud_, float curvature, float free_path_length) {
  Vector2f p(0.0, 0.0);
  float c_min = max_clearance - 0.2405;
  
  // Moving in a straight line
  if (abs(curvature) < 0.01) {
    // Loop through pointcloud
    for (int i = 0; i < (int)point_cloud_.size(); i++) {
      p = point_cloud_[i];

      // If obstacle afftects clearance
      if (p.x() > 0 && p.x() < free_path_length+0.5 && abs(p.y()) < max_clearance && abs(p.y()) > 0.2405) {
        c = abs(p.y())-0.2405;

        // Update minimum clearance
        if (c < c_min){
          c_min = c;
        }
      }
    }
  }

  // Moving along an arc
  else {
    r = abs(1 / curvature);  // radius of curvature
    Vector2f cc(0.0, r); // center of Curvature as a point 
    float theta_max = theta_new;  // maximum free path length theta to travel
    float cminp;

    // Loop through pointcloud
    for (int i = 0; i < (int)point_cloud_.size(); i++){
      p = point_cloud_[i];
      cminp = sqrt(pow(cc.x() - p.x(), 2) + pow(cc.y() - p.y(), 2));

      // Project car towards point
      // Left turn
      if (curvature > 0.0) {
        theta = atan2(p.x(), abs(1/abs(curvature) - p.y()));
      }
      // Right turn
      else {
        theta = atan2(p.x(), abs(-1/abs(curvature) - p.y()));
      }

      // Check if point affects clearance
      if (abs(cminp - r) < max_clearance && theta > 0 && theta < theta_max) {
        // Compute clearance
        if (cminp > r) {
          c = cminp - r2;
        }
        else {
          c = r1 - cminp;
        }

        // Update minimum clearance
        if (c < c_min) {
          c_min = abs(c);
        }
      }
    }
  }

  
  return c_min;
}

float Controller::DistanceLeft(std::vector<Eigen::Vector2f> point_cloud_, float curvature, float free_path_length) {
  // Nihar + Eric squad
  // Find end point of curvature
  float distance_left = 0.0;
  Vector2f base_link(0.0, 0.0);
  Vector2f goal(10.0, 0.0);

  //Closest point of approach
  float theta_cpoa = atan(goal.x()/(1/curvature));
  Vector2f cpoa((1/curvature)*sin(theta_cpoa), (1/curvature)-(1/curvature)*cos(theta_cpoa));
  float dist_cpoa = (1/curvature)*theta_cpoa;
  // std::cout << theta_cpoa*180/M_PI << "\t theta \t" << dist_cpoa << "\t" << "dist" << std::endl;

  // for (int i = 0; i < (int)point_cloud_.size(); i++) {
  //   p = point_cloud_[i];
  //   if (curvature > 0.0) {
  //     theta = atan2(p.x(), abs(1/abs(curvature) - p.y()));
  //   }
  //   // Right turn
  //   else {
  //     theta = atan2(p.x(), abs(-1/abs(curvature) - p.y()));
  //   }

    // straight
  if (abs(curvature) < 0.01){
    // Vector2f difference = base_link - goal;
    // distance_left = sqrt(pow(difference.x(), 2) + pow(difference.y(), 2));
    distance_left = 3;
  }

  // turn
  else {
    if (free_path_length >= dist_cpoa){
      distance_left = sqrt(pow(1/curvature, 2)+pow(goal.x(),2))-abs(1/curvature);
    }
    else{
      float psi = free_path_length/abs(1/curvature);
      Vector2f calcpoint(abs(1/curvature)*sin(psi),abs(1/curvature)-abs(1/curvature)*cos(psi));
      distance_left = sqrt(pow(calcpoint.x()-goal.x(),2)+pow(calcpoint.y()-goal.y(),2));
    }
    // Vector2f end;
    // end.x() = free_path_length * (cos(curvature * free_path_length));
    // end.y() = free_path_length * (sin(curvature * free_path_length));
    // Vector2f difference = end - goal;
    // distance_left = sqrt(pow(difference.x(), 2) + pow(difference.y(), 2));
  }
  

  return distance_left;

}

}  // namespace navigation
