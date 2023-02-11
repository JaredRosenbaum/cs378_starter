//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "controller.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(cp1_distance, 20.0, "Distance to travel for 1D TOC (cp1)");
DEFINE_double(cp1_curvature, 0.5, "Curvature for arc path (cp1)");

DEFINE_double(cp3_curvature, 0.0, "Curvature for arc path (cp3)");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// Controller variables
float timestep = 0.05;
float vCurrent = 0.0;
float distanceTraveled = 0.0;
float controlVelocity;

navigation::Controller TOC;

// PointCloud visualization variable
Vector2f p(0.0, 0.0);

// Obstacle avoidance variables
float curvature_step = 0.1;
float best_score, selected_free_path_length, selected_curvature;
float min_dist, clearance, distance_to_goal;
float w1 = 1; // Weight for Clearance 
float w2 = 1; // Weight for Distance to Goal

// Latency calculations
vector<float> prevCommands{};

} // namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;

  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }

  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // Visualize pointcloud from LiDAR.
  for (int i = 0; i < (int)point_cloud_.size(); i++) {
    p = point_cloud_[i];
    // if (sqrt(pow(p.x(),2)+pow(p.x(),2)) < 10){}
    visualization::DrawCross(p, timestep, 0, local_viz_msg_);
    // cout << p.x() << "\t\t" << p.y() << endl;
  }

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Evaluate path options for obstacle avoidance
  float score;
  float straight_score = 0;
  best_score = -1.0;
  selected_curvature = 0.0;
  float curv;
  for (curv = -1.0; curv <= 1.0; curv += curvature_step) {
    
    // curv = 0.3;


    min_dist = TOC.FreePathLength(point_cloud_, curv);
    clearance = TOC.Clearance(point_cloud_, curv, min_dist);
    distance_to_goal = TOC.DistanceLeft(point_cloud_, curv, min_dist);

    // Calculate and update score
    // TODO Use complete formula
    score = min_dist + w1 * clearance + w2 * min_dist;
    score = min_dist + w1 * clearance;

    //edge case
    if (abs(curv) < 0.01){
      straight_score = score;
    }

    cout << min_dist << "\t\t" << clearance << "\t\t\t  " << score << "\t" << curv << endl;
    if (score >= best_score) {
      best_score = score;
      selected_free_path_length = min_dist;
      selected_curvature = curv;
    }

    if (straight_score == best_score){
      selected_curvature = 0.0;
    }
    // break;
  }

  // TODO Delete
  // min_dist = TOC.FreePathLength(point_cloud_, FLAGS_cp3_curvature);

  // Calculate distance traveled assuming a 0.2s actuation latency
  float distanceLatency = 0.0;

  for (unsigned i = 0; i < prevCommands.size(); i++) {
    distanceLatency += prevCommands[i];
  }
  distanceLatency *= timestep;
  
  // Run the Time Optimal Controller to calculate velocity value
  distanceTraveled = 0;   // car will continue moving forward indefinitely
  controlVelocity = TOC.Run(vCurrent, distanceTraveled + distanceLatency, FLAGS_cp1_distance, selected_free_path_length - distanceLatency);
  vCurrent = robot_vel_.norm();
  distanceTraveled = (odom_loc_ - odom_start_loc_).norm();

  // Set the control values to issue drive commands:
  // TODO Uncomment correct values
  drive_msg_.velocity = controlVelocity;
  drive_msg_.curvature = selected_curvature;
  // drive_msg_.velocity = 0.2;
  // drive_msg_.curvature = 0.3;

  // Keep track of the previous velocity commands for latency compensation
  if(prevCommands.size() == 2) {
      prevCommands.pop_back();
  } 
  prevCommands.insert(prevCommands.begin(), controlVelocity);

  const float PI = 3.1415;
  const uint32_t COLOR = 0x000000;
  const float WIDTH = 0.2405; // width of car + margin

  //Visualize turning radius
  visualization::DrawArc(Vector2f (0, 1/FLAGS_cp3_curvature), 1/abs(FLAGS_cp3_curvature) - WIDTH, 0, 2*PI, COLOR, local_viz_msg_);
  visualization::DrawArc(Vector2f (0, 1/FLAGS_cp3_curvature), sqrt(pow(1/abs(FLAGS_cp3_curvature) + WIDTH, 2) + pow(0.5, 2)), 0, 2*PI, COLOR, local_viz_msg_);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
  ros::spinOnce();
}

}  // namespace navigation
