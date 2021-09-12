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
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
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


// float Navigation::Simple1DTOC(Vector2f point)
// {
//   float x3 = pow(MAX_VELOCITY,2) / MAX_DECEL;
//   Vector2f base_link;
//   base_link[0] = 0;
//   base_link[1] = 0;
//   float distance_left  = abs((point - base_link).norm());

//   // accelerate towards vmax
//   if (robot_vel_[0] < MAX_VELOCITY && distance_left >= x3)
//   {
//     printf("a\n");
//     float new_v = robot_vel_[0] + MAX_ACCEL * 1/20;
//     return std::min(new_v, MAX_VELOCITY);
//   }
  
//   // Cruise
//   if (robot_vel_[0] == MAX_VELOCITY && distance_left >= x3) 
//   {
//     printf("\tc\n");
//     return MAX_VELOCITY;
//   }

//   printf("\t\td\n");
  
//   // Stop
//   float new_v = robot_vel_[0] - MAX_DECEL * 1/20;
//   float zero = 0.0;
//   return std::max(new_v, zero);
// }

float Navigation::getTravellableDistance(float curvature)
{
  float res = INF;
  // Calculate the steering angle from curvature
  float steering_angle = atan(WHEELBASE * curvature);
  for (auto point : point_cloud_)
  {
    float distance = GetMaxDistance(steering_angle, point);
    res = std::min(distance, res);
  }
  return res;
}

float* Navigation::getBestCurvature(){
  float curvature = -1.0;
  float delta_c = 0.1;
  float best_curvature = 0.0;
  float max_dist = 0.0;

  while (curvature <= 1.0)
  {
    float distance = getTravellableDistance(curvature);
    if (distance > max_dist)
    {
      max_dist = distance;
      best_curvature = curvature;
    }
    curvature += delta_c;
  }
  return new float[2] {best_curvature, max_dist};
}

float* Navigation::Simple1DTOC()
{
  float* action = getBestCurvature();
  float curvature = action[0];
  float dist = action[1];
  float x3 = pow(MAX_VELOCITY,2) / MAX_DECEL;

  // accelerate towards vmax
  if (robot_vel_[0] < MAX_VELOCITY && dist >= x3)
  {
    printf("a\n");
    float new_v = robot_vel_[0] + MAX_ACCEL * 1/20;
    return new float[2] {curvature, new_v};  
  }
  
  // Cruise
  if (robot_vel_[0] == MAX_VELOCITY && dist >= x3) 
  {
    printf("\tc\n");
    return new float[2] {curvature, MAX_VELOCITY};
  }

  printf("\t\td\n");
  
  // Stop
  float new_v = robot_vel_[0] - MAX_DECEL * 1/20;
  float zero = 0.0;
  return new float[2] {curvature, std::max(new_v, zero)};
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  float* res = Simple1DTOC();
  drive_msg_.curvature = res[0];
  drive_msg_.velocity = res[1];
  
  // for (auto point : point_cloud_) {
  //   visualization::DrawPoint(point,0x4287f5,local_viz_msg_);
  // }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
