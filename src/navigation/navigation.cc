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

float dist = 10.0;

float Navigation::Simple1DTOC(Vector2f point)
{
  float v_max = 1.0;
  float max_a = 0.4;
  float max_d = 0.4;
  float x3 = pow(v_max,2) / max_d;
  Vector2f base_link;
  base_link[0] = 0;
  base_link[1] = 0;
  float distance_left  = abs((point - base_link).norm());

  // accelerate towards vmax
  if (robot_vel_[0] < v_max && distance_left > x3)
  {
    printf("a\n");
    float new_v = robot_vel_[0] + max_a * 1/20;
    return std::min(new_v, v_max);
  }
  
  // Cruise
  if (robot_vel_[0] == v_max && distance_left > x3) 
  {
    printf("\tc\n");
    return v_max;
  }

  printf("\t\td\n");
  
  // Stop
  float new_v = robot_vel_[0] - max_d * 1/20;
  float zero = 0.0;
  return std::max(new_v, zero);
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
  visualization::DrawPoint(Vector2f(dist, 0),0x4287f5,local_viz_msg_);

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = 0;
  drive_msg_.velocity = Simple1DTOC(Vector2f(dist, 0));
  dist -= drive_msg_.velocity * 1/20;

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

// float Navigation::GetMaxDistance(float theta, Vector2f point) { 
//   float radius = width / tan(theta));
//   Vector2f CoT(0,radius);

//   float max_radius = sqrt(pow((radius+width)/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
//   float min_radius = radius - width/2.0;

//   float point_radius = abs(point - CoT);

//   if (point_radius >= min_radius && point_radius <= max_radius) {
//     // Collision with this point
//     float inner_corner_radius = sqrt(pow(min_radius,2) + pow(length - (length - wheelbase)/2.0,2));
//     Vector2f collision_point;
//     if (point_radius >= min_radius && point_radius <= inner_corner_radius) {
//       // collision with side
//       collision_point[1] = width/2.0;
//       collision_point[0] = sqrt(pow(point_radius,2) - pow(min_radius,2));

//     } else {
//       // collision with front
//       collision_point[1] = sqrt(pow(point_radius,2)- pow(length - (length - wheelbase)/2),2) - radius;
//       collision_point[0] = length - (length - wheelbase)/2
//     }

//     float collision_angle = 2 * asin(abs(point - collision_point) / (2 * point_radius));
//     float max_distance = point_radius * collision_angle;
//     return max_distance;

//   }
// }


}  // namespace navigation
