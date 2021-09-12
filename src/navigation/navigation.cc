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

float Navigation::Simple1DTOC(Eigen::Vector2f point)
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
  if (iteration == 0) {
    obstacle = Eigen::Vector2f(-19.3, 8.0);


  }
    visualization::ClearVisualizationMsg(global_viz_msg_);
    visualization::ClearVisualizationMsg(local_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"
  // visualization::DrawPoint(Vector2f(dist, 0),0x4287f5,local_viz_msg_);
  visualization::DrawCross(obstacle,0.5, 0xaa00aa,global_viz_msg_);

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = 0;
  // drive_msg_.velocity = Simple1DTOC(Vector2f(dist, 0));
  drive_msg_.curvature = -(M_PI / 6);
  drive_msg_.velocity = 0.2;

  dist -= drive_msg_.velocity * 1/20;

  if (VISUALIZE) {
    DrawCar();
  }

  // drive_msg_.curvature = - M_PI / 6;
  // drive_msg_.velocity = 1.0;

  // for (auto point : point_cloud_) {
  //   visualization::DrawPoint(point,0x4287f5,local_viz_msg_);
  // }

  // printf("Iteration: %d\n", iteration);

  // visualization::DrawCross(obstacle, 0.1, 0x0000ff,global_viz_msg_);

  
  // GlobalToRobot(obstacle);
  // PointCollidesWithArc(-M_PI / 6, GlobalToRobot(obstacle));
  printf("Collides with point: %lf\n", GetMaxDistance(-(M_PI / 6), GlobalToRobot(obstacle)));
  // PointCollidesWithArc(M_PI / 6, Eigen::Vector2f(2.0,2.0));

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
  iteration++;
}

double Navigation::GetMaxDistanceStraight(Eigen::Vector2f point) {
  double l = LENGTH + SAFETY_MARGIN;
  double w = WIDTH + SAFETY_MARGIN;
  double wb = WHEELBASE;

  Eigen::Vector2f bbox_min (0 - (l - wb)/2, -w / 2.0);
  Eigen::Vector2f bbox_max ((wb + (l - wb)/2) + 5, w / 2);

  if (point(0) >= bbox_min(0) && point(0) <= bbox_max(0) && point(1) >= bbox_min(1) && point(1) <= bbox_max(1)) {
    return (wb + (l - wb)/2) - point(0); 
  }

  return -1;
}

// Assumes collision is already checked
double Navigation::GetMaxDistance(double theta, Eigen::Vector2f point) { 
  if (theta == 0) {
    return GetMaxDistanceStraight(point);
  }

  // just flip calculation if theta is to the right
  if (theta < 0) {
    theta = -theta;
    point(1) = -point(1);
  }

  double width = WIDTH + SAFETY_MARGIN;
  double length = LENGTH + SAFETY_MARGIN;
  double wheelbase = WHEELBASE;

  double radius = wheelbase / tan(theta/2);
  Vector2f CoT(0,radius);

  double max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
  double min_radius = radius - width/2.0;
  
  auto point_dis = point - CoT;
  double point_radius = abs(point_dis.norm());

  if (point_radius >= min_radius && point_radius <= max_radius) {
    // Collision with this point
    double inner_corner_radius = sqrt(pow(min_radius,2) + pow(length - (length - wheelbase)/2.0,2));
    Vector2f collision_point;
    if (point_radius >= min_radius && point_radius <= inner_corner_radius) {
      // collision with side
      printf("Collides with front\n");
      collision_point[1] = width/2.0;
      collision_point[0] = sqrt(pow(point_radius,2) - pow(min_radius,2));
    } else {
      // collision with front
      printf("Collides with side\n");
      collision_point[1] = sqrt(pow(point_radius,2) - pow(length - (length - wheelbase)/2,2)) - radius;
      collision_point[0] = length - (length - wheelbase)/2;
    }

    // double distance = (point - collision_point).norm();
    auto v1 = CoT - collision_point;
    auto v2 = CoT - point;
    double signed_angle = atan2(v2(1),v2(0)) - atan2(v1(1),v1(0));
    if (signed_angle > 0) {
      double collision_angle = 2 * asin((point - collision_point).norm() / (2 * point_radius));
      double max_distance = point_radius * collision_angle;
      return max_distance;
    }
  }
  return -1;
}

void Navigation::DrawCar() {
  double l = LENGTH + SAFETY_MARGIN;
  double w = WIDTH + SAFETY_MARGIN;
  double wb = WHEELBASE;

  Eigen::Vector2f bbox_min (0 - (l - wb)/2, -w / 2.0);
  Eigen::Vector2f bbox_max ((wb + (l - wb)/2), w / 2);
  visualization::DrawLine(bbox_min, Eigen::Vector2f(bbox_min(0),bbox_max(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_min, Eigen::Vector2f(bbox_max(0),bbox_min(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_max, Eigen::Vector2f(bbox_min(0),bbox_max(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_max, Eigen::Vector2f(bbox_max(0),bbox_min(1)),0x000000,local_viz_msg_);
}

Eigen::Vector2f Navigation::GlobalToRobot(Eigen::Vector2f point) {

  // printf("Robot location: %f %f\n", robot_loc_(0), robot_loc_(1));

  float angle = -robot_angle_;
  Eigen::Matrix2f rot;
  rot(0,0) = cos(angle);
  rot(0,1) = -sin(angle);
  rot(1,0) = sin(angle);
  rot(1,1) = cos(angle);

  auto translated_point = rot * (point - robot_loc_);

  visualization::DrawCross(translated_point, 0.5, 0x0000ff,local_viz_msg_);

  return translated_point;

}


}  // namespace navigation
