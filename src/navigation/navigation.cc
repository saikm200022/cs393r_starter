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

bool fEquals (float a, float b) {
  return (a >= b - kEpsilon && a <= b + kEpsilon);
}

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
                                   float time) {
  point_cloud_ = cloud;                                     
}

float Navigation::getTravellableDistance(struct PathOption& option)
{
  float curvature = option.curvature;
  float res = INF;
  // printf("Curve: %f\n", curvature);
  // Calculate the steering angle from curvature
  option.theta = atan(WHEELBASE * curvature);
  for (auto point : point_cloud_)
  {
    float distance = GetMaxDistance(option, point);
    res = std::min(distance, res);
  }

  // printf("Best distance: %f\n", res);

  return res;
}

float* Navigation::getBestCurvature() {
  float curvature = -1.0;
  float delta_c = 0.5;
  float best_curvature = 0.0;
  float max_dist = 0.0;

  while (curvature <= 1.0)
  {
    struct PathOption option;
    option.curvature = curvature;

    // printf("Curvature: %f\n", curvature);
    option.free_path_length = getTravellableDistance(option);
    if (option.free_path_length > max_dist)
    {
      max_dist = option.free_path_length;
      best_curvature = curvature;
    } else {
      if (fEquals(option.free_path_length, max_dist)) {
       if (abs(best_curvature) > abs(curvature)) {
         // Favor curvatures close to straight
         best_curvature = curvature;
       }
      }
    }
    curvature += delta_c;
  }
  // printf("Max distance: %f\n", max_dist);

  return new float[2] {best_curvature, max_dist};
}

float* Navigation::Simple1DTOC()
{
  float* action = getBestCurvature();
  float curvature = action[0];
  float dist = action[1];

  // printf("DISTANCE: %f\n", dist);

  if (VISUALIZE) {
    DrawCar();
    DrawArcs(curvature, dist);
  }

  float x3 = pow(robot_vel_[0],2) / MAX_DECEL;

  // accelerate towards vmax
  if (robot_vel_[0] < MAX_VELOCITY && dist >= x3)
  {
    // printf("a\n");
    float new_v = robot_vel_[0] + MAX_ACCEL * 1/20;
    return new float[2] {curvature, new_v};  
  }
  
  // Cruise
  if (robot_vel_[0] == MAX_VELOCITY && dist >= x3) 
  {
    // printf("\tc\n");
    return new float[2] {curvature, MAX_VELOCITY};
  }

  // printf("\t\td\n");
  
  // Stop
  float new_v = robot_vel_[0] - MAX_DECEL * 1/20;
  float zero = 0.0;
  return new float[2] {curvature, std::max(new_v, zero)};
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  if (iteration == 0) {
    obstacle = Eigen::Vector2f(-19.3, 8.0);
  }

    visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // if (iteration % 100 == 0) {
    visualization::ClearVisualizationMsg(local_viz_msg_);
    float* res = Simple1DTOC();
    drive_msg_.curvature = res[0];
    drive_msg_.velocity = res[1];
  // } else {
  //       drive_msg_.curvature = 0;
  //   drive_msg_.velocity = 0;
  // }
  
  // int size = point_cloud_.size();
  // printf("Cloud size: %d\n", size);
  for (auto point : point_cloud_) {
    visualization::DrawPoint(point,0x4287f5,local_viz_msg_);
  }

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

void Navigation::GetClearance (struct PathOption& option) {
  float width = WIDTH + SAFETY_MARGIN;
  float length = LENGTH + SAFETY_MARGIN;
  float wheelbase = WHEELBASE;

  float radius = option.radius;
  Vector2f CoT(0,radius);

  float max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
  float min_radius = radius - width/2.0;

  float min_clearance = INF;

  for (auto point : point_cloud_) {
    Eigen::Vector2f point_vector = point - CoT;
    float point_radius = point_vector.norm();

    if (point_radius >= min_radius && point_radius <= max_radius) {
      // collides - we don't care about this one
      continue;
    }

    // See if point is at a reasonable angle
    Eigen::Vector2f car_vector (0, -radius);
    float point_angle = GetAngleBetweenVectors(car_vector, point_vector);
    float distance_angle = option.free_path_length / radius;

    if (point_angle < distance_angle) {

      // point is close to arc that is travelled
      min_clearance = std::min(min_clearance, std::min(abs(point_radius - min_radius), abs(point_radius - max_radius)));
    } 
    else {
      // printf("Clearance set to outward\n");
      // point is outside, look at endpoints instead
      // Eigen::Vector2f distance_point = 2 * radius * sin(distance_angle/2);
      // Eigen::Vector2f distance_vector = (distance_point - CoT).normalized();

      // Eigen::Vector2f min_endpoint = distance_point - (distance_vector * (radius - min_radius));
      // Eigen::Vector2f max_endpoint = distance_point + (distance_vector * (max_radius - radius));

      // float min_clearance = std::min(min_clearance, std::min((min_endpoint - point).norm(), (max_endpoint - point).norm()));
    }
  }

  option.clearance = min_clearance;
}

float Navigation::GetMaxDistanceStraight(Eigen::Vector2f point) {
  // printf("in straight");

  float l = LENGTH + SAFETY_MARGIN;
  float w = WIDTH + SAFETY_MARGIN;
  float wb = WHEELBASE;

  float car_front = wb + (l - wb)/2;

  Eigen::Vector2f bbox_min (car_front, -w / 2.0);
  Eigen::Vector2f bbox_max (INF, w/2);

  if (point(0) >= bbox_min(0) && point(0) <= bbox_max(0) && point(1) >= bbox_min(1) && point(1) <= bbox_max(1)) { 
    float distance = point(0) - car_front;
    // printf("max distance straight: %f\n", distance);
    return distance; 
  }
  // printf("max distance straight: %f\n", INF);

  return INF;
}
float Navigation::GetMaxDistance(struct PathOption& option, Eigen::Vector2f point) { 
  float theta = option.theta;

  if (fEquals(theta, 0.0)) {
    return GetMaxDistanceStraight(point);
  }

  // just flip calculation if theta is to the right
  if (theta < 0) {
    theta = -theta;
    point(1) = -point(1);
  }

  float width = WIDTH + SAFETY_MARGIN;
  float length = LENGTH + SAFETY_MARGIN;
  float wheelbase = WHEELBASE;

  float radius = wheelbase / tan(theta/2);
  Vector2f CoT(0,radius);

  if(option.theta < 0) {
    option.CoT = Eigen::Vector2f(0, -radius);
  } else {
    option.CoT = CoT;
  }
  option.radius = radius;

  float max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
  float min_radius = radius - width/2.0;
  
  auto point_vec = point - CoT;
  float point_radius = abs(point_vec.norm());

  if (point_radius >= min_radius && point_radius <= max_radius) {
    auto down_vec = Eigen::Vector2f(0, -radius);
    float ab = GetAngleBetweenVectors(down_vec, point_vec);

    float inner_corner_radius = sqrt(pow(min_radius,2) + pow(length - (length - wheelbase)/2.0,2));
    float beta = 0;
    if (point_radius >= min_radius && point_radius <= inner_corner_radius) {
      // collision with side
      float y = radius - width/2;
      beta = acos(y / point_radius);

    } else {
      // collision with front
      float x = length - ((length - wheelbase)/2.0);
      beta = asin(x / point_radius);
    }

    float alpha = ab - beta;
    return radius * alpha;
  }

  return M_PI*2 * radius;
}

float Navigation::GetAngleBetweenVectors (Eigen::Vector2f a, Eigen::Vector2f b) {
  return acos(a.dot(b) / (a.norm() * b.norm()));
}


// float Navigation::GetMaxDistance2(struct PathOption& option, Eigen::Vector2f point) { 
//   float theta = option.theta;

//   if (fEquals(theta, 0.0)) {
//     return GetMaxDistanceStraight(point);
//   }

//   // just flip calculation if theta is to the right
//   if (theta < 0) {
//     theta = -theta;
//     point(1) = -point(1);
//   }

//   float width = WIDTH + SAFETY_MARGIN;
//   float length = LENGTH + SAFETY_MARGIN;
//   float wheelbase = WHEELBASE;

//   float radius = wheelbase / tan(theta/2);
//   Vector2f CoT(0,radius);

//   if(option.theta < 0) {
//     option.CoT = Eigen::Vector2f(0, -radius);
//   } else {
//     option.CoT = CoT;
//   }
//   option.radius = radius;

//   float max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
//   float min_radius = radius - width/2.0;
  
//   auto point_dis = point - CoT;
//   float point_radius = abs(point_dis.norm());

//   if (point_radius >= min_radius && point_radius <= max_radius) {
//     // Collision with this point
//     float inner_corner_radius = sqrt(pow(min_radius,2) + pow(length - (length - wheelbase)/2.0,2));
//     Vector2f collision_point;
//     if (point_radius >= min_radius && point_radius <= inner_corner_radius) {
//       // collision with side
//       // printf("Collision with side\n");
//       collision_point[1] = width/2.0;
//       collision_point[0] = sqrt(pow(point_radius,2) - pow(min_radius,2));
//     } else {
//       // collision with front
//       // printf("Collision with front\n");
//       collision_point[1] = sqrt(pow(point_radius,2) - pow(length - (length - wheelbase)/2,2)) - radius;
//       collision_point[0] = length - (length - wheelbase)/2;
//     }

//     // float distance = (point - collision_point).norm();
//     // auto v1 = CoT - collision_point;
//     // auto v2 = CoT - point;
//     // float signed_angle = atan2(v2(1),v2(0)) - atan2(v1(1),v1(0));
//     // if (signed_angle > 0) {
//       float collision_angle = 2 * asin((point - collision_point).norm() / (2 * point_radius));
//       float max_distance = point_radius * collision_angle;
//       return max_distance;
//     // }
//     // else {
//     //   float collision_angle = 2 * asin((point - collision_point).norm() / (2 * point_radius));
//     //   float max_distance = point_radius * collision_angle;
//     //   return (M_PI * radius * 2) - max_distance;
//     // }
//   }
//   return M_PI * radius;
// }

void Navigation::DrawCar() {
  float l = LENGTH + SAFETY_MARGIN;
  float w = WIDTH + SAFETY_MARGIN;
  float wb = WHEELBASE;

  Eigen::Vector2f bbox_min (0 - (l - wb)/2, -w / 2.0);
  Eigen::Vector2f bbox_max ((wb + (l - wb)/2), w / 2);
  visualization::DrawLine(bbox_min, Eigen::Vector2f(bbox_min(0),bbox_max(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_min, Eigen::Vector2f(bbox_max(0),bbox_min(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_max, Eigen::Vector2f(bbox_min(0),bbox_max(1)),0x000000,local_viz_msg_);
  visualization::DrawLine(bbox_max, Eigen::Vector2f(bbox_max(0),bbox_min(1)),0x000000,local_viz_msg_);
}

void Navigation::DrawArcs(float curvature, float dist) {
  if (fEquals(curvature, 0.0)) {
    visualization::DrawLine(Eigen::Vector2f(CAR_FRONT,0), Eigen::Vector2f(dist+CAR_FRONT,0), 0xff0000, local_viz_msg_);
    return;
  }
  if (curvature > 0) {
    float theta = atan(WHEELBASE * curvature);
    // float theta = curvature;
    float radius = WHEELBASE / tan(theta/2);
    Vector2f CoT(0,radius);
    // visualization::DrawArc(CoT, radius, -M_PI / 2, 0, 0xff0000, local_viz_msg_);
    visualization::DrawArc(CoT, radius, -M_PI / 2, -M_PI / 2 + (dist/radius), 0xff0000, local_viz_msg_);


  } else {
      float theta = atan(WHEELBASE * -curvature);
      // float theta = -curvature;
      float radius = WHEELBASE / tan(theta/2);
      Vector2f CoT(0,-radius);
      // visualization::DrawArc(CoT, radius, -(dist/radius), 0, 0xff0000, local_viz_msg_);
      // visualization::DrawArc(CoT, radius, 0, (dist/radius), 0xff0000, local_viz_msg_);
      visualization::DrawArc(CoT, radius, (M_PI / 2) - (dist/radius), M_PI / 2, 0xff0000, local_viz_msg_);
  }
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
