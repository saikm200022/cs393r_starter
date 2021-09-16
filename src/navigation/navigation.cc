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

  option.free_path_length = res;
  return res;
}

void Navigation::scorePath(struct PathOption& option) {
  getTravellableDistance(option);
  TrimDistanceToGoal(option);
  GetClearance(option);

  // printf("Path: length=%f, clearance=%f, goal distance=%f\n", option.free_path_length, option.clearance, option.distance_to_goal);
  
  option.score = option.free_path_length + CLEARANCE_WEIGHT * option.clearance + GOAL_WEIGHT * option.distance_to_goal;
}

float* Navigation::getBestCurvature() {
  float curvature = -1.0;
  float delta_c = 0.1;
  float best_curvature = 0.0;
  float dist = 0.0;
  float best_score = 0;

  while (curvature <= 1.0)
  {
    struct PathOption option;
    option.curvature = curvature;

    // printf("Curvature: %f\n", curvature);
    scorePath(option);

    if (option.score > best_score) {
      best_curvature = option.curvature;
      best_score = option.score;
      dist = option.free_path_length;
    }

    curvature += delta_c;
  }
  // printf("Max distance: %f\n", max_dist);

  return new float[2] {best_curvature, dist};
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

  // printf("Curvature %f\tDist %f\tx3 %f\t\n", curvature, dist, x3);

  // accelerate towards vmax
  if (robot_vel_[0] < MAX_VELOCITY && dist >= x3)
  {
    // printf("a\n");
    float new_v = robot_vel_[0] + MAX_ACCEL * 1/20;
    return new float[2] {curvature, std::max(float(0.2), new_v)};  
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

void Navigation::TrimDistanceToGoal (struct PathOption& option) {
  // Take a maximum free path length and trim it to closest point of approach

  // special case for straight
  if (fEquals(option.theta,0)) {
    float new_max_dist = GOAL(0);
    option.free_path_length = std::min(new_max_dist, option.free_path_length);
    option.distance_to_goal = GOAL(0) - option.free_path_length;
  } else {
    // Gets the angle between current pos and goal
    float phi = GetAngleBetweenVectors(-option.CoT, GOAL - option.CoT);
    float new_max_dist = phi * option.radius;

    option.free_path_length = std::min(new_max_dist, option.free_path_length);
    option.distance_to_goal = abs((GOAL - option.CoT).norm() - option.radius);
  }
}

float Navigation::GetRotation(float velocity, float curvature) {
  float theta = atan(WHEELBASE * curvature);
  float radius = WHEELBASE / tan(theta/2);
  Vector2f CoT(0,radius);

  float distance_travelled = velocity * LATENCY;
  float distance_angle = distance_travelled / radius;

  return (M_PI / 2) - distance_angle;
}

Eigen::Vector2f Navigation::GetTranslation(float velocity, float curvature) {
  float theta = atan(WHEELBASE * curvature);
  float radius = WHEELBASE / tan(theta/2);
  Vector2f CoT(0,radius);

  float distance_travelled = velocity * LATENCY;
  float distance_angle = distance_travelled / radius;
  // Eigen::Vector2f A = CoT;
  Eigen::Vector2f B(0,0);

  float ac = radius;
  float ab = radius;
  float bc = 2 * radius * sin(distance_angle/2.0);

  float c_y = (pow(ab,2) + pow(ac,2) - pow(bc,2)) / (2 * ab);
  float c_x = sqrt(pow(ac,2) - pow(c_y,2));

  return Eigen::Vector2f(c_x, c_y);
}

void Navigation::GetClearance (struct PathOption& option) {
  float width = WIDTH + SAFETY_MARGIN * 2;
  float length = LENGTH + SAFETY_MARGIN * 2;
  float wheelbase = WHEELBASE;

  float radius = option.radius;
  Vector2f CoT(0,radius);

  // min and max radius of swept volume
  float max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
  float min_radius = radius - width/2.0;

  float min_clearance = INF;

  // The point closest to car defines the min clearance
  for (auto point : point_cloud_) {
    Eigen::Vector2f point_vector = point - CoT;
    float point_radius = point_vector.norm();
    
    // See if point is at a reasonable angle
    Eigen::Vector2f car_vector (0, -radius);
    float point_angle = GetAngleBetweenVectors(car_vector, point_vector);
    float distance_angle = option.free_path_length / radius;

    // If the point is outside of the distance traversed by car, we don't care
    if (point_angle < distance_angle) {

      if (point_radius >= min_radius && point_radius <= max_radius) {
        // collides - we don't care about this one
        // also shouldn't happen, but might if car is overshooting
        min_clearance = 0;
        continue;
      }


        // Figure out if point is on inside or outside of swept volume 
        float inside_distance = abs(point_radius - min_radius);
        float outside_distance = abs(point_radius - max_radius);

        // point is close to arc that is travelled
        min_clearance = std::min(min_clearance, std::min(inside_distance, outside_distance));
    } else {
      float angle_difference = point_angle - distance_angle;
      float inside_distance = sqrt(pow(radius,2) + pow(min_radius,2) - 2 * radius * min_radius * cos(angle_difference));
      float outside_distance = sqrt(pow(radius,2) + pow(max_radius,2) - 2 * radius * max_radius * cos(angle_difference));
      min_clearance = std::min(min_clearance, std::min(inside_distance, outside_distance));
    }
  }

  option.clearance = min_clearance;
}

float Navigation::GetMaxDistanceStraight(Eigen::Vector2f point) {
  // printf("in straight");

  float l = LENGTH + SAFETY_MARGIN * 2;
  float w = WIDTH + SAFETY_MARGIN * 2;
  float wb = WHEELBASE;

  float car_front = wb + (l - wb)/2;

  // Get the bounding box of swept volume of car
  Eigen::Vector2f bbox_min (0, -w / 2.0);
  Eigen::Vector2f bbox_max (INF, w/2);

  // Check if point is within bbox
  if (point(0) >= bbox_min(0) && point(0) <= bbox_max(0) && point(1) >= bbox_min(1) && point(1) <= bbox_max(1)) { 
    // distance between point and car front is the max free path
    float distance = point(0) - car_front;

    // Sometimes we end up with negative distance if car overshot and point is within safety margin
    return distance; 
  }

  // Point does not collide - return infinity (maybe this is bad)
  return INF;
}
float Navigation::GetMaxDistance(struct PathOption& option, Eigen::Vector2f point) { 
  float theta = option.theta;

  // If angle is 0, use straight special case
  if (fEquals(theta, 0.0)) {
    return GetMaxDistanceStraight(point);
  }

  // just flip calculation if theta is to the right
  if (theta < 0) {
    theta = -theta;
    point(1) = -point(1);
  }

  float width = WIDTH + SAFETY_MARGIN * 2;
  float length = LENGTH + SAFETY_MARGIN * 2;
  float wheelbase = WHEELBASE;

  float radius = wheelbase / tan(theta/2);
  Vector2f CoT(0,radius);

  // hacky, saving values for later use
  if(option.theta < 0) {
    option.CoT = Eigen::Vector2f(0, -radius);
  } else {
    option.CoT = CoT;
  }
  option.radius = radius;

  // Get the min and max radius of the swept volume of car
  float max_radius = sqrt(pow(radius+width/2.0,2) + pow(length - (length - wheelbase)/2.0, 2));
  float min_radius = radius - width/2.0;
  
  // get the radius between obstacle and CoT
  auto point_vec = point - CoT;
  float point_radius = abs(point_vec.norm());

  // If point_radius is within swept volume, means it collides
  if (point_radius >= min_radius && point_radius <= max_radius) {
    // Get alpha + beta
    // alpha = angle between current base_link and base_link after driving max length
    // beta = angle between base_line and point on car that collides
    // alpha + beta can be easily found by getting angle between base_link and point
    auto down_vec = Eigen::Vector2f(0, -radius);
    float ab = GetAngleBetweenVectors(down_vec, point_vec);

    // Use inner corner radius to determine whether collision is with front or side
    float inner_corner_radius = sqrt(pow(min_radius,2) + pow(length - (length - wheelbase)/2.0,2));
    float beta = 0;

    // Use trig to get angles - don't bother calculating actual points (from class)
    if (point_radius >= min_radius && point_radius <= inner_corner_radius) {
      // collision with side
      float y = radius - width/2;
      beta = acos(y / point_radius);

    } else {
      // collision with front
      float x = length - ((length - wheelbase)/2.0);
      beta = asin(x / point_radius);
    }

    // Alpha is angle that we travel
    float alpha = ab - beta;
    option.angle_travelled = alpha;
    return radius * alpha;
  }
  option.angle_travelled = M_PI * 2;
  // If no collision, return a quarter of the circle
  return M_PI*2 * radius / 4;
}

float Navigation::GetAngleBetweenVectors (Eigen::Vector2f a, Eigen::Vector2f b) {
  float angle = acos(a.dot(b) / (a.norm() * b.norm()));
  if (angle < 0) {
    angle = (M_PI * 2) + angle;
  }
  return angle;
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
  float l = LENGTH + SAFETY_MARGIN * 2;
  float w = WIDTH + SAFETY_MARGIN * 2;
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
    // these might be slightly off but you get the gist
    float theta = atan(WHEELBASE * curvature);
    float radius = WHEELBASE / tan(theta/2);
    Vector2f CoT(0,radius);
    visualization::DrawArc(CoT, radius, -M_PI / 2, -M_PI / 2 + (dist/radius), 0xff0000, local_viz_msg_);


  } else {
      float theta = atan(WHEELBASE * -curvature);
      float radius = WHEELBASE / tan(theta/2);
      Vector2f CoT(0,-radius);
      visualization::DrawArc(CoT, radius, (M_PI / 2) - (dist/radius), M_PI / 2, 0xff0000, local_viz_msg_);
  }
}

Eigen::Vector2f Navigation::GlobalToRobot(Eigen::Vector2f point) {

  // Translate global frame coords to robot coords

  float angle = -robot_angle_;
  Eigen::Matrix2f rot;
  rot(0,0) = cos(angle);
  rot(0,1) = -sin(angle);
  rot(1,0) = sin(angle);
  rot(1,1) = cos(angle);

  auto translated_point = rot * (point - robot_loc_);

  // visualization::DrawCross(translated_point, 0.5, 0x0000ff,local_viz_msg_);

  return translated_point;

}


}  // namespace navigation
