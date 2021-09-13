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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include <math.h>

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float theta;
  float clearance;
  float radius;
  Eigen::Vector2f CoT;
  float free_path_length;
  float distance_to_goal;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

 private:

  float* Simple1DTOC();
  float* getBestCurvature();
  float getTravellableDistance(struct PathOption& option);

  double GetMaxDistance(struct PathOption& option, Eigen::Vector2f point);

  // bool PointCollidesWithArc(double theta, Eigen::Vector2f point);

  // bool PointCollidesStraight(Eigen::Vector2f point);

  double GetMaxDistanceStraight(Eigen::Vector2f point);
  float Simple1DTOC(Eigen::Vector2f point);
  float getDistanceToGoal(struct PathOption& option);


  void DrawCar();
  void DrawArcs(double theta);


  Eigen::Vector2f GlobalToRobot(Eigen::Vector2f point);


  // REAL CAR CONSTANTS
  // const double LENGTH = 0.5;
  // const double WIDTH = 0.25;
  // const double WHEELBASE = 0.35;
  // const double TRACK = 0.25;
  // const double SAFETY_MARGIN = 0.25;

  // SIMULATOR CONSTANTS
  const double LENGTH = 0.535;
  const double WIDTH = 0.281;
  const double WHEELBASE = 0.535;
  const double TRACK = 0.281;
  const double SAFETY_MARGIN = 0.25;

  const float MAX_VELOCITY = 1.0;
  const float MAX_ACCEL = 0.4;
  const float MAX_DECEL = 0.4;

  const float INF = std::numeric_limits<float>::max();


  // const double GOAL = 5.0;
  Eigen::Vector2f GOAL = Eigen::Vector2f(5, 0);

  bool VISUALIZE = 1;
  
  int iteration = 0;

  Eigen::Vector2f obstacle; 

  
  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
