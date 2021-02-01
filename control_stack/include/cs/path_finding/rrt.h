#pragma once

#include <math.h>
#include <boost/functional/hash.hpp>

#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <list>

#include "cs/path_finding/path_finder.h"
#include "cs/util/constants.h"
#include "cs/util/visualization.h"
#include "libMultiRobotPlanning/bounded_a_star.hpp"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"



namespace cs {
namespace path_finding {

namespace params {
CONFIG_INT(num_samples, "rrt.num_samples");
CONFIG_FLOAT(cost_bias, "rrt.cost_bias");
CONFIG_FLOAT(path_length, "rrt.path_length");
CONFIG_FLOAT(ped_var_scale, "rrt.ped_var_scale");
CONFIG_FLOAT(ped_var_power, "rrt.ped_var_power");
CONFIG_FLOAT(ped_var_bias, "rrt.ped_var_bias");
CONFIG_FLOAT(robot_radius, "rrt.robot_radius");
CONFIG_FLOAT(collision_buffer, "rrt.collision_buffer");
CONFIG_FLOAT(t_horizon, "rrt.t_horizon");
CONFIG_FLOAT(switch_discount, "rrt.switch_discount");


CONFIG_FLOAT(vel_scale, "rrt.vel_scale");
CONFIG_FLOAT(atan_scale, "rrt.atan_scale");

}  // namespace params

template <int max_samples>
class RRT : public PathFinder {
 private:
  std::vector<Path2f> paths_;

  float get_angle_facing_waypoint(const Eigen::Vector2f& p,
                             const Eigen::Vector2f& waypoint) {
  const Eigen::Vector2f d = waypoint - p;
  return math_util::AngleMod(std::atan2(d.y(), d.x()));
}

  util::Pose get_local_path_pose_(const util::Pose& current_pose,
                            const Eigen::Vector2f& global_waypoint,
                            const util::Pose& goal_pose,
                            const path_finding::Path2f& path) {
  if (path.waypoints.size() <= 1) {
    const auto delta = (global_waypoint - current_pose.tra);
    const float delta_dist = delta.norm();
    const float angle = std::atan2(delta.y(), delta.x());
    return {current_pose.tra + delta.normalized() * (delta_dist * 0.66), angle};
  }

  const Eigen::Vector2f& next_waypoint = path.waypoints[1];
  if (next_waypoint == goal_pose.tra) {
    return goal_pose;
  }

  return {next_waypoint,
           get_angle_facing_waypoint(current_pose.tra, next_waypoint)};
}

  // Calculates euclidian distance from path end point to goal
  float euclid_dist_(const Path2f& path, const util::Pose& goal) {
    return sqrt(pow(path.waypoints.back()[0] - goal.tra.x(), 2) + pow(path.waypoints.back()[1] - goal.tra.y(), 2));
  }

  // Calculates to probability of colliding with a pedestrian along a path
  float find_collision_prob_(const ped_detection::PedDetector& ped_detector, 
     const util::DynamicFeatures& dynamic_features,
                  const motion_planning::PIDController& motion_planner,
                  Path2f& path,
    const Eigen::Vector2f&  start, Eigen::Vector2f  vel,
    const util::Pose& current_pose,
    const Eigen::Vector2f& global_waypoint,
    const util::Pose& goal_pose) {

    (void) start;

    Eigen::Vector2f est_vel;

    est_vel[0] = vel.x();
    est_vel[1] = vel.y();
    
    util::Pose local_waypoint =
      get_local_path_pose_(current_pose, global_waypoint, goal_pose, path);
    util::Twist command = motion_planner.DriveToPose(
      dynamic_features, local_waypoint);
    Eigen::Vector2f new_vel;

    float y_diff = path.waypoints.back().y() - path.waypoints.front().y();

    float x_diff = path.waypoints.back().x() - path.waypoints.front().x();
    float theta = atan(params::CONFIG_atan_scale * y_diff/ x_diff);
    float x_vel = cos(theta) * command.tra.x();
    float y_vel = sin(theta) * command.tra.x();

    if ((x_diff < 0 && x_vel > 0) || (x_diff > 0 && x_vel < 0)) {
      x_vel *= -1;
    }

    if ((y_diff < 0 && y_vel > 0) || (y_diff > 0 && y_vel < 0)) {
      y_vel *= -1;
    }
    new_vel[0] =  params::CONFIG_vel_scale * x_vel;
    new_vel[1] = params::CONFIG_vel_scale * y_vel;


    vel = new_vel;
    path.v0 = util::Twist(vel.x(), vel.y(), command.rot);


    float prob_no_collision = 1;
    for (auto ped: ped_detector.GetPeds().peds) {
      float ped_vel_x = ped.vel.tra.x() + est_vel.x();
    
      // Calculate t_min, the time at which a path will be closest to a pedestrian in the future
      float numerator_x = (ped.pose.tra.x()) * (vel.x() - ped_vel_x);
      float numerator_y = (ped.pose.tra.y()) * (vel.y() - ped.vel.tra.y());
      float numerator = numerator_x + numerator_y;
      float denom = (pow(vel.x() - ped_vel_x, 2) + pow(vel.y() - ped.vel.tra.y(), 2));
      float t_min = numerator / denom;

      if (t_min > params::CONFIG_t_horizon) {
        t_min = params::CONFIG_t_horizon;
      }

      if (t_min < 0) {
        t_min = 0;
      }

      // Calculate params for pedestrian distrubtion (modeled as Gaussian)
      float sigma = sqrt(params::CONFIG_ped_var_bias + params::CONFIG_ped_var_scale * pow(t_min, params::CONFIG_ped_var_power));
      float mean_x = ped.pose.tra.x() + ped_vel_x * t_min;
      float mean_y = ped.pose.tra.y()  + ped.vel.tra.y() * t_min;

      float x_robot = vel.x() * t_min;
      float y_robot = vel.y() * t_min;
      float collision_radius = params::CONFIG_robot_radius + ped.radius + params::CONFIG_collision_buffer;

      // Limits of integration for probability of collision
      float x_lo = x_robot - collision_radius / 2;
      float x_hi = x_robot + collision_radius / 2;
      float y_lo = y_robot - collision_radius / 2;
      float y_hi = y_robot + collision_radius / 2;


      float cdf_hi_x = 0.5 * std::erfc(-(x_hi- mean_x)/(sigma*sqrt(2)));
      float cdf_lo_x = 0.5 * std::erfc(-(x_lo- mean_x)/(sigma*sqrt(2)));
      float p_x = cdf_hi_x - cdf_lo_x;

      float cdf_hi_y = 0.5 * std::erfc(-(y_hi- mean_y)/(sigma*sqrt(2)));
      float cdf_lo_y = 0.5 * std::erfc(-(y_lo- mean_y)/(sigma*sqrt(2)));
      float p_y = cdf_hi_y - cdf_lo_y;
      float prob_single_collision = p_x * p_y;
      
      prob_no_collision *= (1 - prob_single_collision);
    }

    return 1 - prob_no_collision;
  }


  // Calculates cost for a path based on its euclidian distance to the goal and the probability of
  // colliding with a pedestrian along the path
  void calculate_cost_(
    Path2f& path,
    const ped_detection::PedDetector& ped_detector,
    const util::DynamicFeatures& dynamic_features,
    const motion_planning::PIDController& motion_planner,
    const Eigen::Vector2f& start,
    const Eigen::Vector2f& goal,
    const Eigen::Vector2f& vel,
    const util::Pose& current_pose,
    const util::Pose& goal_pose,
    const float worst_dist
    ) {
      float collision_prob = find_collision_prob_(ped_detector, dynamic_features, motion_planner, path, start, vel, 
      current_pose, goal, goal_pose);
      float dist_from_goal = euclid_dist_(path, goal_pose) / worst_dist;
      float cost = dist_from_goal + params::CONFIG_cost_bias * collision_prob;
      path.collision_prob = collision_prob;
      path.dist_from_goal = dist_from_goal;
      path.cost  = cost;
  }
 public:
  explicit RRT(const util::vector_map::VectorMap& map,
                 const float& robot_radius,
                 const float& safety_margin,
                 const float& inflation)
      : PathFinder(map, robot_radius, safety_margin, inflation), paths_() {}

  // Calculates the cost for each sampled path and returns the lowest cost path
  Path2f FindPath(const ped_detection::PedDetector& ped_detector,
                  const util::DynamicFeatures& dynamic_features,
                  const motion_planning::PIDController& motion_planner,
                  const Eigen::Vector2f&  start,
                  const Eigen::Vector2f& goal,
                  const Eigen::Vector2f& est_vel,
                  const util::Pose& current_pose,
                  const util::Pose& goal_pose) {    
    paths_.clear();

    // Sample paths around the clock
    float worst_dist = 0;
    for (int i = 0; i < params::CONFIG_num_samples; i++) {
      float theta = 2 * i * M_PI / params::CONFIG_num_samples;
      const Eigen::Vector2f sample(params::CONFIG_path_length * sin(theta), 
                                   params::CONFIG_path_length * cos(theta));
      Path2f path;
      path.index = i;

      path.waypoints.push_back(start);
      path.waypoints.push_back(start + sample);
      worst_dist = std::max(worst_dist, euclid_dist_(path, goal_pose));
    }

    for (int i = 0; i < params::CONFIG_num_samples; i++) {
      float theta = 2 * i * M_PI / params::CONFIG_num_samples;
      const Eigen::Vector2f sample(params::CONFIG_path_length * sin(theta), 
                                   params::CONFIG_path_length * cos(theta));
      Path2f path;
      path.index = i;

      path.waypoints.push_back(start);
      path.waypoints.push_back(start + sample);
      calculate_cost_(path, ped_detector, dynamic_features, motion_planner, start, goal, est_vel,
        current_pose, goal_pose, worst_dist);
      if (prev_path_.index == i) {
        path.cost *= params::CONFIG_switch_discount;
      }
      paths_.push_back(path);
    }

    // Find the minimum cost path
    Path2f min_cost_path = *std::min_element(begin(paths_), end(paths_),
                                [](const Path2f& a, const Path2f& b){
      return a.cost < b.cost;
    });

    prev_path_ = min_cost_path;
    return min_cost_path;
  }

  std::vector<Path2f> GetCandidatePaths(int num_paths) override {
    std::sort(paths_.begin(), paths_.end(),[](const Path2f& a, const Path2f& b){
      return a.cost < b.cost;
    });
    return std::vector<Path2f>(paths_.begin() + 1, paths_.begin() + num_paths);
  }
};

}  // namespace path_finding
}  // namespace cs