#include <Eigen/Dense>
#include <controller/model_acceleration_pursuit_controller.hpp>
#include <iostream>

/**
 * @brief Default constructor.
 * Initializes the ModelAccelerationPursuitController with no values.
 */
ModelAccelerationPursuitController::ModelAccelerationPursuitController() {}

/**
 * @brief Constructor with parameters.
 * Initializes the ModelAccelerationPursuitController with the given parameters.
 * @param alpha (const float) lookahead distance slope [s]
 * @param beta (const float) lookahead distance bias [m]
 * @param ell_min (const float) minimum lookahead distance [m]
 * @param ell_max (const float) maximum lookahead distance [m]
 */
ModelAccelerationPursuitController::ModelAccelerationPursuitController(
    const float alpha, const float beta, const float ell_min,
    const float ell_max)
    : PurePursuitController(alpha, beta, ell_min, ell_max, 0.0) {}

/**
 * @brief Performs one full controller step, by calculating and setting
 * the lookahead point and returning the steering angle and longitudinal
 * velocity
 * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
 * [m]
 * @param q (const Eigen::Quaternionf) robot orientation as a quaternion
 * @return (std::pair<float, float>) longitudinal velocity [m/s] and steering
 * angle [rad]
 */
std::pair<float, float> ModelAccelerationPursuitController::step(
    const Eigen::Vector2f p, const Eigen::Quaternionf q) {
  // TODO: this is outdated - FIX
  // Calculate the lookahead distance based off of target speed
  float ell = this->calculate_lookahead_distance(0.0);

  // Clip the lookahead distance
  this->clip_lookahead_distance(ell, this->ell_min_, this->ell_max_);

  // Find the closest waypoint to the robot
  int closest_waypoint_index =
      this->find_closest_waypoint_index(p, this->path_);

  // Roll the path such that the closest index is the
  // (0)th element of the path
  this->roll_path(this->path_, closest_waypoint_index);

  // Find the lookahead point
  this->lookahead_point_ = this->calculate_lookahead_point(p, ell, this->path_);

  return std::make_pair(0.0f, 0.0f);
}
