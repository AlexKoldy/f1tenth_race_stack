#include <controller/geometric_controller.hpp>

/**
 * @brief Sets the path for the controller.
 * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
 * waypoints corresponding to the path [m]
 */
void GeometricController::set_path(const Eigen::MatrixXf path) {
  this->path_ = path;
}

/**
 * @brief Sets the velocity profile to follow for the controller.
 * @param velocity_profile (const Eigen::VectorXf) 1-by-num_waypoints vector of
 * longitudinal velocities
 */
void GeometricController::set_velocity_profile(
    const Eigen::VectorXf velocity_profile) {
  this->velocity_profile_ = velocity_profile;
}
