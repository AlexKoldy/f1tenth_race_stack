#include <controller/geometric_controller.hpp>

/**
 * @brief Sets the path for the controller.
 * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
 * waypoints corresponding to the path [m]
 */
void GeometricController::set_path(const Eigen::MatrixXf path) {
  this->path_ = path;
}
