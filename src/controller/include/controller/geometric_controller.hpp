#include <Eigen/Dense>
#include <memory>

#ifndef GEOMETRIC_CONTROLLER_HPP
#define GEOMETRIC_CONTROLLER_HPP

class GeometricController {
 public:
  /**
   * @brief Sets the path for the controller
   * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
   * waypoints corresponding to the path [m]
   */
  void set_path(const Eigen::MatrixXf path);

  /**
   * @brief Returns the lookahead point of the controller
   * @return (Eigen::Vector2f) (x,y)-position of lookahead point [m]
   */
  virtual Eigen::Vector2f get_lookahead_point() = 0;

  /**
   * @brief Performs one full controller step, by calculating and setting
   * the lookahead point and returning the steering angle
   * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
   * [m]
   * @param q (const Eigen::Quaternionf) robot orientation as a quaternion
   * @param v_x (const float) commanded robot velocity in the robot frame [m/s]
   * @return (float) steering angle [rad]
   */
  virtual float step(const Eigen::Vector2f p, const Eigen::Quaternionf q,
                     const float v_x) = 0;

  /**
   * @brief Default destructor.
   */
  virtual ~GeometricController() {}

 protected:
  // Path to follow
  Eigen::MatrixXf path_;

 private:
};

#endif  // GEOMETRIC_CONTROLLER_HPP