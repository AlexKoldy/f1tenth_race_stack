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
   * @brief Sets the velocity profile to follow for the controller.
   * @param velocity_profile (const Eigen::VectorXf) 1-by-num_waypoints vector
   * of longitudinal velocities
   */
  void set_velocity_profile(const Eigen::VectorXf velocity_profile);

  /**
   * @brief Sets the parameters for the controller.
   * @param alpha (const float) lookahead distance slope [s]
   * @param beta (const float) lookahead distance bias [m]
   * @param ell_min (const float) minimum lookahead distance [m]
   * @param ell_max (const float) maximum lookahead distance [m]
   * @param k_p (const float) steering gain [m]
   */
  virtual void set_parameters(const float alpha, const float beta,
                              const float ell_min, const float ell_max,
                              const float k_p) = 0;

  /**
   * @brief Returns the lookahead point of the controller
   * @return (Eigen::Vector2f) (x,y)-position of lookahead point [m]
   */
  virtual Eigen::Vector2f get_lookahead_point() = 0;

  virtual int get_closest_waypoint_index() = 0;

  /**
   * @brief Performs one full controller step, by calculating and setting
   * the lookahead point and returning the steering angle and longitudinal
   * velocity.
   * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
   * [m]
   * @param q (const Eigen::Quaternionf) robot orientation as a quaternion
   * @return (std::pair<float, float>) longitudinal velocity [m/s] and steering
   * angle [rad]
   */
  virtual std::pair<float, float> step(const Eigen::Vector2f p,
                                       const Eigen::Quaternionf q) = 0;

  /**
   * @brief Default destructor.
   */
  virtual ~GeometricController() {}

 protected:
  // Path to follow and velocity profile
  Eigen::MatrixXf path_;
  Eigen::VectorXf velocity_profile_;

 private:
};

#endif  // GEOMETRIC_CONTROLLER_HPP