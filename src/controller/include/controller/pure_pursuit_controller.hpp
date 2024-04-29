#include <Eigen/Dense>
#include <controller/geometric_controller.hpp>

#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

class PurePursuitController : public GeometricController {
 public:
  /**
   * @brief Default constructor.
   * Initializes the PurePursuitController with no values.
   */
  PurePursuitController();

  /**
   * @brief Constructor with parameters.
   * Initializes the PurePursuitController with the given
   * parameters.
   * @param alpha (const float) lookahead distance slope [s]
   * @param beta (const float) lookahead distance bias [m]
   * @param ell_min (const float) minimum lookahead distance [m]
   * @param ell_max (const float) maximum lookahead distance [m]
   * @param k_p (const float) steering gain [m]
   */
  PurePursuitController(const float alpha, const float beta,
                        const float ell_min, const float ell_max,
                        const float k_p);

  /**
   * @brief Sets the parameters for the controller.
   * @param alpha (const float) lookahead distance slope [s]
   * @param beta (const float) lookahead distance bias [m]
   * @param ell_min (const float) minimum lookahead distance [m]
   * @param ell_max (const float) maximum lookahead distance [m]
   * @param k_p (const float) steering gain [m]
   */
  void set_parameters(const float alpha, const float beta, const float ell_min,
                      const float ell_max, const float k_p) override;
  /**
   * @brief Returns the lookahead point of the controller.
   * @return (Eigen::Vector2f) (x,y)-position of lookahead point [m]
   */
  Eigen::Vector2f get_lookahead_point() override;

  int get_closest_waypoint_index() override;

  /**
   * @brief TODO
   * @return
   */
  int get_closest_point_index();

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
  virtual std::pair<float, float> step(const Eigen::Vector2f p,
                                       const Eigen::Quaternionf q) override;

 protected:
  // Lookahead parameters
  float alpha_;                      // lookahead distance slope [s]
  float beta_;                       // lookahead distance offset [m]
  float ell_min_;                    // minimum lookahead distance [m]
  float ell_max_;                    // maximum lookahead distance [m]
  Eigen::Vector2f lookahead_point_;  // (x, y)-position of lookahead point [m]

  // Steering angle parameters
  float k_p_;  // steering gain [m]

  /**
   * @brief Calculates the raw lookahead distance based off of linear
   * relationship with velocity: ell = alpha * v_x + beta.
   * @param v_x (const float) commanded robot velocity in the robot frame
   * [m/s]
   * @return (float) raw lookahead distance [m]
   */
  float calculate_lookahead_distance(const float v_x);

  /**
   * @brief Clips the lookahead distance between some minimum and maximum value.
   * @param ell (float&) lookahead distance to be clipped [m]
   * @param ell_min (const float) minimum lookahead distance [m]
   * @param ell_max (const float) maximum lookahead distance [m]
   */
  void clip_lookahead_distance(float& ell, const float ell_min,
                               const float ell_max);

  /**
   * @brief Returns the index of the closest waypoint to the robot.
   * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
   * [m]
   * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
   * waypoints corresponding to the path [m]
   * @return (int) index of the closest waypoint to the robot
   */
  int find_closest_waypoint_index(const Eigen::Vector2f p,
                                  const Eigen::MatrixXf path);

  /**
   * @brief Rolls the path matrix such that the column corresponding to the
   * start index is at zero.
   * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
   * waypoints corresponding to the path to be rolled [m]
   * @param start_index (int) index of column to be pushed to zero
   * @return (Eigen::MatrixXf) rolled path
   */
  Eigen::MatrixXf roll_path(const Eigen::MatrixXf path, const int start_index);

  /**
   * @brief Rolls the indices of the velocity_profile vector such that the
   * element corresponding to the start index is at zero.
   * @param velocity_profile (const Eigen::VectorXf) 1-by-num_waypoints vector
   * of longitudinal velocities
   * @param start_index (int) index of element to be pushed to zero
   */
  void roll_velocity_profile(Eigen::VectorXf& velocity_profile,
                             const int start_index);

  /**
   * @brief Calculates the parameterization parameter of a circle and line
   * intersection point. If the point does not exist, this function will return
   * a negative parameter.
   * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
   * [m]
   * @param ell (const float) lookahead distance [m]
   * @param waypoints (const Eigen::Matrix2f) two waypoints for line segment
   * parameterization [m]
   * @return (float) parameterization parameter. Will return -1 if no valid
   * intersection exists
   */
  float calculate_circle_line_intersect_parameter(
      const Eigen::Vector2f p, const float ell,
      const Eigen::Matrix2f waypoints);

  /**
   * @brief Finds and calculates the lookahead point on the path.
   * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
   * [m]
   * @param ell (const float) lookahead distance [m]
   * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
   * waypoints corresponding to the path [m]
   * @return (Eigen::Vector2f) lookahead point [m]
   */
  Eigen::Vector2f calculate_lookahead_point(const Eigen::Vector2f p,
                                            const float ell,
                                            const Eigen::MatrixXf path);

 private:
  // TODO
  int closest_waypoint_index_;

  /**
   * @brief Returns the curvature of the turn.
   * @param y (const float) y distance between lookahead point and robot in
   * robot frame [m]
   * @param ell (const float) lookahead distance [m]
   * @return (float) curvature [rad/m]
   */
  float calculate_curvature(const float y, const float ell);

  /**
   * @brief Calculates the desired steering angle to send to the robot.
   * @param p (const Eigen::Vector2f) robot (x, y)-position in the global
   * frame [m]
   * @param lookahead_point_g (Eigen::Vector2f) lookahead point in the
   * global frame [m]
   * @param R_g_to_r (Eigen::Matrix3f) rotation matrix to bring points from
   * the global to body frame
   * @param ell (const float) lookahead distance [m]
   * @return (float) steering angle [rad]
   */
  float calculate_steering_angle(const Eigen::Vector2f p,
                                 const Eigen::Vector2f lookahead_point_g,
                                 const Eigen::Matrix3f R_g_to_r,
                                 const float ell);
};

#endif  // PURE_PURSUIT_CONTROLLER_HPP