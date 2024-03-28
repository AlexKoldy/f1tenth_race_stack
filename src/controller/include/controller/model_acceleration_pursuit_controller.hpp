#include <Eigen/Dense>
#include <controller/pure_pursuit_controller.hpp>

#ifndef MODEL_ACCELERATION_PURSUIT_CONTROLLER_HPP
#define MODEL_ACCELERATION_PURSUIT_CONTROLLER_HPP

class ModelAccelerationPursuitController : public PurePursuitController {
 public:
  /**
   * @brief Default constructor.
   * Initializes the ModelAccelerationPursuitController with no values.
   */
  ModelAccelerationPursuitController();

  /**
   * @brief Constructor with parameters.
   * Initializes the ModelAccelerationPursuitController with the given
   * parameters.
   * @param alpha (const float) lookahead distance slope [s]
   * @param beta (const float) lookahead distance bias [m]
   * @param ell_min (const float) minimum lookahead distance [m]
   * @param ell_max (const float) maximum lookahead distance [m]
   */
  ModelAccelerationPursuitController(const float alpha, const float beta,
                                     const float ell_min, const float ell_max);

  /**
   * @brief Performs one full controller step, by calculating and setting
   * the lookahead point and returning the steering angle.
   * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
   * [m]
   * @param q (const Eigen::Quaternionf) robot orientation as a quaternion
   * @param v_x (const float) commanded robot velocity in the robot frame [m/s]
   * @return (float) steering angle [rad]
   */
  float step(const Eigen::Vector2f p, const Eigen::Quaternionf q,
             const float v_x) override;

 private:
};

#endif  // MODEL_ACCELERATION_PURSUIT_CONTROLLER_HPP