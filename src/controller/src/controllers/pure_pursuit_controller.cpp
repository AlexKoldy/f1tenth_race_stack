#include <Eigen/Dense>
#include <cmath>
#include <controller/pure_pursuit_controller.hpp>
#include <iostream>

/**
 * @brief Default constructor.
 * Initializes the PurePursuitController with no values.
 */
PurePursuitController::PurePursuitController() {}

/**
 * @brief Constructor with parameters.
 * Initializes the PurePursuitController with the given parameters.
 * @param alpha (const float) lookahead distance slope [s]
 * @param beta (const float) lookahead distance bias [m]
 * @param ell_min (const float) minimum lookahead distance [m]
 * @param ell_max (const float) maximum lookahead distance [m]
 */
PurePursuitController::PurePursuitController(const float alpha,
                                             const float beta,
                                             const float ell_min,
                                             const float ell_max,
                                             const float k_p)
    : alpha_(alpha),
      beta_(beta),
      ell_min_(ell_min),
      ell_max_(ell_max),
      k_p_(k_p) {}

/**
 * @brief Returns the lookahead point of the controller
 * @return (Eigen::Vector2f) (x,y)-position of lookahead point [m]
 */
Eigen::Vector2f PurePursuitController::get_lookahead_point() {
  return this->lookahead_point_;
}

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
std::pair<float, float> PurePursuitController::step(
    const Eigen::Vector2f p, const Eigen::Quaternionf q) {
  // Find the closest waypoint to the robot
  int closest_waypoint_index =
      this->find_closest_waypoint_index(p, this->path_);

  // Roll the path and velocity profile such that the closest index is the
  // (0)th element of the matrix and vector, respectively
  this->roll_path(this->path_, closest_waypoint_index);
  this->roll_velocity_profile(this->velocity_profile_, closest_waypoint_index);

  // Extract the longitudinal velocity from the profile
  float v_x = this->velocity_profile_(0);  // [m/s]

  // Calculate the lookahead distance based off of target speed
  float ell = this->calculate_lookahead_distance(v_x);

  // Clip the lookahead distance
  this->clip_lookahead_distance(ell, this->ell_min_, this->ell_max_);

  // std::cout << this->velocity_profile_ << std::endl;
  std::cout << "v_x: " << v_x << std::endl;
  std::cout << "ell: " << ell << std::endl;

  // Find the lookahead point
  this->lookahead_point_ = this->calculate_lookahead_point(p, ell, this->path_);

  // Convert the orientation quaternion to a rotation matrix. Transpose the
  // matrix so it can be used to rotate from the global frame to the robot frame
  Eigen::Matrix3f R = q.normalized().toRotationMatrix();

  // Calculate the steering angle
  float delta = this->calculate_steering_angle(p, this->lookahead_point_,
                                               R.transpose(), ell);  // [rad]
  return std::make_pair(v_x, delta);
}

/**
 * @brief Calculates the raw lookahead distance based off of linear relationship
 * with velocity: ell = alpha * v_x + beta.
 * @param v_x (const float) commanded robot velocity in the robot frame [m/s]
 * @return (float) raw lookahead distance [m]
 */
float PurePursuitController::calculate_lookahead_distance(const float v_x) {
  return this->alpha_ * v_x + this->beta_;
}

/**
 * @brief Clips the lookahead distance between some minimum and maximum value
 * @param ell (float&) lookahead distance to be clipped [m]
 * @param ell_min (const float) minimum lookahead distance [m]
 * @param ell_max (const float) maximum lookahead distance [m]
 */
void PurePursuitController::clip_lookahead_distance(float& ell,
                                                    const float ell_min,
                                                    const float ell_max) {
  ell = std::min(ell_max, std::max(ell_min, ell));
}

/**
 * @brief Returns the index of the closest waypoint to the robot.
 * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
 * [m]
 * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
 * waypoints corresponding to the path [m]
 * @return (int) index of the closest waypoint to the robot
 */
int PurePursuitController::find_closest_waypoint_index(
    const Eigen::Vector2f p, const Eigen::MatrixXf path) {
  // Set up an invalid index and large minimum distance
  int closest_waypoint_index = -1;
  float d_min = std::numeric_limits<float>::max();  // minimum distance [m]

  // Loop through each point in the path and find the closest waypoint
  for (int i = 0; i < path.cols(); ++i) {
    Eigen::Vector2f waypoint = path.col(i).head<2>();

    // Calculate the distance between the vectors representing
    // the points
    float d = (waypoint - p).norm();

    // Update the minimum distance and closest index if appropriate
    if (d < d_min) {
      d_min = d;
      closest_waypoint_index = i;
    }
  }

  return closest_waypoint_index;
}

/**
 * @brief Rolls the path matrix such that the column corresponding to the start
 * index is at zero.
 * @param path (Eigen::MatrixXf&) 2-by-num_waypoints matrix of (x, y)
 * waypoints corresponding to the path to be rolled [m]
 * @param start_index (int) index of column to be pushed to zero
 */
void PurePursuitController::roll_path(Eigen::MatrixXf& path,
                                      const int start_index) {
  // Extract dimensions of the path matrix
  int num_rows = path.rows();
  int num_cols = path.cols();

  // Set up a temporary rolled matrix
  Eigen::MatrixXf rolled_path(num_rows, num_cols);

  // Copy the elements from the start index to the end such that
  // the columns corresponding to start index are now at zero
  rolled_path.block(0, 0, num_rows, num_cols - start_index) =
      path.block(0, start_index, num_rows, num_cols - start_index);

  // Copy the elements from zero to the start index
  rolled_path.block(0, num_cols - start_index, num_rows, start_index) =
      path.block(0, 0, num_rows, start_index);

  // Update the path
  path = rolled_path;
}

/**
 * @brief Rolls the indices of the velocity_profile vector such that the element
 * corresponding to the start index is at zero.
 * @param velocity_profile (const Eigen::VectorXf) 1-by-num_waypoints vector of
 * longitudinal velocities
 * @param start_index (int) index of element to be pushed to zero
 */
void PurePursuitController::roll_velocity_profile(
    Eigen::VectorXf& velocity_profile, const int start_index) {
  int num_elements = velocity_profile.size();

  // Set up a temporary rolled vector
  Eigen::VectorXf rolled_velocity_profile(num_elements);

  // Copy the elements from the start index to the end such that
  // the element corresponding to start index is now at zero
  rolled_velocity_profile.head(num_elements - start_index) =
      velocity_profile.segment(start_index, num_elements - start_index);

  // Copy the elements from zero to the start index
  rolled_velocity_profile.tail(start_index) =
      velocity_profile.head(start_index);

  // Update the velocity_profile
  velocity_profile = rolled_velocity_profile;
}

/**
 * @brief Calculates the parameterization parameter of a circle and line
 * intersection point. If the point does not exist, this function will return a
 * negative parameter.
 * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
 * [m]
 * @param ell (const float) lookahead distance [m]
 * @param waypoints (const Eigen::Matrix2f) two waypoints for line segment
 * parameterization [m]
 * @return (float) parameterization parameter. Will return -1 if no valid
 * intersection exists
 */
float PurePursuitController::calculate_circle_line_intersect_parameter(
    const Eigen::Vector2f p, const float ell, const Eigen::Matrix2f waypoints) {
  // Calculate direction vector between the waypoints
  Eigen::Vector2f dir = waypoints.col(1) - waypoints.col(0);

  // Calculate displacement from the circle's center to the
  // first waypoint
  Eigen::Vector2f disp = waypoints.col(0) - p;

  // The line is parameterized as waypoint_0 + t * dir
  // Now we form the quadratic equation at^2 + bt + c = 0
  // and solve for t. This will give us our intersections
  float a = dir.dot(dir);
  float b = 2.0 * disp.dot(dir);
  float c = disp.dot(disp) - std::pow(ell, 2);
  float discriminant = std::pow(b, 2) - 4 * a * c;

  // Check if the discriminant is valid, if not we don't have
  // a valid solution
  if (discriminant < 0) {
    return -1.0f;
  }

  // Calculate the intersection parameter 't'
  float t_1 = (-b + std::sqrt(discriminant)) / (2.0 * a);
  float t_2 = (-b - std::sqrt(discriminant)) / (2.0 * a);

  // If both points lie on the line, i.e., 0 <= t <= 1, then
  // we return the maximum, as this corresponds to a point
  // further along the line. If neither point is within this
  // range, we return an invalid solution
  if ((0 <= t_1 && t_1 <= 1) && (0 <= t_2 && t_2 <= 1)) {
    return std::max(t_1, t_2);
  } else if (0 <= t_1 && t_1 <= 1) {
    return t_1;
  } else if (0 <= t_2 && t_2 <= 1) {
    return t_2;
  } else {
    return -1.0f;
  }
}

/**
 * @brief Finds and calculates the lookahead point on the path.
 * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
 * [m]
 * @param ell (const float) lookahead distance [m]
 * @param path (const Eigen::MatrixXf) 2-by-num_waypoints matrix of (x, y)
 * waypoints corresponding to the path [m]
 * @return (Eigen::Vector2f) lookahead point [m]
 */
Eigen::Vector2f PurePursuitController::calculate_lookahead_point(
    const Eigen::Vector2f p, const float ell, const Eigen::MatrixXf path) {
  // Find the number of waypoints
  int num_waypoints = path.cols();

  // Loop through each (i)th and (i+1)th waypoint, parameterize
  // a line between them, and check for an intersection with the
  // lookahead circle
  for (int i = 0; i < num_waypoints - 1; ++i) {
    Eigen::Matrix2f waypoints;
    waypoints.col(0) = path.col(i);
    waypoints.col(1) = path.col(i + 1);

    // This function will return -1 if there is no intersection
    float t =
        this->calculate_circle_line_intersect_parameter(p, ell, waypoints);

    // Check if an intersection exists
    if (t >= 0) {
      // Reconstruct the line segment leading up to the
      // intersction, i.e.,
      //  lookahead_point = waypoint[i] + t * dir
      Eigen::Vector2f dir = waypoints.col(1) - waypoints.col(0);
      return waypoints.col(0) + t * dir;
    }
  }

  // If no valid intersection is found, simply return the
  // first index of the path (which should correspond to
  // the closest waypoint to the robot)
  return path.col(0);
}

/**
 * @brief Returns the curvature of the turn.
 * @param y (const float) y distance between lookahead point and robot in robot
 * frame [m]
 * @param ell (const float) lookahead distance [m]
 * @return (float) curvature [rad/m]
 */
float PurePursuitController::calculate_curvature(const float y,
                                                 const float ell) {
  return 2 * y / (std::pow(ell, 2));
}

/**
 * @brief Calculates the desired steering angle to send to the robot.
 * @param p (const Eigen::Vector2f) robot (x, y)-position in the global frame
 * [m]
 * @param lookahead_point_g (const Eigen::Vector2f) lookahead point in the
 * global frame [m]
 * @param R_g_to_r (const Eigen::Matrix3f) rotation matrix to bring points from
 * the global to body frame
 * @param ell (const float) lookahead distance [m]
 * @return (float) steering angle [rad]
 */
float PurePursuitController::calculate_steering_angle(
    const Eigen::Vector2f p, const Eigen::Vector2f lookahead_point_g,
    const Eigen::Matrix3f R_g_to_r, const float ell) {
  // TODO: find an elegeant way to do this so you don't have to do it manually
  // Transpose the lookahead point from the global frame to the robot frame
  Eigen::Vector2f lookahead_point_r =
      R_g_to_r.block<2, 2>(0, 0) * (lookahead_point_g - p);

  // Calculate the curvature using the y-value of the lookahead point in the
  // robot frame
  float kappa = this->calculate_curvature(lookahead_point_r(1),
                                          ell);  // curvature [rad/m]

  // Apply a proportional gain to the curvature to calculate the steering angle
  return this->k_p_ * kappa;  // steering angle [rad]
}
