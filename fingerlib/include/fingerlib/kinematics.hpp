#ifndef FINGERLIB_KINEMATICS_HPP_INCLUDE_GUARD
#define FINGERLIB_KINEMATICS_HPP_INCLUDE_GUARD

#if defined(F)
#  pragma push_macro("F")
#  undef F
#  define FINGERLIB_RESTORE_F_MACRO
#endif

#if defined(B1)
#  pragma push_macro("B1")
#  undef B1
#  define FINGERLIB_RESTORE_B1_MACRO
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "fingerlib/constants.hpp"

#include <limits>

#ifdef FINGERLIB_RESTORE_B1_MACRO
#  pragma pop_macro("B1")
#  undef FINGERLIB_RESTORE_B1_MACRO
#endif

#ifdef FINGERLIB_RESTORE_F_MACRO
#  pragma pop_macro("F")
#  undef FINGERLIB_RESTORE_F_MACRO
#endif

namespace fingerlib {

  struct FingertipTrackingOptions {
    double tip_offset_m{0.03};
    int branch{+1};
    double position_weight{1.0};
    double orientation_weight{0.5};
    double damping{1.0e-4};
    double step_gain{1.0};
    int max_iterations{50};
    double tolerance{1.0e-6};
  };
  

  /// \brief Closed-form four-bar closure: solves qD given qP.
  /// \param qP_deg Proximal joint angle in degrees.
  /// \param branch Assembly mode selector: +1 or -1. Use +1 to match qD ≈ qP at 0°, 45°, 90°.
  /// \return DIP angle in radians, as a double
  double solve_dip_from_pip(double qP_deg, int branch = +1);

  /// \brief Forward kinematics from the three actuated joint angles to the fingertip pose.
  /// \param q_actuated Commanded joint vector [mcp_splay, mcp_flexion, pip_flexion] in radians.
  /// \param tip_offset_m Distance from the DIP joint center to the fingertip along the distal phalanx.
  /// \param branch Assembly mode selector passed to the four-bar closure.
  /// \return Fingertip pose in the base_link frame.
  Eigen::Isometry3d fingertip_pose(const Eigen::Vector3d& q_actuated,
                                   double tip_offset_m = FINGERTIP_DEFAULT_OFFSET_M,
                                   int branch = +1);

  /// \brief Fingertip position extracted from fingertip_pose().
  Eigen::Vector3d fingertip_position(const Eigen::Vector3d& q_actuated,
                                     double tip_offset_m = FINGERTIP_DEFAULT_OFFSET_M,
                                     int branch = +1);

  /// \brief Body-frame pose Jacobian for the fingertip pose.
  /// \return 6x3 Jacobian mapping joint rates to body twist [vx, vy, vz, wx, wy, wz].
  Eigen::Matrix<double, 6, 3> fingertip_pose_jacobian(const Eigen::Vector3d& q_actuated,
                                                     double tip_offset_m = FINGERTIP_DEFAULT_OFFSET_M,
                                                     int branch = +1);

  /// \brief Pose error used by the fingertip tracker.
  /// \return 6-vector [position_error, orientation_error], both expressed in the current fingertip frame.
  Eigen::Matrix<double, 6, 1> fingertip_pose_error(const Eigen::Isometry3d& desired_pose,
                                                   const Eigen::Isometry3d& current_pose);

  /// \brief Compute one damped least-squares tracking step toward a desired fingertip pose.
  /// \param q_actuated Current actuated joint angles.
  /// \param desired_pose Desired fingertip pose in the base frame.
  /// \param options Tracking configuration.
  /// \return Joint increment that reduces pose error.
  Eigen::Vector3d fingertip_pose_tracking_step(const Eigen::Vector3d& q_actuated,
                                               const Eigen::Isometry3d& desired_pose,
                                               const FingertipTrackingOptions& options = {});

  /// \brief Iteratively solve for actuated joint angles that achieve a desired fingertip pose.
  /// \param desired_pose Desired fingertip pose in the base frame.
  /// \param initial_guess Initial joint-angle guess.
  /// \param options Tracking configuration.
  /// \return Actuated joint angles that approximately achieve the pose.
  Eigen::Vector3d fingertip_inverse_kinematics(const Eigen::Isometry3d& desired_pose,
                                               const Eigen::Vector3d& initial_guess,
                                               const FingertipTrackingOptions& options = {});

  /// \brief Calculate necessary tendon tensions to achieve desired joint torques with NNLS.
  /// \param desired_torques A vector of desired joint torques.
  /// \param J The Jacobian matrix mapping tendon tensions to joint torques.
  /// \return A vector of tendon tensions.
  Eigen::VectorXd tendon_tensions(Eigen::VectorXd desired_torques,
                                  Eigen::MatrixXd J);

  /// \brief NNLS solution, maintaining equal tensions in differential tendons
  /// \param desired_torques A vector of desired joint torques.
  /// \param J The Jacobian matrix mapping tendon tensions to joint torques.
  /// \return A vector of tendon tensions.
  Eigen::VectorXd tendon_tensions_soft_constraint(Eigen::VectorXd desired_torques,
                                                  Eigen::MatrixXd J);

} 
#endif