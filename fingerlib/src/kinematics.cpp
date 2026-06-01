#include "fingerlib/kinematics.hpp"
#include "fingerlib/constants.hpp"
#include "fingerlib/nnls.h"

#include <cmath>
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>


namespace fingerlib {
  namespace {
    // Create a translation transformation matrix from a translation vector
    Eigen::Matrix4d translate(const Eigen::Vector3d& xyz)
    {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform.topRightCorner<3, 1>() = xyz;
      return transform;
    }

    // Get the homogeneous transformation matrix for a joint rotation about a specified axis and translation
    Eigen::Matrix4d joint_transform(const Eigen::Vector3d& xyz,
                                    const Eigen::Vector3d& axis,
                                    double angle)
    {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform.topLeftCorner<3, 3>() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
      transform.topRightCorner<3, 1>() = xyz;
      return transform;
    }

    // Convert a rotation matrix to a rotation vector
    Eigen::Vector3d rotation_vector(const Eigen::Matrix3d& rotation)
    {
      const Eigen::AngleAxisd angle_axis(rotation);
      if (!std::isfinite(angle_axis.angle()) || std::abs(angle_axis.angle()) < 1.0e-12) {
        return Eigen::Vector3d::Zero();
      }
      return angle_axis.axis() * angle_axis.angle();
    }

    // Compute FK: commanded joint positions -> end effector pose
    Eigen::Isometry3d fingertip_pose_internal(const Eigen::Vector3d& q_actuated,
                                              double tip_offset_m,
                                              int branch)
    {
      // Extract each commanded joint angle
      const double q_mcp_splay = q_actuated[0];
      const double q_mcp_flexion = q_actuated[1];
      const double q_pip = q_actuated[2];
      const double q_dip = solve_dip_from_pip(rad2deg(q_pip), branch);

      // Transform from base link to fingertip by chaining joint transforms
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform = transform * joint_transform(Eigen::Vector3d(MCP_SPLAY_ORIGIN_X, MCP_SPLAY_ORIGIN_Y, MCP_SPLAY_ORIGIN_Z),
                                              Eigen::Vector3d::UnitZ(), q_mcp_splay);
      transform = transform * joint_transform(Eigen::Vector3d(MCP_FLEX_ORIGIN_X, MCP_FLEX_ORIGIN_Y, MCP_FLEX_ORIGIN_Z),
                                              Eigen::Vector3d::UnitY(), q_mcp_flexion);
      transform = transform * joint_transform(Eigen::Vector3d(PIP_ORIGIN_X, PIP_ORIGIN_Y, PIP_ORIGIN_Z),
                                              Eigen::Vector3d::UnitY(), q_pip);
      transform = transform * joint_transform(Eigen::Vector3d(DIP_ORIGIN_X, DIP_ORIGIN_Y, DIP_ORIGIN_Z),
                                              Eigen::Vector3d::UnitY(), q_dip);
      transform = transform * translate(Eigen::Vector3d(tip_offset_m, 0.0, 0.0));

      // Return pose
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = transform.topLeftCorner<3, 3>();
      pose.translation() = transform.topRightCorner<3, 1>();
      return pose;
    }

    // Get error between two pose objects
    Eigen::Matrix<double, 6, 1> pose_error_body(const Eigen::Isometry3d& desired_pose,
                                                const Eigen::Isometry3d& current_pose)
    {
      const Eigen::Matrix3d current_rotation = current_pose.linear();

      Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
      error.head<3>() = current_rotation.transpose() * (desired_pose.translation() - current_pose.translation());
      error.tail<3>() = rotation_vector(current_rotation.transpose() * desired_pose.linear());
      return error;
    }
  }

  // Solve for the DIP angle given the PIP angle
  double solve_dip_from_pip(double qp_deg, int branch)
  {
    const auto qp = deg2rad(qp_deg);
    const auto p = PHI_P0 + qp;

    const auto cos_p = std::cos(p);
    const auto sin_p = std::sin(p);

    const auto A = D_LEN - L_LEN * cos_p;
    const auto B = -L_LEN * sin_p;
    const auto k = (B_LEN*B_LEN - (A*A + B*B + L_LEN*L_LEN)) / (2.0 * L_LEN);
    const auto R = std::sqrt(A*A + B*B);

    const auto psi = std::atan2(B, A);
    const auto u = psi + branch * std::acos(std::clamp(k / R, -1.0, 1.0));
    const auto qd = PHI_D0 - u;

    return qd;
  }

  // Get fingertip pose from joint angles
  Eigen::Isometry3d fingertip_pose(const Eigen::Vector3d& q_actuated,
                                   double tip_offset_m,
                                   int branch)
  {
    return fingertip_pose_internal(q_actuated, tip_offset_m, branch);
  }

  // Get only fingertip position from joint angles
  Eigen::Vector3d fingertip_position(const Eigen::Vector3d& q_actuated,
                                     double tip_offset_m,
                                     int branch)
  {
    return fingertip_pose_internal(q_actuated, tip_offset_m, branch).translation();
  }

  // Compute the Jacobian of fingertip pose w.r.t joint angles using finite differences
  Eigen::Matrix<double, 6, 3> fingertip_pose_jacobian(const Eigen::Vector3d& q_actuated,
                                                      double tip_offset_m,
                                                      int branch)
  {
    const auto current_pose = fingertip_pose_internal(q_actuated, tip_offset_m, branch);
    const Eigen::Matrix3d current_rotation = current_pose.linear();

    Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();

    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3d q_plus = q_actuated;
      Eigen::Vector3d q_minus = q_actuated;
      q_plus[i] += FINGERTIP_JACOBIAN_EPS;
      q_minus[i] -= FINGERTIP_JACOBIAN_EPS;

      const auto pose_plus = fingertip_pose_internal(q_plus, tip_offset_m, branch);
      const auto pose_minus = fingertip_pose_internal(q_minus, tip_offset_m, branch);

      const Eigen::Vector3d pos_plus_body = current_rotation.transpose() * pose_plus.translation();
      const Eigen::Vector3d pos_minus_body = current_rotation.transpose() * pose_minus.translation();
      const Eigen::Matrix3d rot_plus_body = current_rotation.transpose() * pose_plus.linear();
      const Eigen::Matrix3d rot_minus_body = current_rotation.transpose() * pose_minus.linear();

      J.block<3, 1>(0, i) = (pos_plus_body - pos_minus_body) / (2.0 * FINGERTIP_JACOBIAN_EPS);
      J.block<3, 1>(3, i) = rotation_vector(rot_minus_body.transpose() * rot_plus_body)
              / (2.0 * FINGERTIP_JACOBIAN_EPS);
    }

    return J;
  }

  // Get the fingertip pose error between desired and current
  Eigen::Matrix<double, 6, 1> fingertip_pose_error(const Eigen::Isometry3d& desired_pose,
                                                   const Eigen::Isometry3d& current_pose)
  {
    return pose_error_body(desired_pose, current_pose);
  }

  // Compute one step of joint angle increments to reduce fingertip pose error
  Eigen::Vector3d fingertip_pose_tracking_step(const Eigen::Vector3d& q_actuated,
                                               const Eigen::Isometry3d& desired_pose,
                                               const FingertipTrackingOptions& options)
  {
    // Compute the current pose and error
    const auto current_pose = fingertip_pose_internal(q_actuated, options.tip_offset_m, options.branch);

    Eigen::Matrix<double, 6, 1> error = pose_error_body(desired_pose, current_pose);
    error.head<3>() *= options.position_weight;
    error.tail<3>() *= options.orientation_weight;

    // Compute The jacobian
    Eigen::Matrix<double, 6, 3> J = fingertip_pose_jacobian(q_actuated, options.tip_offset_m, options.branch);
    J.topRows<3>() *= options.position_weight;
    J.bottomRows<3>() *= options.orientation_weight;

    const Eigen::Matrix<double, 6, 6> regularizer = (options.damping * options.damping)
                                                  * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 3, 1> step = J.transpose()
      * (J * J.transpose() + regularizer).ldlt().solve(error);

    const double step_norm = step.norm();
    if (std::isfinite(step_norm) && step_norm > options.max_step_norm && options.max_step_norm > 0.0) {
      step *= options.max_step_norm / step_norm;
    }

    return options.step_gain * step;
  }

  // Iteratively solve for joint angles that achieve a desired fingertip pose
  Eigen::Vector3d fingertip_inverse_kinematics(const Eigen::Isometry3d& desired_pose,
                                               const Eigen::Vector3d& initial_guess,
                                               const FingertipTrackingOptions& options)
  {
    Eigen::Vector3d q = initial_guess;
    for (int iter = 0; iter < options.max_iterations; ++iter) {
      const auto current_pose = fingertip_pose_internal(q, options.tip_offset_m, options.branch);
      const auto error = pose_error_body(desired_pose, current_pose);

      Eigen::Matrix<double, 6, 1> weighted_error = error;
      weighted_error.head<3>() *= options.position_weight;
      weighted_error.tail<3>() *= options.orientation_weight;

      if (weighted_error.norm() < options.tolerance) {
        break;
      }

      q += fingertip_pose_tracking_step(q, desired_pose, options);

      if (options.has_joint_limits) {
        for (int i = 0; i < 3; ++i) {
          q[i] = std::clamp(q[i], options.q_min[i], options.q_max[i]);
        }
      }
    }

    return q;
  }

  //
  // Tendon Tension Solver
  //
  // Get necessary tendon tensions to achieve a desired joint torque using NNLS
  Eigen::VectorXd tendon_tensions(Eigen::VectorXd desired_torques,
                                  Eigen::MatrixXd J)
  {
    fingerlib::NNLS<Eigen::MatrixXd> nnls(J);

    Eigen::VectorXd shift = Eigen::VectorXd::Constant(4, fingerlib::T_MIN); // Min tendon tension

    // Solve NNLS for tendon tensions
    nnls.solve(desired_torques - J * shift);
    if (nnls.info() == Eigen::Success) {
      const Eigen::VectorXd T = nnls.x() + shift;
      if (T.size() >= 4) {
        return T.head(4);
      }
    } else {
      std::cerr << "NNLS did not converge!" << std::endl;
    }

    return Eigen::VectorXd::Zero(4);
  }

  // Get necessary tendon tensions to achieve a desired joint torque using NNLS with soft constraint
  Eigen::VectorXd tendon_tensions_soft_constraint(Eigen::VectorXd desired_torques,
                                                  Eigen::MatrixXd J)
  {

    // Soft constraint T0==T1 by adding a small penalty alpha*(T0-T1)^2 to cost func
    Eigen::RowVector4d C; C << 1.0, -1.0, 0.0, 0.0;
    const double alpha = 0.1;
    Eigen::MatrixXd J_aug(J.rows() + 1, J.cols());
    J_aug.topRows(J.rows()) = J;
    J_aug.row(J.rows()) = std::sqrt(alpha) * C;
    Eigen::VectorXd b_aug(desired_torques.size() + 1);
    b_aug.head(desired_torques.size()) = desired_torques;
    b_aug.tail(1).setZero();

    fingerlib::NNLS<Eigen::MatrixXd> nnls(J_aug);

    Eigen::VectorXd shift = Eigen::VectorXd::Constant(4, fingerlib::T_MIN); // Min tendon tension

    // Solve NNLS for tendon tensions
    nnls.solve(b_aug - J_aug * shift);
    if (nnls.info() == Eigen::Success) {
      const Eigen::VectorXd T = nnls.x() + shift;
      if (T.size() >= 4) {
        return T.head(4);
      }
    } else {
      std::cerr << "NNLS did not converge!" << std::endl;
    }

    return Eigen::VectorXd::Zero(4);
  }

}