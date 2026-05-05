#include "fingerlib/kinematics.hpp"
#include "fingerlib/constants.hpp"
#include "fingerlib/nnls.h"

#include <cmath>
#include <algorithm>
#include <iostream>


namespace fingerlib {
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