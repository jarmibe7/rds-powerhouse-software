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

  Eigen::VectorXd tendon_tensions(Eigen::Matrix<double, 4, 1> desired_torques,
                                  Eigen::Matrix<double, 4, 3> J)
  {
    fingerlib::NNLS<Eigen::Matrix<double, 4, 3>> nnls(J);

    // Eigen::VectorXd shift = Eigen::VectorXd::Constant(4, 25);

    // Solve NNLS for tendon tensions
    nnls.solve(desired_torques);
    if (nnls.info() == Eigen::Success) {
      const Eigen::VectorXd T = nnls.x();
      if (T.size() >= 4) {
        return T.head<4>();
      }
    } else {
      std::cerr << "NNLS did not converge!" << std::endl;
    }

    return Eigen::VectorXd::Zero();
  }

}