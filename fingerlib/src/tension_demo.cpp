#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include "fingerlib/kinematics.hpp"
#include "fingerlib/jacobian_lookup.hpp"
#include "fingerlib/nnls.h"

int main()
{
  try {
    // Jacobian
    Eigen::Matrix<double, 3, 4> J;
        // Use the same Jacobian hard-coded in the firmware test harness
        J << -0.00706, 0.00473, 0.000799, -0.00216,
          -0.026,   -0.026,   0.026,    -0.026,
          -0.039959,-0.039959,0.0,      0.039959;
    std::cout << "J:\n" << J << "\n\n";

    // Desired joint torques
    Eigen::Vector3d tau;
    tau << -1.0, 0.0, 0.0;
    std::cout << "Desired tau (Nm): " << tau.transpose() << "\n\n";

    // Min tendon tension
    const double T_min = 10.0;
    std::cout << "Min tendon tension (N): " << T_min << "\n\n";
    const Eigen::Vector4d T_shift = Eigen::Vector4d::Constant(T_min);

    // fingerlib NNLS
    Eigen::VectorXd tau_vec = tau;
    Eigen::MatrixXd Jdyn = J.cast<double>();
    fingerlib::NNLS<Eigen::MatrixXd> solver(Jdyn);
    solver.solve(tau_vec - Jdyn * T_shift);
    std::cout << "fingerlib NNLS tensions (N): " << (solver.x() + T_shift).transpose() << "\n";

    // Hard constrain T0==T1 by reducing Jacobian
    Eigen::Matrix<double, 3, 3> Jred;
    Jred.col(0) = J.col(0) + J.col(1);
    Jred.col(1) = J.col(2);
    Jred.col(2) = J.col(3);
    Eigen::MatrixXd Jred_dyn = Jred.cast<double>();
    fingerlib::NNLS<Eigen::MatrixXd> solver_red(Jred_dyn);
    solver_red.solve(tau_vec);
    Eigen::VectorXd xred = solver_red.x();
    Eigen::Vector4d T_red;
    T_red << xred(0), xred(0), xred(1), xred(2);
    std::cout << "Hard constraint tensions (N): " << T_red.transpose() << "\n";

    // Soft constraint T0==T1 by adding a small penalty alpha*(T0-T1)^2 to cost func
    Eigen::RowVector4d C; C << 1.0, -1.0, 0.0, 0.0;
    const double alpha = 0.1;
    Eigen::MatrixXd A_aug(Jdyn.rows() + 1, Jdyn.cols());
    A_aug.topRows(Jdyn.rows()) = Jdyn;
    A_aug.row(Jdyn.rows()) = std::sqrt(alpha) * C;
    Eigen::VectorXd b_aug(tau_vec.size() + 1);
    b_aug.head(tau_vec.size()) = tau_vec;
    b_aug.tail(1).setZero();

    fingerlib::NNLS<Eigen::MatrixXd> solver_aug(A_aug);
    solver_aug.solve(b_aug - A_aug * T_shift);
    std::cout << "Soft constraint alpha=" << alpha << " tensions (N): "
              << (solver_aug.x() + T_shift).transpose() << "\n";

    std::cout << "\nDone.\n";
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
