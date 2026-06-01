#include "fingerlib/constants.hpp"
#include "fingerlib/kinematics.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

// ./install/fingerlib/bin/precompute_jacobian /home/jarmibe7/ws_rds/src/rds-powerhouse-software/fingerlib/jacobian_transposes.csv

namespace {

struct Radii {
  double r_s1;
  double r_s2;
  double r_s3;
  double r_s4;
  double rm;
  double rp;
  double rd;
};

// Motor pulley radii in meters
constexpr Radii K_RADII{
    .r_s1 = 0.007056,
    .r_s2 = 0.004731,
    .r_s3 = 0.000799,
    .r_s4 = 0.002163,
    .rm = 0.026,
    .rp = 0.020,
    .rd = 0.020,
};

// Constants
constexpr int K_BRANCH = +1;                // Param for dip calculation
constexpr double K_QP_START_DEG = 0.0;      // Sweep start PIP angle [deg]
constexpr double K_QP_STOP_DEG = 90.0;      // Sweep stop PIP angle [deg]
constexpr double K_QP_STEP_DEG = 1.0;       // Sweep angle interval [deg]
constexpr double K_DERIV_STEP_DEG = 0.01;   // Finite difference step for alpha calc [deg]

// Helper for computing Jacobian for given alpha and radii
Eigen::Matrix<double, 4, 3> jacobian_from_alpha(const Radii& r, double alpha)
{
  Eigen::Matrix<double, 4, 3> J;
  J << -r.r_s1, -r.rm, -r.rp - alpha * r.rd,
       r.r_s2, -r.rm, -r.rp - alpha * r.rd,
       r.r_s3,  r.rm, 0.0,
      -r.r_s4, -r.rm,  r.rp + alpha * r.rd;
  return J;
}

// Compute ratio of velocities using finite difference dDIP / dPIP
double alpha_from_pip(double qp_deg, int branch, double h_deg = K_DERIV_STEP_DEG)
{
  const double qd_plus = fingerlib::solve_dip_from_pip(qp_deg + h_deg, branch);
  const double qd_minus = fingerlib::solve_dip_from_pip(qp_deg - h_deg, branch);
  const double dqd = qd_plus - qd_minus;
  const double dqp = fingerlib::deg2rad(2.0 * h_deg);
  return dqd / dqp;
}

// Helper for printing Jacobian
// void print_matrix(const Eigen::Matrix<double, 4, 3>& J)
// {
//   const Eigen::IOFormat FMT(9, 0, ", ", "\n", "[", "]");
//   std::cout << "\n" << J.transpose().format(FMT) << "\n\n";
// }

}

int main(int argc, char* argv[])
{
  const std::string output_path = (argc > 1) ? argv[1] : "jacobian_transposes.csv";
  std::ofstream out(output_path);
  if (!out.is_open()) {
    std::cerr << "Failed to open output file: " << output_path << "\n";
    return 1;
  }

  out << std::fixed << std::setprecision(9);
  out << "pip_deg,"
      << "jt_00,jt_01,jt_02,jt_03,"
      << "jt_10,jt_11,jt_12,jt_13,"
      << "jt_20,jt_21,jt_22,jt_23\n";

  // Perform sweep
  for (double qp_deg = K_QP_START_DEG; qp_deg <= K_QP_STOP_DEG + 1e-12; qp_deg += K_QP_STEP_DEG) {
    const double alpha = alpha_from_pip(qp_deg, K_BRANCH);
    const auto J = jacobian_from_alpha(K_RADII, alpha);
    const auto JT = J.transpose();

    out << qp_deg;
    for (int i = 0; i < JT.rows(); ++i) {
      for (int j = 0; j < JT.cols(); ++j) {
        out << ',' << JT(i, j);
      }
    }
    out << '\n';
  }

  std::cout << "Saved Jacobian transposes to " << output_path << "\n";

  return 0;
}
