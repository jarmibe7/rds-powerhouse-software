#include "fingerlib/constants.hpp"
#include "fingerlib/kinematics.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iomanip>
#include <iostream>

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
constexpr double K_QP_STEP_DEG = 5.0;       // Sweep angle interval [deg]
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

// Helper for printing Jacobian
void print_matrix(const Eigen::Matrix<double, 4, 3>& J)
{
  const Eigen::IOFormat FMT(9, 0, ", ", "\n", "[", "]");
  std::cout << "\n" << J.transpose().format(FMT) << "\n\n";
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

}

int main()
{
  std::cout << std::fixed << std::setprecision(9);

  // Perform sweep
  for (double qp_deg = K_QP_START_DEG; qp_deg <= K_QP_STOP_DEG + 1e-12; qp_deg += K_QP_STEP_DEG) {
    const double qd_rad = fingerlib::solve_dip_from_pip(qp_deg, K_BRANCH);
    const double alpha = alpha_from_pip(qp_deg, K_BRANCH);
    const auto J = jacobian_from_alpha(K_RADII, alpha);

    std::cout << "q_pip_deg=" << qp_deg
              << " q_dip_deg=" << fingerlib::rad2deg(qd_rad)
              << " alpha=" << alpha;
    print_matrix(J);
  }

  return 0;
}
