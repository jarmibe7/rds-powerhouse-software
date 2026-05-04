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

#ifdef FINGERLIB_RESTORE_B1_MACRO
#  pragma pop_macro("B1")
#  undef FINGERLIB_RESTORE_B1_MACRO
#endif

#ifdef FINGERLIB_RESTORE_F_MACRO
#  pragma pop_macro("F")
#  undef FINGERLIB_RESTORE_F_MACRO
#endif

namespace fingerlib {
  

  /// \brief Closed-form four-bar closure: solves qD given qP.
  /// \param qP_deg Proximal joint angle in degrees.
  /// \param branch Assembly mode selector: +1 or -1. Use +1 to match qD ≈ qP at 0°, 45°, 90°.
  /// \return DIP angle in radians, as a double
  double solve_dip_from_pip(double qP_deg, int branch = +1);

  /// \brief Calculate necessary tendon tensions to achieve desired joint torques with NNLS.
  /// \param desired_torques A vector of desired joint torques.
  /// \param J The Jacobian matrix mapping tendon tensions to joint torques.
  /// \return A vector of tendon tensions.
  Eigen::VectorXd tendon_tensions(Eigen::Matrix<double, 4, 1> desired_torques,
                                  Eigen::Matrix<double, 4, 3> J);

} 
#endif