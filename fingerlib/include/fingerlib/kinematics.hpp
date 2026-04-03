#ifndef FINGERLIB_KINEMATICS_HPP_INCLUDE_GUARD
#define FINGERLIB_KINEMATICS_HPP_INCLUDE_GUARD

namespace fingerlib {
  /// \brief Closed-form four-bar closure: solves qD given qP.
  /// \param qP_deg Proximal joint angle in degrees.
  /// \param branch Assembly mode selector: +1 or -1. Use +1 to match qD ≈ qP at 0°, 45°, 90°.
  /// \return DIP angle in radians, as a double
  double solve_qD_from_qP(double qP_deg, int branch = +1);
} 
#endif