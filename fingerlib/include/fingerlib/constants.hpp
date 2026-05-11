#ifndef FINGERLIB_CONSTANTS_HPP_INCLUDE_GUARD
#define FINGERLIB_CONSTANTS_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles and constants
#include <cmath>
#include <numbers>

namespace fingerlib {
  // Four-bar geometry [m]
  inline constexpr double D_LEN = 0.040;         // Center distance between DIP and PIP joints
  inline constexpr double L_LEN = 0.010888;      // Distance between PIP and exterior bar attachment on proximal member
  inline constexpr double B_LEN = 0.036299;      // Exterior bar length, center to center

  // Physical constants
  inline constexpr double PI = 3.14159265358979;

  
  inline constexpr double T_MIN = 25.0;     // Minimum tendon tension [N]
  inline constexpr double R_MOTOR = 0.006;  // Motor pulley radius [m]

  /// \brief Approximately compare two floating-point numbers using
  ///        an absolute comparison
  /// \param d1 A number to compare
  /// \param d2 A second number to compare2
  /// \param epsilon Absolute threshold required for equality
  /// \return true if abs(d1 - d2) < epsilon
  constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
  {
      return std::abs(d1 - d2) < epsilon;
  }

  /// \brief Wrap an angle to (-pi, pi]
  /// \param rad Angle in radians
  /// \return An equivalent angle the range (-pi, pi]
  constexpr double normalize_angle(double rad)
  {
      const auto two_pi = 2.0 * PI;
      rad = std::fmod(rad, two_pi);                                      // [-2pi, 2pi)
      if (rad < -PI || almost_equal(rad, -PI)) { rad += two_pi; }        // (-pi, pi]
      else if (rad > PI) { rad -= two_pi; }
      return rad;
  }

  /// \brief Convert degrees to radians
  /// \param deg Angle in degrees
  /// \returns The equivalent angle in radians
  constexpr double deg2rad(double deg)
  {
      return (PI / 180.0)*deg;
  }

  /// \brief Convert radians to degrees
  /// \param rad  Angle in radians
  /// \returns The equivalent angle in degrees
  constexpr double rad2deg(double rad)
  {
      return (180.0 / PI)*rad;
  }

  // Mount angles [rad]
  inline constexpr double PHI_P0 = deg2rad(-109.736);
  inline constexpr double PHI_D0 =  deg2rad(160.264);
}
#endif