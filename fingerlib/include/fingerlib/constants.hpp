#ifndef FINGERLIB_CONSTANTS_HPP_INCLUDE_GUARD
#define FINGERLIB_CONSTANTS_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles and constants
#include <cmath>
#include <numbers>

namespace fingerlib {

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
      using std::numbers::pi;
      const auto two_pi = 2.0 * pi;
      rad = std::fmod(rad, two_pi);                                      // [-2pi, 2pi)
      if (rad < -pi || almost_equal(rad, -pi)) { rad += two_pi; }        // (-pi, pi]
      else if (rad > pi) { rad -= two_pi; }
      return rad;
  }

  /// \brief Convert degrees to radians
  /// \param deg Angle in degrees
  /// \returns The equivalent angle in radians
  constexpr double deg2rad(double deg)
  {
      return (std::numbers::pi / 180.0)*deg;
  }

  /// \brief Convert radians to degrees
  /// \param rad  Angle in radians
  /// \returns The equivalent angle in degrees
  constexpr double rad2deg(double rad)
  {
      return (180.0 / std::numbers::pi)*rad;
  }

  // Four-bar geometry [mm]
  inline constexpr double D_LEN = 40.0;
  inline constexpr double L_LEN = 10.0;
  inline constexpr double B_LEN = 36.293;

  // Mount angles [rad]
  inline constexpr double PHI_P0 = deg2rad(-109.736);
  inline constexpr double PHI_D0 =  deg2rad(160.264);

}
#endif