#ifndef FINGERLIB_JACOBIAN_LOOKUP_HPP_INCLUDE_GUARD
#define FINGERLIB_JACOBIAN_LOOKUP_HPP_INCLUDE_GUARD

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace fingerlib {

  /// \brief Fast degree-indexed lookup for tendon Jacobian transposes.
  ///
  /// Loads a CSV table of Jacobian transpose entries indexed by PIP angle
  /// in degrees, then provides O(1) nearest-degree lookup.
  /// Angles are rounded to the nearest whole degree and clamped to table bounds.
  class JacobianLookup {
  public:
    /// \brief Construct a lookup table from a Jacobian-transpose CSV file.
    /// \param csv_path Path to a CSV containing rows: pip_deg, jt_00...jt_23
    /// \throws std::runtime_error If file cannot be opened or is malformed.
    explicit JacobianLookup(const std::string& csv_path);

    /// \brief Return the Jacobian nearest to a given angle in degrees.
    /// \param angle_deg Measured angle in degrees.
    /// \return Const reference to the table entry at nearest_degree(angle_deg).
    const Eigen::Matrix<double, 3, 4>& jacobian_for_angle_deg(float angle_deg) const;

    /// \brief Round to nearest whole degree and clamp to loaded table limits.
    /// \param angle_deg Measured angle in degrees.
    /// \return Degree index used for lookup.
    int nearest_degree(float angle_deg) const;

    /// \brief Smallest degree available in the loaded table.
    int min_degree() const { return min_degree_; }

    /// \brief Largest degree available in the loaded table.
    int max_degree() const { return max_degree_; }

  private:
    int min_degree_{0};
    int max_degree_{-1};
    std::vector<Eigen::Matrix<double, 3, 4>> jacobians_;
  };

}

#endif