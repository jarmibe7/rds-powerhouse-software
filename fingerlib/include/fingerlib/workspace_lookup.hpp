#ifndef FINGERLIB_WORKSPACE_LOOKUP_HPP_INCLUDE_GUARD
#define FINGERLIB_WORKSPACE_LOOKUP_HPP_INCLUDE_GUARD

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace fingerlib {

  struct WorkspaceSample {
    Eigen::Vector3d q_deg;
    Eigen::Vector3d position_m;
  };

  /// \brief Fast degree-indexed lookup for precomputed fingertip workspace samples.
  ///
  /// Loads a dense CSV table of joint-angle samples and fingertip positions,
  /// then provides O(1) nearest-grid lookup in joint-angle space.
  /// Angles are rounded to the nearest sampled grid point and clamped to the table bounds.
  class WorkspaceLookup {
  public:
    /// \brief Construct a lookup table from a workspace CSV file.
    /// \param csv_path Path to a CSV containing rows: q0_deg,q1_deg,q2_deg,x_m,y_m,z_m.
    /// \throws std::runtime_error If file cannot be opened or is malformed.
    explicit WorkspaceLookup(const std::string& csv_path);

    /// \brief Return the fingertip position for a given joint-angle triple in degrees.
    /// \param q_deg Measured joint angles in degrees.
    /// \return Fingertip position (x,y,z) in meters (by value).
    Eigen::Vector3d sample_workspace(const Eigen::Vector3d& q_deg) const;

    /// \brief Return the nearest sample for a given joint-angle triple in degrees.
    const WorkspaceSample& sample_for_joint_angles_deg(const Eigen::Vector3d& q_deg) const;

    /// \brief Convenience accessor returning only the fingertip position (stored reference).
    const Eigen::Vector3d& position_for_joint_angles_deg(const Eigen::Vector3d& q_deg) const { return sample_for_joint_angles_deg(q_deg).position_m; }

    /// \brief Round each angle to the nearest sampled grid point and clamp to table bounds.
    /// \param q_deg Measured joint angles in degrees.
    /// \return Grid-aligned joint angles used for lookup.
    Eigen::Vector3i nearest_joint_degrees(const Eigen::Vector3d& q_deg) const;

    int q0_min_degree() const { return q0_min_degree_; }
    int q0_max_degree() const { return q0_max_degree_; }
    int q0_step_degree() const { return q0_step_degree_; }

    int q1_min_degree() const { return q1_min_degree_; }
    int q1_max_degree() const { return q1_max_degree_; }
    int q1_step_degree() const { return q1_step_degree_; }

    int q2_min_degree() const { return q2_min_degree_; }
    int q2_max_degree() const { return q2_max_degree_; }
    int q2_step_degree() const { return q2_step_degree_; }

  private:
    struct AxisGrid {
      int min_degree{0};
      int max_degree{-1};
      int step_degree{1};
      std::size_t count{0};
    };

    static int nearest_grid_degree(float angle_deg, const AxisGrid& grid);
    std::size_t index_for_degrees(int q0_deg, int q1_deg, int q2_deg) const;

    AxisGrid q0_grid_;
    AxisGrid q1_grid_;
    AxisGrid q2_grid_;
    int q0_min_degree_{0};
    int q0_max_degree_{-1};
    int q0_step_degree_{1};
    int q1_min_degree_{0};
    int q1_max_degree_{-1};
    int q1_step_degree_{1};
    int q2_min_degree_{0};
    int q2_max_degree_{-1};
    int q2_step_degree_{1};
    std::vector<WorkspaceSample> samples_;
  };

}

#endif