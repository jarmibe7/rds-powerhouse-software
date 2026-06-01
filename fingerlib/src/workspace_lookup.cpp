#include "fingerlib/workspace_lookup.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <unordered_set>
#include <limits>
#include <vector>

namespace fingerlib {

  // Construct lookup table from CSV
  WorkspaceLookup::WorkspaceLookup(const std::string& csv_path)
  {
    // Open CSV
    std::ifstream in(csv_path);
    if (!in.is_open()) {
      throw std::runtime_error("Failed to open workspace CSV: " + csv_path);
    }

    std::string line;
    if (!std::getline(in, line)) {
      throw std::runtime_error("Workspace CSV is empty: " + csv_path);
    }

    std::vector<WorkspaceSample> rows;
    std::unordered_set<int> q0_set;
    std::unordered_set<int> q1_set;
    std::unordered_set<int> q2_set;
    int q0_min = std::numeric_limits<int>::max();
    int q1_min = std::numeric_limits<int>::max();
    int q2_min = std::numeric_limits<int>::max();
    int q0_max = std::numeric_limits<int>::min();
    int q1_max = std::numeric_limits<int>::min();
    int q2_max = std::numeric_limits<int>::min();

    // Read CSV rows into vector of samples
    while (std::getline(in, line)) {
      if (line.empty()) {
        continue;
      }

      std::stringstream ss(line);
      std::string cell;
      std::vector<double> values;
      values.reserve(6);

      while (std::getline(ss, cell, ',')) {
        values.push_back(std::stod(cell));
      }

      if (values.size() != 6) {
        throw std::runtime_error("Malformed workspace CSV row: expected 6 columns");
      }

      const int q0_deg = static_cast<int>(std::lround(values[0]));
      const int q1_deg = static_cast<int>(std::lround(values[1]));
      const int q2_deg = static_cast<int>(std::lround(values[2]));

      rows.push_back(WorkspaceSample{
        Eigen::Vector3d(values[0], values[1], values[2]),
        Eigen::Vector3d(values[3], values[4], values[5]),
      });

      q0_set.insert(q0_deg);
      q1_set.insert(q1_deg);
      q2_set.insert(q2_deg);

      q0_min = std::min(q0_min, q0_deg);
      q1_min = std::min(q1_min, q1_deg);
      q2_min = std::min(q2_min, q2_deg);
      q0_max = std::max(q0_max, q0_deg);
      q1_max = std::max(q1_max, q1_deg);
      q2_max = std::max(q2_max, q2_deg);
    }

    if (rows.empty()) {
      throw std::runtime_error("Workspace CSV has no data rows: " + csv_path);
    }

    // Compute per-axis min/max/count and derive integer step assuming a regular grid.
    if (q0_set.empty() || q1_set.empty() || q2_set.empty()) {
      throw std::runtime_error("Workspace CSV has no valid grid values: " + csv_path);
    }

    q0_grid_.min_degree = q0_min;
    q0_grid_.max_degree = q0_max;
    q0_grid_.count = static_cast<std::size_t>(q0_set.size());
    q0_grid_.step_degree = (q0_grid_.count > 1) ? ((q0_grid_.max_degree - q0_grid_.min_degree) / (static_cast<int>(q0_grid_.count) - 1)) : 1;
    q0_min_degree_ = q0_grid_.min_degree;
    q0_max_degree_ = q0_grid_.max_degree;
    q0_step_degree_ = q0_grid_.step_degree;

    q1_grid_.min_degree = q1_min;
    q1_grid_.max_degree = q1_max;
    q1_grid_.count = static_cast<std::size_t>(q1_set.size());
    q1_grid_.step_degree = (q1_grid_.count > 1) ? ((q1_grid_.max_degree - q1_grid_.min_degree) / (static_cast<int>(q1_grid_.count) - 1)) : 1;
    q1_min_degree_ = q1_grid_.min_degree;
    q1_max_degree_ = q1_grid_.max_degree;
    q1_step_degree_ = q1_grid_.step_degree;

    q2_grid_.min_degree = q2_min;
    q2_grid_.max_degree = q2_max;
    q2_grid_.count = static_cast<std::size_t>(q2_set.size());
    q2_grid_.step_degree = (q2_grid_.count > 1) ? ((q2_grid_.max_degree - q2_grid_.min_degree) / (static_cast<int>(q2_grid_.count) - 1)) : 1;
    q2_min_degree_ = q2_grid_.min_degree;
    q2_max_degree_ = q2_grid_.max_degree;
    q2_step_degree_ = q2_grid_.step_degree;

    const std::size_t expected_size = q0_grid_.count * q1_grid_.count * q2_grid_.count;
    if (rows.size() != expected_size) {
      throw std::runtime_error("Workspace CSV does not form a dense rectangular grid");
    }

    samples_.assign(expected_size, WorkspaceSample{});

    for (const auto& sample : rows) {
      const int q0_deg = static_cast<int>(std::lround(sample.q_deg.x()));
      const int q1_deg = static_cast<int>(std::lround(sample.q_deg.y()));
      const int q2_deg = static_cast<int>(std::lround(sample.q_deg.z()));

      samples_[index_for_degrees(q0_deg, q1_deg, q2_deg)] = sample;
    }
  }

  // Round a given angle to nearest grid point and clamp to table bounds
  int WorkspaceLookup::nearest_grid_degree(float angle_deg, const AxisGrid& grid)
  {
    const double scaled = (static_cast<double>(angle_deg) - static_cast<double>(grid.min_degree))
                        / static_cast<double>(grid.step_degree);
    const int index = static_cast<int>(std::lround(scaled));
    const int clamped_index = std::clamp(index, 0, static_cast<int>(grid.count) - 1);
    return grid.min_degree + clamped_index * grid.step_degree;
  }

  // Compute 1D index into the samples vector for given joint angles in degrees
  std::size_t WorkspaceLookup::index_for_degrees(int q0_deg, int q1_deg, int q2_deg) const
  {
    const int q0_offset = q0_deg - q0_grid_.min_degree;
    const int q1_offset = q1_deg - q1_grid_.min_degree;
    const int q2_offset = q2_deg - q2_grid_.min_degree;

    if (q0_offset < 0 || q1_offset < 0 || q2_offset < 0) {
      throw std::out_of_range("Workspace CSV lookup degree is below table bounds");
    }
    if ((q0_offset % q0_grid_.step_degree) != 0
        || (q1_offset % q1_grid_.step_degree) != 0
        || (q2_offset % q2_grid_.step_degree) != 0) {
      throw std::out_of_range("Workspace CSV lookup degree is not on the sampled grid");
    }

    const std::size_t q0_index = static_cast<std::size_t>(q0_offset / q0_grid_.step_degree);
    const std::size_t q1_index = static_cast<std::size_t>(q1_offset / q1_grid_.step_degree);
    const std::size_t q2_index = static_cast<std::size_t>(q2_offset / q2_grid_.step_degree);

    return (q0_index * q1_grid_.count + q1_index) * q2_grid_.count + q2_index;
  }

  // Return the workspace position for neares joint angles to query
  Eigen::Vector3d WorkspaceLookup::sample_workspace(const Eigen::Vector3d& q_deg) const
  {
    const int q0_nearest = nearest_grid_degree(static_cast<float>(q_deg.x()), q0_grid_);
    const int q1_nearest = nearest_grid_degree(static_cast<float>(q_deg.y()), q1_grid_);
    const int q2_nearest = nearest_grid_degree(static_cast<float>(q_deg.z()), q2_grid_);
    return samples_[index_for_degrees(q0_nearest, q1_nearest, q2_nearest)].position_m;
  }

  // Return the nearest sample (rounded angles + workspace position) for a given joint-angle query
  const WorkspaceSample& WorkspaceLookup::sample_for_joint_angles_deg(const Eigen::Vector3d& q_deg) const
  {
    const int q0_nearest = nearest_grid_degree(static_cast<float>(q_deg.x()), q0_grid_);
    const int q1_nearest = nearest_grid_degree(static_cast<float>(q_deg.y()), q1_grid_);
    const int q2_nearest = nearest_grid_degree(static_cast<float>(q_deg.z()), q2_grid_);
    return samples_[index_for_degrees(q0_nearest, q1_nearest, q2_nearest)];
  }

  // Round given joint query to sweep grid
  Eigen::Vector3i WorkspaceLookup::nearest_joint_degrees(const Eigen::Vector3d& q_deg) const
  {
    return Eigen::Vector3i(
      nearest_grid_degree(static_cast<float>(q_deg.x()), q0_grid_),
      nearest_grid_degree(static_cast<float>(q_deg.y()), q1_grid_),
      nearest_grid_degree(static_cast<float>(q_deg.z()), q2_grid_));
  }

}