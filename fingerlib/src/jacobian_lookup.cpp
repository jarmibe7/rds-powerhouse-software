#include "fingerlib/jacobian_lookup.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fingerlib {

  // Construct lookup table from CSV
  JacobianLookup::JacobianLookup(const std::string& csv_path)
  {
    // Open CSV
    std::ifstream in(csv_path);
    if (!in.is_open()) {
      throw std::runtime_error("Failed to open Jacobian CSV: " + csv_path);
    }

    std::string line;
    if (!std::getline(in, line)) {
      throw std::runtime_error("Jacobian CSV is empty: " + csv_path);
    }

    // Read CSV rows into vector of (degree, Jacobian) pairs, then sort by degree.
    std::vector<std::pair<int, Eigen::Matrix<double, 3, 4>>> rows;
    while (std::getline(in, line)) {
      if (line.empty()) {
        continue;
      }

      std::stringstream ss(line);
      std::string cell;
      std::vector<double> values;
      values.reserve(13);

      while (std::getline(ss, cell, ',')) {
        values.push_back(std::stod(cell));
      }

      if (values.size() != 13) {
        throw std::runtime_error("Malformed Jacobian CSV row: expected 13 columns");
      }

      const int degree = static_cast<int>(std::lround(values[0]));

      Eigen::Matrix<double, 3, 4> jt;
      int idx = 1;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
          jt(r, c) = values[idx++];
        }
      }

      rows.emplace_back(degree, jt);
    }

    if (rows.empty()) {
      throw std::runtime_error("Jacobian CSV has no data rows: " + csv_path);
    }

    std::sort(rows.begin(), rows.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    min_degree_ = rows.front().first;
    max_degree_ = rows.back().first;

    // Dense degree indexing enables O(1) lookup after nearest-degree mapping.
    // Pre-allocating the the lookup table storage makes lookup fast yippee
    jacobians_.assign(
      static_cast<std::size_t>(max_degree_ - min_degree_ + 1),
      Eigen::Matrix<double, 3, 4>::Zero());

    for (const auto& [deg, jt] : rows) {
      jacobians_[static_cast<std::size_t>(deg - min_degree_)] = jt;
    }
  }

  int JacobianLookup::nearest_degree(float angle_deg) const
  {
    // Nearest 1-degree interval, then clamp to the valid CSV range.
    const int rounded = static_cast<int>(std::lround(angle_deg));
    return std::clamp(rounded, min_degree_, max_degree_);
  }

  const Eigen::Matrix<double, 3, 4>& JacobianLookup::jacobian_for_angle_deg(float angle_deg) const
  {
    const int deg = nearest_degree(angle_deg);
    return jacobians_[static_cast<std::size_t>(deg - min_degree_)];
  }

}