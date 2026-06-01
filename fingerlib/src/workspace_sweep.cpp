#include "fingerlib/kinematics.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct SweepRange {
  double min_deg;
  double max_deg;
  double step_deg;
};

struct Sample {
  Eigen::Vector3d q_deg;
  Eigen::Vector3d position_m;
};

void validate_range(const SweepRange& range, const char* name)
{
  if (range.step_deg <= 0.0) {
    throw std::runtime_error(std::string(name) + " step must be positive");
  }
  if (range.max_deg < range.min_deg) {
    throw std::runtime_error(std::string(name) + " max must be >= min");
  }
}

std::vector<Sample> sweep_workspace(std::size_t& skipped_samples)
{
  const SweepRange q0{-10.0, 10.0, 1.0};
  const SweepRange q1{0.0, 90.0, 2.0};
  const SweepRange q2{0.0, 90.0, 2.0};
  const double tip_offset_m = fingerlib::FINGERTIP_DEFAULT_OFFSET_M;
  const int branch = +1;

  validate_range(q0, "q0");
  validate_range(q1, "q1");
  validate_range(q2, "q2");

  std::vector<Sample> samples;
  skipped_samples = 0;

  // Sweep angles for all joints and get positions
  for (double q0_deg = q0.min_deg; q0_deg <= q0.max_deg + 1.0e-12; q0_deg += q0.step_deg) {
    for (double q1_deg = q1.min_deg; q1_deg <= q1.max_deg + 1.0e-12; q1_deg += q1.step_deg) {
      for (double q2_deg = q2.min_deg; q2_deg <= q2.max_deg + 1.0e-12; q2_deg += q2.step_deg) {
        const Eigen::Vector3d q_rad(
          fingerlib::deg2rad(q0_deg),
          fingerlib::deg2rad(q1_deg),
          fingerlib::deg2rad(q2_deg));

        const Eigen::Vector3d position = fingerlib::fingertip_position(q_rad, tip_offset_m, branch);
        if (!position.allFinite()) {
          ++skipped_samples;
          continue;
        }

        samples.push_back(Sample{
          Eigen::Vector3d(q0_deg, q1_deg, q2_deg),
          position,
        });
      }
    }
  }

  return samples;
}

struct Bounds {
  Eigen::Vector3d min{Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())};
  Eigen::Vector3d max{Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity())};
};

Bounds compute_bounds(const std::vector<Sample>& samples)
{
  Bounds bounds;
  for (const auto& sample : samples) {
    bounds.min = bounds.min.cwiseMin(sample.position_m);
    bounds.max = bounds.max.cwiseMax(sample.position_m);
  }
  return bounds;
}

}

int main(int argc, char* argv[])
{
  try {
    std::size_t skipped_samples = 0;
    const std::vector<Sample> samples = sweep_workspace(skipped_samples);
    if (samples.empty()) {
      std::cerr << "No valid workspace samples were generated.\n";
      return 1;
    }

    const std::filesystem::path default_output_path =
      std::filesystem::path(__FILE__).parent_path() / "workspace_samples.csv";
    const std::filesystem::path output_path = (argc > 1) ? argv[1] : default_output_path;
    std::ofstream out(output_path);
    if (!out.is_open()) {
      std::cerr << "Failed to open output file: " << output_path.string() << '\n';
      return 1;
    }

    out << std::fixed << std::setprecision(9);
    out << "q0_deg,q1_deg,q2_deg,x_m,y_m,z_m\n";
    for (const auto& sample : samples) {
      out << sample.q_deg.x() << ','
          << sample.q_deg.y() << ','
          << sample.q_deg.z() << ','
          << sample.position_m.x() << ','
          << sample.position_m.y() << ','
          << sample.position_m.z() << '\n';
    }

    const Bounds bounds = compute_bounds(samples);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Wrote " << samples.size() << " samples to " << output_path.string() << '\n';
    if (skipped_samples > 0) {
      std::cout << "Skipped " << skipped_samples << " invalid samples\n";
    }
    std::cout << "Workspace bounds:\n"
              << "  x in [" << bounds.min.x() << ", " << bounds.max.x() << "] m\n"
              << "  y in [" << bounds.min.y() << ", " << bounds.max.y() << "] m\n"
              << "  z in [" << bounds.min.z() << ", " << bounds.max.z() << "] m\n";

    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}