#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_all.hpp"

#include "fingerlib/kinematics.hpp"
#include "fingerlib/constants.hpp"

TEST_CASE("DIP angle at zero PIP", "[solve_dip_from_pip]")
{
    const double dip = fingerlib::solve_dip_from_pip(0.0, +1);

    REQUIRE(dip == Catch::Approx(0.0).margin(2e-1));
}

TEST_CASE("DIP angle at 45 deg PIP", "[solve_dip_from_pip]")
{
    const double dip = fingerlib::solve_dip_from_pip(45.0, +1);

    REQUIRE(dip == Catch::Approx(fingerlib::deg2rad(45.0)).margin(2e-1));
}

TEST_CASE("DIP angle at 90 deg PIP", "[solve_dip_from_pip]")
{
    const double dip = fingerlib::solve_dip_from_pip(90.0, +1);

    REQUIRE(dip == Catch::Approx(fingerlib::deg2rad(90.0)).margin(2e-1));
}

TEST_CASE("DIP angle is continuous over PIP range", "[solve_dip_from_pip]")
{
    const double step = 1.0;
    double prev = fingerlib::solve_dip_from_pip(0.0, +1);

    for (double pip = step; pip <= 90.0; pip += step) {
        const double curr = fingerlib::solve_dip_from_pip(pip, +1);
        REQUIRE(std::abs(curr - prev) < fingerlib::deg2rad(2.0));   // TODO: Better way to check if continuous? 2 deg deviation is arbitrary
        prev = curr;
    }
}

TEST_CASE("Fingertip pose at zero configuration matches URDF offsets", "[fingertip_pose]")
{
    const Eigen::Vector3d q = Eigen::Vector3d::Zero();
    const auto pose = fingerlib::fingertip_pose(q);

    // q_dip is set by four-bar closure, so zero actuated joints do not imply identity distal orientation.
    REQUIRE(pose.translation().x() == Catch::Approx(0.167497957).margin(1e-5));
    REQUIRE(pose.translation().y() == Catch::Approx(0.0033).margin(1e-5));
    REQUIRE(pose.translation().z() == Catch::Approx(0.0979498911).margin(1e-5));
    REQUIRE(pose.linear().transpose().isApprox(pose.linear().inverse(), 1e-12));
    REQUIRE(pose.linear().determinant() == Catch::Approx(1.0).margin(1e-12));
}

TEST_CASE("Fingertip inverse kinematics is locally consistent", "[fingertip_pose]")
{
    const Eigen::Vector3d q_target(0.05, 0.2, 0.35);
    const auto desired_pose = fingerlib::fingertip_pose(q_target);

    fingerlib::FingertipTrackingOptions options;
    options.max_iterations = 20;
    options.tolerance = 1e-8;

    const auto q_solved = fingerlib::fingertip_inverse_kinematics(desired_pose, Eigen::Vector3d::Zero(), options);
    const auto solved_pose = fingerlib::fingertip_pose(q_solved);

    REQUIRE((q_solved - q_target).norm() < 5e-3);
    REQUIRE((solved_pose.translation() - desired_pose.translation()).norm() < 5e-4);
}