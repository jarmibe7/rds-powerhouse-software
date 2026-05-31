#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_all.hpp"

#include "fingerlib/workspace_lookup.hpp"

#include <Eigen/Dense>

#include <filesystem>

TEST_CASE("WorkspaceLookup returns nearest sample on the sampled grid", "[workspace_lookup]")
{
    const std::filesystem::path csv =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "workspace_samples.csv";

    fingerlib::WorkspaceLookup lookup(csv.string());

    const Eigen::Vector3d query(-9.6, 1.2, 1.1);
    const auto nearest = lookup.nearest_joint_degrees(query);
    const auto& sample = lookup.sample_for_joint_angles_deg(query);

    REQUIRE(nearest.x() ==  -10);
    REQUIRE(nearest.y() ==    2);
    REQUIRE(nearest.z() ==    2);

    REQUIRE(sample.q_deg.x() == Catch::Approx(-10.0));
    REQUIRE(sample.q_deg.y() == Catch::Approx(2.0));
    REQUIRE(sample.q_deg.z() == Catch::Approx(2.0));
    REQUIRE(sample.position_m.x() == Catch::Approx(0.165510147));
    REQUIRE(sample.position_m.y() == Catch::Approx(-0.022394620));
    REQUIRE(sample.position_m.z() == Catch::Approx(0.089695705));

    const auto position = lookup.sample_workspace(query);
    REQUIRE(position.x() == Catch::Approx(sample.position_m.x()));
    REQUIRE(position.y() == Catch::Approx(sample.position_m.y()));
    REQUIRE(position.z() == Catch::Approx(sample.position_m.z()));
}

TEST_CASE("WorkspaceLookup clamps out-of-range joint angles", "[workspace_lookup]")
{
    const std::filesystem::path csv =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "workspace_samples.csv";

    fingerlib::WorkspaceLookup lookup(csv.string());

    const Eigen::Vector3d query(-20.0, -1.0, -3.0);
    const auto nearest = lookup.nearest_joint_degrees(query);
    const auto& sample = lookup.sample_for_joint_angles_deg(query);

    REQUIRE(nearest.x() == -10);
    REQUIRE(nearest.y() ==   0);
    REQUIRE(nearest.z() ==   0);

    REQUIRE(sample.position_m.x() == Catch::Approx(0.165822574));
    REQUIRE(sample.position_m.y() == Catch::Approx(-0.022449710));
    REQUIRE(sample.position_m.z() == Catch::Approx(0.097949891));
}