#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_all.hpp"

#include "fingerlib/jacobian_lookup.hpp"

#include <filesystem>

TEST_CASE("JacobianLookup rounds to nearest degree", "[jacobian_lookup]")
{
    const std::filesystem::path csv =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "jacobian_transposes.csv";

    fingerlib::JacobianLookup lookup(csv.string());

    const auto& j0 = lookup.jacobian_for_angle_deg(0.4f);
    const auto& j1 = lookup.jacobian_for_angle_deg(0.6f);

    REQUIRE(j0(2, 0) == Catch::Approx(-0.041554279));
    REQUIRE(j1(2, 0) == Catch::Approx(-0.041165479));
}

TEST_CASE("JacobianLookup clamps out-of-range angles", "[jacobian_lookup]")
{
    const std::filesystem::path csv =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "jacobian_transposes.csv";

    fingerlib::JacobianLookup lookup(csv.string());

    REQUIRE(lookup.nearest_degree(-15.0f) == 0);
    REQUIRE(lookup.nearest_degree(120.0f) == 90);

    const auto& low = lookup.jacobian_for_angle_deg(-15.0f);
    const auto& high = lookup.jacobian_for_angle_deg(120.0f);

    REQUIRE(low(2, 0) == Catch::Approx(-0.041554279));
    REQUIRE(high(2, 0) == Catch::Approx(-0.038331365));
}