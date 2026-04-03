#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_all.hpp"

#include "fingerlib/kinematics.hpp"
#include "fingerlib/constants.hpp"

TEST_CASE("DIP angle at zero PIP", "[solve_dip_from_pip]")
{
    const double dip = fingerlib::solve_dip_from_pip(0.0, +1);

    REQUIRE(dip == Catch::Approx(0.0).margin(1e-4));
}

TEST_CASE("DIP angle at 45 deg PIP", "[solve_dip_from_pip]")
{
    const double dip = fingerlib::solve_dip_from_pip(45.0, +1);

    REQUIRE(dip == Catch::Approx(fingerlib::deg2rad(45.0)).margin(1e-4));
}

TEST_CASE("DIP angle at 90 deg PIP", "[solve_dip_from_pip]")
{
    const double dip = fingerlib::solve_dip_from_pip(90.0, +1);

    REQUIRE(dip == Catch::Approx(fingerlib::deg2rad(90.0)).margin(1e-4));
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