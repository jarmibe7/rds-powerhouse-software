#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_all.hpp"

#include <Eigen/Dense>

#include "fingerlib/nnls.h"

TEST_CASE("NNLS recovers exact nonnegative solution", "[nnls]")
{
    const Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d b(1.0, 2.0, 3.0);

    fingerlib::NNLS<Eigen::Matrix3d> solver(A);
    const auto& x = solver.solve(b);

    REQUIRE(solver.info() == Eigen::Success);
    REQUIRE(x(0) == Catch::Approx(1.0).margin(1e-12));
    REQUIRE(x(1) == Catch::Approx(2.0).margin(1e-12));
    REQUIRE(x(2) == Catch::Approx(3.0).margin(1e-12));
}

TEST_CASE("NNLS clamps negative unconstrained components", "[nnls]")
{
    const Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d b(1.0, -2.0, 3.0);

    fingerlib::NNLS<Eigen::Matrix3d> solver(A);
    const auto& x = solver.solve(b);

    REQUIRE(solver.info() == Eigen::Success);
    REQUIRE(x(0) == Catch::Approx(1.0).margin(1e-12));
    REQUIRE(x(1) == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(x(2) == Catch::Approx(3.0).margin(1e-12));
}

TEST_CASE("NNLS solves small overdetermined system", "[nnls]")
{
    Eigen::Matrix<double, 3, 2> A;
    A << 1.0, 0.0,
         0.0, 1.0,
         1.0, 1.0;

    const Eigen::Vector3d b(1.0, 1.0, 3.0);

    fingerlib::NNLS<Eigen::Matrix<double, 3, 2>> solver(A);
    const auto& x = solver.solve(b);

    REQUIRE(solver.info() == Eigen::Success);
    REQUIRE(x(0) == Catch::Approx(4.0 / 3.0).margin(1e-10));
    REQUIRE(x(1) == Catch::Approx(4.0 / 3.0).margin(1e-10));
    REQUIRE((A * x - b).norm() == Catch::Approx(0.57735026919).margin(1e-9));
}

TEST_CASE("NNLS returns zero for zero rhs", "[nnls]")
{
    const Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d b = Eigen::Vector3d::Zero();

    fingerlib::NNLS<Eigen::Matrix3d> solver(A);
    const auto& x = solver.solve(b);

    REQUIRE(solver.info() == Eigen::Success);
    REQUIRE(x.norm() == Catch::Approx(0.0).margin(1e-12));
}
