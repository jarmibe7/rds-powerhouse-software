#ifndef FINGERLIB_PD_CONTROLLER_HPP_INCLUDE_GUARD
#define FINGERLIB_PD_CONTROLLER_HPP_INCLUDE_GUARD

#include <Eigen/Core>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace fingerlib {

/// \brief Simple joint-space PD controller for an N-DOF finger.
/// \tparam N Number of joints (compile-time).
template <int N>
class PDController {
public:
    using Vec = Eigen::Matrix<double, N, 1>;

    /// \brief Construct with scalar gains applied identically to every joint.
    /// \param kp Proportional gain  [N·m / rad]
    /// \param kd Derivative gain    [N·m·s / rad]
    /// \param tau_max Per-joint torque clamp [N·m]  (must be > 0)
    PDController(double kp, double kd, double tau_max)
        : kp_(Vec::Constant(kp))
        , kd_(Vec::Constant(kd))
        , tau_max_(Vec::Constant(tau_max))
        , q_des_(Vec::Zero())
        , dq_des_(Vec::Zero())
        , tau_ff_(Vec::Zero())
    {
        validate();
    }

    /// \brief Construct with per-joint gain vectors.
    /// \param kp Proportional gains [N·m / rad], length N
    /// \param kd Derivative gains   [N·m·s / rad], length N
    /// \param tau_max Per-joint torque clamps [N·m], length N (all > 0)
    PDController(const Vec& kp, const Vec& kd, const Vec& tau_max)
        : kp_(kp)
        , kd_(kd)
        , tau_max_(tau_max)
        , q_des_(Vec::Zero())
        , dq_des_(Vec::Zero())
        , tau_ff_(Vec::Zero())
    {
        validate();
    }

    // ── Setters ──────────────────────────────────────────────────────────────

    /// \brief Set the desired joint positions [rad].
    void set_target(const Vec& q_des) { q_des_ = q_des; }

    /// \brief Set desired positions and velocities [rad], [rad/s].
    void set_target(const Vec& q_des, const Vec& dq_des) {
        q_des_  = q_des;
        dq_des_ = dq_des;
    }

    /// \brief Set a constant feed-forward torque [N·m] added before clamping.
    /// Pass Vec::Zero() to disable.
    void set_feedforward(const Vec& tau_ff) { tau_ff_ = tau_ff; }

    /// \brief Update proportional gains at runtime (e.g. gain scheduling).
    void set_kp(const Vec& kp) { kp_ = kp; validate(); }
    void set_kd(const Vec& kd) { kd_ = kd; validate(); }

    // ── Compute ───────────────────────────────────────────────────────────────

    /// \brief Compute clamped torque commands.
    /// \param q   Measured joint positions [rad], length N
    /// \param dq  Measured joint velocities [rad/s], length N
    /// \return    Torque vector [N·m], length N, clamped to ±tau_max
    Vec compute(const Vec& q, const Vec& dq) const {
        const Vec pos_error = q_des_ - q;
        const Vec vel_error = dq_des_ - dq;
        const Vec tau_raw  = kp_.cwiseProduct(pos_error) + kd_.cwiseProduct(vel_error) + tau_ff_;
        return tau_raw.cwiseMax(-tau_max_).cwiseMin(tau_max_);
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    const Vec& target_position() const { return q_des_; }
    const Vec& target_velocity() const { return dq_des_; }
    const Vec& kp()              const { return kp_; }
    const Vec& kd()              const { return kd_; }
    const Vec& tau_max()         const { return tau_max_; }

private:
    void validate() const {
        if ((tau_max_.array() <= 0.0).any())
            throw std::invalid_argument("PDController: tau_max must be > 0 for all joints");
        if ((kp_.array() < 0.0).any() || (kd_.array() < 0.0).any())
            throw std::invalid_argument("PDController: gains must be non-negative");
    }

    Vec kp_;
    Vec kd_;
    Vec tau_max_;
    Vec q_des_;
    Vec dq_des_;
    Vec tau_ff_;
};

// ── Convenience alias for powerhouse finger ─────────────────────────────────────
// TODO: Make 3
using FingerPDController = PDController<4>;

}

#endif