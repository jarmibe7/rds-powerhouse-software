/* Non-Negative Least Squares (NNLS) for Eigen.
 *
 * Self-contained implementation requiring only Eigen/Core and Eigen/QR.
 * Compatible with Eigen 3.4.0. No unsupported module or version upgrade needed.
 *
 * Solves:  min ||Ax - b||^2  subject to  x >= 0
 *
 * Based on the Lawson-Hanson active-set algorithm described in:
 *   "Solving Least Squares Problems", Lawson & Hanson, Prentice-Hall, 1974.
 *
 * Original authors:
 *   Copyright (C) 2021 Essex Edwards <essex.edwards@gmail.com>
 *   Copyright (C) 2013 Hannes Matuschek <hannes.matuschek@uni-potsdam.de>
 * Internal Householder helper sourced from Eigen/src/QR/HouseholderQR.h:
 *   Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
 * Code Conversion and Refactoring for FingerLib:
 *   Claude, Anthropic LLM (https://claude.ai) with human guidance/review/edits by Jared Berry
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef FINGERLIB_NNLS_H
#define FINGERLIB_NNLS_H

#include <Eigen/Core>
#include <Eigen/QR>

namespace fingerlib {

/**
 * @brief Incrementally updates an in-place QR factorization by one column.
 *
 * Assumes the first @p k columns of @p mat already hold the compact QR
 * factorization of the first @p k columns of the inactive-set submatrix.
 * Appends @p new_col as column @p k and updates the factorization in-place
 * using a new Householder reflector.
 *
 * This replaces `Eigen::internal::householder_qr_inplace_update`, which is
 * not available in Eigen 3.4.0.
 *
 * @tparam MatrixQR  Type of the QR storage matrix.
 * @tparam HCoeffs   Type of the Householder coefficient vector.
 * @tparam VectorQR  Type of the new column vector.
 *
 * @param mat       In/out. QR storage matrix (m x n). First k cols on entry
 *                  hold the existing factorization; col k is written on exit.
 * @param h_coeffs  In/out. Householder coefficients. Entry k is written.
 * @param new_col   The new column of A^N to append (length m).
 * @param k         Zero-based index of the column being appended.
 * @param tmp       Caller-supplied scratch buffer of length >= n.
 */
template <typename MatrixQR, typename HCoeffs, typename VectorQR>
static void qr_update(MatrixQR& mat, HCoeffs& h_coeffs,
                      const VectorQR& new_col,
                      typename MatrixQR::Index k,
                      typename MatrixQR::Scalar* tmp)
{
  typedef typename MatrixQR::Index      Index;
  typedef typename MatrixQR::RealScalar RealScalar;

  const Index rows = mat.rows();
  mat.col(k) = new_col;

  // Apply existing reflectors H_0 ... H_{k-1} to the new column.
  for (Index i = 0; i < k; ++i) {
    const Index rem = rows - i;
    mat.col(k).tail(rem).applyHouseholderOnTheLeft(
        mat.col(i).tail(rem - 1), h_coeffs.coeffRef(i), tmp + i + 1);
  }

  // Compute and store the new reflector H_k.
  RealScalar beta;
  mat.col(k).tail(rows - k).makeHouseholderInPlace(h_coeffs.coeffRef(k), beta);
  mat.coeffRef(k, k) = beta;
}

/**
 * @brief Non-Negative Least Squares solver (Lawson-Hanson active-set method).
 *
 * Solves the constrained least-squares problem:
 * @code
 *   min ||Ax - b||^2   subject to   x >= 0
 * @endcode
 *
 * The algorithm maintains a partition of the variables into an *active set*
 * (pinned to zero) and an *inactive set* (solved freely). At each outer
 * iteration the variable with the steepest gradient is freed; at each inner
 * iteration the unconstrained subproblem is solved and any variable that
 * goes negative is re-pinned. An incremental QR factorization makes the
 * inner solves efficient.
 *
 * **To enforce x >= lower_bound instead of x >= 0**, shift the problem:
 * @code
 *   Eigen::VectorXd shift = Eigen::VectorXd::Constant(n, lower_bound);
 *   solver.solve(b - A * shift);
 *   Eigen::VectorXd x = solver.x().array() + lower_bound;
 * @endcode
 *
 * @tparam MatrixType  Dense Eigen matrix type for A (e.g. `Eigen::MatrixXd`).
 *                     Must have a real scalar type.
 *
 * ### Example
 * @code
 *   fingerlib::NNLS<Eigen::MatrixXd> solver(A);
 *   solver.solve(b);
 *   if (solver.info() == Eigen::Success)
 *       std::cout << solver.x() << "\n";
 * @endcode
 */
template <typename MatrixType>
class NNLS
{
public:
  using Scalar    = typename MatrixType::Scalar;
  using Index     = typename MatrixType::Index;
  using ColVec   = Eigen::Matrix<Scalar, MatrixType::ColsAtCompileTime, 1>;
  using RowVec   = Eigen::Matrix<Scalar, MatrixType::RowsAtCompileTime, 1>;
  using IndexVec = Eigen::Matrix<Index,  MatrixType::ColsAtCompileTime, 1>;

  // -------------------------------------------------------------------------

  /**
   * @brief Construct and initialise with system matrix A.
   *
   * @param a         The system matrix (m x n).
   * @param max_iter  Max inner iterations. Defaults to 2*n if negative.
   * @param tol       Gradient convergence tolerance.
   */
  explicit NNLS(const MatrixType& a,
                Index  max_iter = -1,
                Scalar tol      = Eigen::NumTraits<Scalar>::dummy_precision())
    : max_iter_(max_iter), tol_(tol)
  {
    EIGEN_STATIC_ASSERT(!Eigen::NumTraits<Scalar>::IsComplex,
                        NUMERIC_TYPE_MUST_BE_REAL);
    a_   = a;
    ata_ = a_.transpose() * a_;
    allocate_();
  }

  // -------------------------------------------------------------------------

  /**
   * @brief Solve for a given right-hand side b.
   *
   * May be called repeatedly with different b vectors without rebuilding A.
   *
   * @param b  Right-hand side vector (length m).
   * @return   Const reference to the solution vector x (length n).
   *           Check info() to confirm convergence.
   */
  const ColVec& solve(const RowVec& b)
  {
    const Index n        = a_.cols();
    const Index max_iter = max_iter_ < 0 ? 2 * n : max_iter_;

    // Initialise.
    iters_     = 0;
    info_      = Eigen::NumericalIssue;
    x_.setZero();
    idx_       = IndexVec::LinSpaced(n, 0, n - 1);  // identity permutation
    n_inactive_ = 0;

    atb_.noalias() = a_.transpose() * b;

    // Outer loop: free one variable per iteration.
    while (true) {
      if (n_inactive_ == n) { info_ = Eigen::Success; return x_; }

      // Gradient w = A^T b - A^T A x; find max over active variables.
      grad_.noalias() = atb_ - ata_ * x_;
      Index arg_max = -1;
      const Scalar max_g = grad_(idx_.tail(n - n_inactive_)).maxCoeff(&arg_max);
      arg_max += n_inactive_;

      if (max_g < tol_) { info_ = Eigen::Success; return x_; }

      move_to_inactive_(arg_max);  // free the most promising variable

      // Inner loop: fix infeasible variables.
      while (true) {
        if (iters_ >= max_iter) { info_ = Eigen::NoConvergence; return x_; }

        solve_inactive_(b);  // unconstrained solve on inactive set -> y_
        ++iters_;

        // Find the most-violated variable (if any).
        bool   ok    = true;
        Scalar alpha = Eigen::NumTraits<Scalar>::highest();
        Index  hit   = -1;
        for (Index i = 0; i < n_inactive_; ++i) {
          const Index j = idx_[i];
          if (y_(j) < Scalar(0)) {
            const Scalar t = -x_(j) / (y_(j) - x_(j));
            if (t < alpha) { alpha = t; hit = i; ok = false; }
          }
        }

        if (ok) { x_ = y_; break; }  // feasible: accept and go outer

        // Interpolate to boundary, then re-pin the violated variable.
        for (Index i = 0; i < n_inactive_; ++i)
          x_(idx_[i]) += alpha * (y_(idx_[i]) - x_(idx_[i]));

        move_to_active_(hit);
      }
    }
  }

  // -------------------------------------------------------------------------
  // Accessors

  /** @brief Solution vector from the last solve(). */
  const ColVec& x() const { return x_; }

  /**
   * @brief Solver status after the last solve().
   * @return `Eigen::Success`, `Eigen::NoConvergence`, or `Eigen::NumericalIssue`.
   */
  Eigen::ComputationInfo info() const { return info_; }

  /** @brief Number of inner iterations performed during the last solve(). */
  Index iterations() const { return iters_; }

private:
  // -------------------------------------------------------------------------
  // Helpers

  /** Resize all internal storage to match a_ dimensions. */
  void allocate_()
  {
    const Index n = a_.cols(), m = a_.rows();
    x_.resize(n);    grad_.resize(n); y_.resize(n);   atb_.resize(n);
    idx_.resize(n);  qr_.resize(m, n); qrc_.resize(n);
    tmp_c_.resize(n); tmp_r_.resize(m);
  }

  /** Move variable at position @p pos in idx_ from active -> inactive. */
  void move_to_inactive_(Index pos)
  {
    std::swap(idx_(pos), idx_(n_inactive_));
    qr_update(qr_, qrc_, a_.col(idx_(n_inactive_)), n_inactive_, tmp_c_.data());
    ++n_inactive_;
  }

  /** Move variable at position @p pos in idx_ from inactive -> active. */
  void move_to_active_(Index pos)
  {
    std::swap(idx_(pos), idx_(n_inactive_ - 1));
    --n_inactive_;
    for (Index i = pos; i < n_inactive_; ++i)
      qr_update(qr_, qrc_, a_.col(idx_(i)), i, tmp_c_.data());
  }

  /** Solve the unconstrained LS on the inactive set; result goes to y_. */
  void solve_inactive_(const RowVec& b)
  {
    // Q^T b
    tmp_r_ = b;
    tmp_r_.applyOnTheLeft(
      Eigen::householderSequence(
        qr_.leftCols(n_inactive_),
        qrc_.head(n_inactive_)).transpose());

    // R x = Q^T b  (back-substitution on inactive block)
    tmp_c_.head(n_inactive_) =
      qr_.topLeftCorner(n_inactive_, n_inactive_)
         .template triangularView<Eigen::Upper>()
         .solve(tmp_r_.head(n_inactive_));

    tmp_c_.tail(y_.size() - n_inactive_).setZero();

    // Permute back to original column order.
    y_.noalias() = idx_.asPermutation() * tmp_c_.head(y_.size());
  }

  // -------------------------------------------------------------------------
  // Data

  using Mat_ata = Eigen::Matrix<Scalar,
                                MatrixType::ColsAtCompileTime,
                                MatrixType::ColsAtCompileTime>;

  Index                  max_iter_;   ///< iteration cap (-1 = auto)
  Scalar                 tol_;        ///< gradient convergence threshold
  Index                  iters_;      ///< inner iterations in last solve()
  Index                  n_inactive_; ///< current inactive-set size
  Eigen::ComputationInfo info_;       ///< result of last solve()

  MatrixType a_;    ///< system matrix copy
  Mat_ata    ata_;  ///< precomputed A^T A

  ColVec   x_;     ///< solution
  ColVec   grad_;  ///< gradient A^T b - A^T A x
  ColVec   y_;     ///< candidate inactive-set solution
  ColVec   atb_;   ///< precomputed A^T b
  IndexVec idx_;   ///< permutation: [inactive | active]
  MatrixType qr_;  ///< incremental QR of inactive columns
  ColVec   qrc_;   ///< Householder coefficients
  ColVec   tmp_c_; ///< scratch (cols-sized)
  RowVec   tmp_r_; ///< scratch (rows-sized)
};

}

#endif