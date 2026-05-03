#include "adas/lateral_mpc.hpp"

#include <algorithm>


namespace lateral_control {

LateralMPC::LateralMPC(
    double ts,
    double tau,
    int p,
    int c,
    double L,
    double s,
    const std::vector<double>& q_vals,
    const Limit& u_limits
)
    : ts{ts}
    , tau{tau}
    , p{p}
    , c{c}
    , L{L}
    , s{s}
    , u_limits{u_limits}
{
    // F_hat: C*H tiled p times (C = I, H = I → each block is identity)
    F_hat = Eigen::MatrixXd::Zero(n * p, n);
    for (int i = 0; i < p; ++i) {
        F_hat.block(i * n, 0, n, n) = Eigen::Matrix<double, n, n>::Identity();
    }

    // Q: block-diagonal output weight matrix
    const Eigen::Vector<double, n> qw = Eigen::VectorXd::Map(q_vals.data(), n);
    const Eigen::Matrix<double, n, n> q = qw.asDiagonal();
    Q = Eigen::MatrixXd::Zero(n * p, n * p);
    for (int i = 0; i < p; ++i) {
        Q.block(i * n, i * n, n, n) = q;
    }

    // H1: tridiagonal smoothness matrix — penalises changes between consecutive inputs
    H1 = Eigen::MatrixXd::Zero(c, c);
    for (int i = 0; i < c; ++i) {
        for (int j = 0; j < c; ++j) {
            if (i == j)
                H1(i, j) = (i == c - 1) ? s : 2.0 * s;
            if (i == j + 1 || j == i + 1)
                H1(i, j) = -s;
        }
    }

    solver.settings()->setTimeLimit(0.04);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(c);
    solver.data()->setNumberOfConstraints(c);
}


std::pair<double, const Eigen::Vector<double, LateralMPC::n>> LateralMPC::calculate_control(
    double e,
    double theta,
    double delta,
    const std::vector<double>& kappa_ref,
    double vx)
{
    const Eigen::Vector<double, n> x{e, theta, delta};

    // Prediction error from last step (disturbance observer / observer correction)
    Eigen::Vector<double, n> ex;
    if (!x_predicted) {
        ex = decltype(ex)::Zero();
    } else {
        ex = x - *x_predicted;
    }

    // ── Build LTV matrices at each prediction step ──────────────────────────
    std::vector<Eigen::Matrix<double, n, n>> A_seq(p);
    std::vector<Eigen::Vector<double, n>>   B_seq(p);
    std::vector<Eigen::Vector<double, n>>   W_seq(p);

    for (int k = 0; k < p; ++k) {
        const double kappa     = kappa_ref[k];
        const double delta_ref = std::atan(L * kappa);
        const double cos2      = std::pow(std::cos(delta_ref), 2);

        Eigen::Matrix<double, n, n> A_k;
        A_k << 1,  vx * ts,                    0,
               0,  1,        -vx * ts / L / cos2,
               0,  0,                 1 - ts/tau;
        A_seq[k] = A_k;

        B_seq[k] = Eigen::Vector<double, n>(0.0, 0.0, ts / tau);

        // Affine offset from linearisation around delta_ref:
        //   θ̇ = -vx/L/cos²(δ_ref) · (δ - δ_ref)
        //   → discrete W_θ = +vx · δ_ref · ts / (L · cos²(δ_ref))
        W_seq[k] = Eigen::Vector<double, n>(0.0, vx * delta_ref * ts / L / cos2, 0.0);
    }

    // ── A_hat: compound transition Φ(i+1,0) = A_i · A_{i-1} · … · A_0 ─────
    A_hat = Eigen::MatrixXd::Zero(n * p, n);
    {
        Eigen::Matrix<double, n, n> A_prod = Eigen::Matrix<double, n, n>::Identity();
        for (int i = 0; i < p; ++i) {
            A_prod = A_seq[i] * A_prod;
            A_hat.block(i * n, 0, n, n) = A_prod;
        }
    }

    // ── D_hat: control influence  D_hat[i, m] = Φ(i+1, m+1) · B[m] ───────
    //
    // Φ(i+1, m+1) = A[i] · A[i-1] · … · A[m+1]   (identity when m == i)
    //
    // For each prediction row i, iterate m from min(i, c-1) down to 0,
    // accumulating phi = Φ(i+1, m+1) via right-multiplication by A[m].
    // When i >= c, pre-accumulate Φ(i+1, c) for the steps beyond the
    // control horizon.
    D_hat = Eigen::MatrixXd::Zero(n * p, c);
    for (int i = 0; i < p; ++i) {
        Eigen::Matrix<double, n, n> phi = Eigen::Matrix<double, n, n>::Identity();
        for (int k = i; k >= c; --k) {
            phi = phi * A_seq[k];
        }
        for (int m = std::min(i, c - 1); m >= 0; --m) {
            D_hat.block(i * n, m, n, 1) = phi * B_seq[m];
            phi = phi * A_seq[m];
        }
    }

    // ── W_hat: accumulated affine offsets through the prediction horizon ─────
    //
    // x(i+1) = Φ(i+1,0)·x + D·U + W_hat[i] + F_hat·ex
    //
    // Recursion: w_hat[0] = W_seq[0]
    //            w_hat[i] = A_seq[i] · w_hat[i-1] + W_seq[i]
    W_hat = Eigen::VectorXd::Zero(n * p);
    {
        Eigen::Vector<double, n> w = W_seq[0];
        W_hat.segment(0, n) = w;
        for (int i = 1; i < p; ++i) {
            w = A_seq[i] * w + W_seq[i];
            W_hat.segment(i * n, n) = w;
        }
    }

    // ── QP formulation ───────────────────────────────────────────────────────
    //
    // Free response (U = 0): F2 = A_hat·x + W_hat + F_hat·ex
    // Target: Y → 0  ⟹  b1 = -F2
    //
    // min  0.5 U^T (H1 + D^T Q D) U  +  (D^T Q F2 - M2·u_prev)^T U
    //  s.t. u_min ≤ U ≤ u_max

    const Eigen::VectorXd F2 = A_hat * x + W_hat + F_hat * ex;
    const Eigen::VectorXd b1 = -F2;

    Eigen::VectorXd M2 = Eigen::VectorXd::Zero(c);
    M2(0) = s;

    const Eigen::VectorXd g1 = -M2 * u_prev;
    const Eigen::VectorXd g2 = -D_hat.transpose() * Q * b1;
    const Eigen::MatrixXd H2 = D_hat.transpose() * Q * D_hat;

    Eigen::MatrixXd Hqp = H1 + H2;
    Eigen::VectorXd g   = g1 + g2;

    Eigen::VectorXd u_min = Eigen::VectorXd::Constant(c, u_limits.min);
    Eigen::VectorXd u_max = Eigen::VectorXd::Constant(c, u_limits.max);

    Eigen::SparseMatrix<double> constr_matrix =
        Eigen::MatrixXd::Identity(c, c).sparseView();
    Eigen::SparseMatrix<double> Hqp_sparse = Hqp.sparseView();

    if (!x_predicted) {
        solver.data()->setHessianMatrix(Hqp_sparse);
        solver.data()->setGradient(g);
        solver.data()->setLinearConstraintsMatrix(constr_matrix);
        solver.data()->setLowerBound(u_min);
        solver.data()->setUpperBound(u_max);
        solver.settings()->setWarmStart(false);
        solver.initSolver();
    } else {
        solver.updateHessianMatrix(Hqp_sparse);
        solver.updateGradient(g);
        solver.updateLinearConstraintsMatrix(constr_matrix);
        solver.updateBounds(u_min, u_max);
    }

    solver.solveProblem();
    double res = solver.getSolution()[0];

    if (solver.getStatus() != OsqpEigen::Status::Solved) {
        res = u_prev;
        solver.settings()->setWarmStart(false);
    } else {
        solver.settings()->setWarmStart(true);
    }

    res = std::clamp(res, u_limits.min, u_limits.max);

    // ── One-step prediction for next call (H = I) ───────────────────────────
    x_predicted = A_seq[0] * x + B_seq[0] * res + W_seq[0] + ex;
    u_prev = res;

    return std::make_pair(res, x);
}

}  // namespace lateral_control
