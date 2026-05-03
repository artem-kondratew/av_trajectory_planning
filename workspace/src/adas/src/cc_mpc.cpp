#include "adas/cc_mpc.hpp"

#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>


namespace cruise_control {

CruiseController::CruiseController(
    double ts,
    double tau,
    int p,
    int c,
    double s,
    const std::vector<double>& phi_vals,
    const std::vector<double>& q_vals,
    const Limit& a_limits,
    const Limit& j_limits,
    const Limit& u_limits
)
    : ts{ts}
    , tau{tau}
    , p{p}
    , c{c}
    , s{s} {

    A = Eigen::Matrix<double, n_in, n_in>{
        {1,  ts,         0},
        {0,  1 - ts/tau, 0},
        {0, -1/tau,      0}
    };

    B = Eigen::Vector<double, n_in>{0, ts/tau, 1/tau};

    C = Eigen::Matrix<double, n_out, n_in>::Identity();
    H = Eigen::Matrix<double, n_in, n_in>::Identity();

    F = C * H;

    F_hat = Eigen::MatrixXd::Zero(F.rows() * p, F.cols());
    for (auto i = 0; i < p; i++) {
        F_hat.block(i * F.rows(), 0, F.rows(), H.cols()) = F;
    }

    Eigen::Vector<double, n_out> phi = Eigen::VectorXd::Map(phi_vals.data(), n_out);
    Eigen::Matrix<double, n_out, n_out> fa = phi.asDiagonal();

    FA = Eigen::MatrixXd::Zero(n_out * p, n_out);

    for (auto i = 0; i < p; i++) {
        FA.block(i * fa.rows(), 0, fa.rows(), fa.cols()) = fa.pow(i + 1);
    }

    const Eigen::Vector<double, n_out> qw = Eigen::VectorXd::Map(q_vals.data(), n_out);
    const Eigen::Matrix<double, n_out, n_out> q = qw.asDiagonal();

    Q = Eigen::MatrixXd::Zero(n_out * p, n_out * p);
    for (auto i = 0; i < p; i++) {
        Q.block(i * q.rows(), i * q.cols(),  q.rows(), q.cols()) = q;
    }

    H1 =  Eigen::MatrixXd::Zero(c, c);
    for (auto i = 0; i < c; ++i) {
        for (auto j = 0; j < c; ++j) {

            if (i == j)
                H1(i, j) = (i == (c - 1)) ? s : 2 * s;

            if (i == (j + 1) || j == (i + 1)) {
                H1(i, j) = -s;
            }
        }
    }

    A_hat = Eigen::MatrixXd::Zero(A.rows() * p, A.cols());
    B_hat = Eigen::MatrixXd::Zero(n_in * p, c);
    C_hat = Eigen::MatrixXd::Zero(C.rows() * p, C.cols());
    D_hat = Eigen::MatrixXd::Zero(n_out * p, c);
    H_hat = Eigen::MatrixXd::Zero(H.rows() * p, H.cols());

    for (auto i = 0; i < p; i++) {
        A_hat.block(i * A.rows(), 0, A.rows(), A.cols()) = A.pow(i + 1);

        for (int m = 0; m < c; m++) {
            if (m <= i) {
                B_hat.block(i * n_in, m, n_in, 1)
                    = A.pow(i - m) * B;
            }
        }

        C_hat.block(i * C.rows(), 0, C.rows(), A.cols()) = C * A.pow(i + 1);

        for (auto k = i, m = 0; k >= 0 && m < c; k--, m++) {
            D_hat.block(i * C.rows(), m, C.rows(), B.cols()) = C * A.pow(k) * B;
        }

        H_hat.block(i * H.rows(), 0, H.rows(), H.cols()) = H;
    }

    M_hat = Eigen::VectorXd::Zero(n_in * p);
    N_hat = Eigen::VectorXd::Zero(n_in * p);

    for (auto i = 0; i < p; i++) {
        M_hat.segment(i * n_in + 1, n_in - 1) <<
            a_limits.min,
            j_limits.min;

        N_hat.segment(i * n_in + 1, n_in - 1) <<
            a_limits.max,
            j_limits.max;
    }

    U_min = Eigen::VectorXd::Constant(c, u_limits.min);
    U_max = Eigen::VectorXd::Constant(c, u_limits.max);

    solver.settings()->setTimeLimit(0.04);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(c);
    solver.data()->setNumberOfConstraints(n_in * p + c);
}


std::pair<double, const Eigen::Vector<double, CruiseController::n_out>> CruiseController::calculate_control(double v_ref, double v, double a) {
    double j = (a - a_prev) / ts;
    j = 0;

    Eigen::Vector<double, n_in> x{v, a, j};

    Eigen::Vector<double, n_in> ex;
    if (!x_predicted) {
        ex = decltype(ex)::Zero();
    }
    else {
        ex = x - *x_predicted;
    }

    Z = Eigen::Vector<double, n_out>{v_ref, 0, 0};

    Z_hat = Eigen::VectorXd::Zero(Z.rows() * p);

    for (auto i = 0; i < p; i++) {
        Z_hat.block(i * Z.rows(), 0, Z.rows(), Z.cols()) = Z;
    }

    Eigen::Vector<double, n_out> y = C * x - Z;
    Eigen::VectorXd Yr = FA * y;

    Eigen::VectorXd F2 = C_hat * x + F_hat * ex - Z_hat;
    Eigen::VectorXd b1 = Yr - F2;

    M2 = Eigen::VectorXd::Zero(c);
    M2(0) = s;

    Eigen::VectorXd g1 = -M2 * u_prev;

    Eigen::VectorXd g2 = -D_hat.transpose() * Q * b1;

    Eigen::MatrixXd H2 = D_hat.transpose() * Q * D_hat;

    Eigen::MatrixXd Hqp = H1 + H2;
    Eigen::VectorXd g = g1 + g2;

    for (auto i = 0; i < p; i++) {
        M_hat.segment(i * n_in, 1) << 0.;
        N_hat.segment(i * n_in, 1) << v_ref + 20.;
    }

    Eigen::VectorXd Mu = M_hat - A_hat * x;
    Eigen::VectorXd Nu = N_hat - A_hat * x;

    int constr_num = n_in * p + c;

    Eigen::VectorXd u_min(constr_num);
    u_min.head(n_in * p) = Mu;
    u_min.tail(c) = U_min;

    Eigen::VectorXd u_max(constr_num);
    u_max.head(n_in * p) = Nu;
    u_max.tail(c) = U_max;

    Eigen::MatrixXd constr_matrix = Eigen::MatrixXd::Zero(constr_num, c);
    constr_matrix.block(0, 0, n_in*p, c) = B_hat;
    constr_matrix.block(n_in*p, 0, c, c) = Eigen::MatrixXd::Identity(c,c);

    Eigen::SparseMatrix<double> constr_matrix_sparse = constr_matrix.sparseView();
    Eigen::SparseMatrix<double> Hqp_sparse = Hqp.sparseView();

    if (!x_predicted) {
        solver.data()->setHessianMatrix(Hqp_sparse);
        solver.data()->setGradient(g);
        solver.data()->setLinearConstraintsMatrix(constr_matrix_sparse);
        solver.data()->setLowerBound(u_min);
        solver.data()->setUpperBound(u_max);

        solver.settings()->setWarmStart(false);

        solver.initSolver();
    }
    else {
        solver.updateHessianMatrix(Hqp_sparse);
        solver.updateGradient(g);
        solver.updateLinearConstraintsMatrix(constr_matrix_sparse);
        solver.updateBounds(u_min, u_max);
    }

    solver.solveProblem();
    double res = solver.getSolution()[0];

    if (solver.getStatus() != OsqpEigen::Status::Solved) {
        res = u_prev;
        solver.settings()->setWarmStart(false);
    }
    else {
        solver.settings()->setWarmStart(true);
    }

    res = std::clamp(res, U_min(0), U_max(0));

    x_predicted = B * res + A * x + H * ex;
    u_prev = res;

    a_prev = a;

    return std::make_pair(res, y);
}

}
