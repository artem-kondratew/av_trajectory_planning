#include "adas/acc_mpc.hpp"

#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>


namespace adaptive_cruise_control {

using ACC = AdaptiveCruiseController;


ACC::AdaptiveCruiseController(
    double tau,
    int p,
    int c,
    double s,
    double d0,
    double th,
    const std::vector<double>& phi_vals,
    const std::vector<double>& q_vals,
    const Limit& u_limits
)
    : tau{tau}
    , p{p}
    , c{c}
    , s{s}
    , d0{d0}
    , th{th}
    , u_limits{u_limits} {

    C = Eigen::Matrix<double, n_out, n_in>::Zero();
    C(0, 0) = 1.0;
    C(0, 1) = -th;
    C.block<3, 3>(1, 2) = Eigen::Matrix<double, 3, 3>::Identity();

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

    solver.settings()->setTimeLimit(0.04);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(c);
    solver.data()->setNumberOfConstraints(c);
}


std::pair<double, const Eigen::Vector<double, ACC::n_out>> ACC::calculate_control(double ts, double dx, double v, double v_rel, double a) {
    double j = (a - a_prev) / ts;
    j = 0;

    Eigen::Vector<double, n_in> x{dx, v, v_rel, a, j};

    Eigen::Vector<double, n_in> ex;
    if (!x_predicted) {
        ex = decltype(ex)::Zero();
    }
    else {
        ex = x - *x_predicted;
    }

    A = Eigen::Matrix<double, n_in, n_in>{
        {1, 0, ts, -0.5*ts*ts, 0},
        {0, 1,  0,         ts, 0},
        {0, 0,  1,        -ts, 0},
        {0, 0,  0,   1-ts/tau, 0},
        {0, 0,  0,     -1./tau, 0}
    };

    B = Eigen::Vector<double, n_in>{0, 0, 0, ts/tau, 1/tau};

    Z = Eigen::Vector<double, n_out>{d0, 0, 0, 0};

    A_hat = Eigen::MatrixXd::Zero(A.rows() * p, A.cols());
    C_hat = Eigen::MatrixXd::Zero(C.rows() * p, C.cols());
    H_hat = Eigen::MatrixXd::Zero(H.rows() * p, H.cols());
    Z_hat = Eigen::VectorXd::Zero(Z.rows() * p);

    for (auto i = 0; i < p; i++) {
        A_hat.block(i * A.rows(), 0, A.rows(), A.cols()) = A.pow(i + 1);
        C_hat.block(i * C.rows(), 0, C.rows(), A.cols()) = C * A.pow(i + 1);
        H_hat.block(i * H.rows(), 0, H.rows(), H.cols()) = H;
        Z_hat.block(i * Z.rows(), 0, Z.rows(), Z.cols()) = Z;
    }

    D_hat = Eigen::MatrixXd::Zero(n_out * p, c);

    for (auto i = 0; i < p; i++) {
        for (auto k = i, m = 0; k >= 0 && m < c; k--, m++) {
            D_hat.block(i * C.rows(), m, C.rows(), B.cols()) = C * A.pow(k) * B;
        }
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

    Eigen::VectorXd u_min = Eigen::VectorXd::Constant(c, u_limits.min);
    Eigen::VectorXd u_max = Eigen::VectorXd::Constant(c,  u_limits.max);

    Eigen::SparseMatrix<double> constr_matrix = Eigen::MatrixXd::Identity(c, c).sparseView();

    Eigen::SparseMatrix<double> Hqp_sparse = Hqp.sparseView();

    if (!x_predicted) {
        solver.data()->setHessianMatrix(Hqp_sparse);
        solver.data()->setGradient(g);
        solver.data()->setLinearConstraintsMatrix(constr_matrix);
        solver.data()->setLowerBound(u_min);
        solver.data()->setUpperBound(u_max);

        solver.settings()->setWarmStart(false);

        solver.initSolver();
    }
    else {
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
    }
    else {
        solver.settings()->setWarmStart(true);
    }

    res = std::clamp(res, u_limits.min, u_limits.max);

    x_predicted = B * res + A * x + H * ex;
    u_prev = res;

    a_prev = a;

    return std::make_pair(res, y);
}

}
