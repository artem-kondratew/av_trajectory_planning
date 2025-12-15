#include "adas/cruise_controller.hpp"

#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>

namespace cruise_control {

CruiseController::CruiseController(
    double tau,
    int p,
    int c,
    const Limit& v_limits,
    const Limit& a_limits,
    const Limit& j_limits,
    const Limit& u_limits
)
    : tau{tau}
    , p{p}
    , c{c}
    , v_limits{v_limits}
    , a_limits{a_limits}
    , j_limits{j_limits}
    , u_limits{u_limits} {

    C = Eigen::Matrix<double, n_out, n_in>::Identity();
    H = Eigen::Matrix<double, n_in, n_in>::Identity();

    F = C * H;

    F_hat = Eigen::MatrixXd::Zero(F.rows() * p, F.cols());
    for (auto i = 0; i < p; i++) {
        F_hat.block(i * F.rows(), 0, F.rows(), H.cols()) = F;
    }
}


double CruiseController::calculate_control(double ts, double v_ref, double v, double a) {
    // reset(ts);

    double j = (a - a_prev) / ts;

    Eigen::Vector3d x{v, a, j};

    ts = 0.06;

    std::cout << ts << " " << tau << std::endl;

    A = Eigen::Matrix<double, n_in, n_in>{
        {1,  ts,         0},
        {0,  1 - ts/tau, 0},
        {0, -1/tau,      0}
    };

    B = Eigen::Vector<double, n_in>{0, ts/tau, 1/tau};

    Z = Eigen::Vector<double, n_out>{v_ref, 0, 0};

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

    a_prev = a;

    cnt++;

    if (cnt == 1)
        exit(0);

    return 0;
}


void CruiseController::reset(double ts) {
    solver.data()->clearHessianMatrix();
    solver.data()->clearLinearConstraintsMatrix();

    solver.clearSolverVariables();
    solver.clearSolver();

    solver.settings()->setWarmStart(false);

    solver.settings()->setTimeLimit(ts);
    solver.settings()->setVerbosity(true);
    solver.data()->setNumberOfVariables(c);
    // solver.data()->setNumberOfConstraints(c + p * n_phys);
}

}
