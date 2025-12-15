#include "cc_mpc.hpp"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace atom { namespace control { namespace cc {

mpc_controller::mpc_controller(
    const double tau,
    const double ts,
    const double dc,
    const double default_speed,
    const double time_gap)
    : tau{tau}
    , ts{ts}
    , dc{dc}
    , time_gap{time_gap} {


    A = Eigen::Matrix<double, n_in, n_in>{
        {1, ts,         0},
        {0, 1-ts/tau,   0},
        {0, -1/tau,     0}
    };

    B = {0, ts/tau, 1/tau};

    C = Eigen::Matrix<double, n_out, n_in>{
        {1, 0,  0},
        {0, 1,  0},
        {0, 0,  1}
    };

    H = Eigen::Matrix<double, n_in, n_in>::Identity();

    F = C * H;

    for (unsigned i = 0; i < p; ++i) {
        const auto Apow = A.pow(i + 1);
        AA.block<n_in, n_in>(i * A.rows(), 0) = Apow;
        HH.block<n_in, n_in>(i * H.rows(), 0) = H;
    }

    for (unsigned i = 0; i < p; ++i) {
        const auto Apow = A.pow(i + 1);
        CC.block<n_out, n_in>(i * C.rows(), 0) = C * Apow;
        FF.block<n_out, n_in>(i * F.rows(), 0) = F;
    }

    DD = decltype(DD)::Zero(C.rows() * p, c);
    for (unsigned i = 0; i < p; ++i) {
        unsigned m = 0;
        for(int j = i; j >= 0 && m < c; --j, ++m) {
            DD.block<n_out, 1>(i * C.rows(), m) = C * A.pow(j) * B;
        }
    }

    const Eigen::Vector<double, n_out> phi{0.95, 0.95, 0.95};
    const Eigen::Matrix<double, phi.rows(), phi.rows()> fa = phi.asDiagonal();
    for (unsigned i = 0; i < p; ++i) {
        FA.block<fa.rows(), fa.cols()>(i * fa.rows(), 0) = fa.pow(i + 1);
    }

    constexpr double s = 18;
    H1 =  Eigen::Matrix<double, c, c>::Zero();
    for (unsigned i = 0; i < c; ++i) {
        for (unsigned j = 0; j < c; ++j) {

            if (i == j)
                H1(i, j) = (i == (c - 1)) ? s : 2 * s;

            if (i == (j + 1) || j == (i + 1)) {
                H1(i, j) = -s;
            }
        }
    }

    M2 = Eigen::Vector<double, c>::Zero();
    M2[0] = s;

    const Eigen::Vector<double, n_out> qw{10, 1, 1};
    const Eigen::Matrix<double, n_out, n_out> q = qw.asDiagonal();
    Q = decltype (Q)::Zero();
    for (unsigned i = 0; i < p; ++i) {
        Q.block<q.rows(), q.cols()>(i * q.rows(), i * q.cols()) = q;
    }

    update_desired_speed(default_speed);

    solver.settings()->setTimeLimit(ts);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(c);
    solver.data()->setNumberOfConstraints(c + p * n_phys);
}

std::pair<double, OsqpEigen::Status>
mpc_controller::operator()(const Eigen::Vector<double, n_in> &x, const Eigen::Vector<double, n_in> &prev_x) {

    const Eigen::Vector<double, n_out> y = C * x - Z;

    const Eigen::Vector<double, n_in> ex = (!predict_x)
        ?  Eigen::Vector<double, n_in>{0, 0, 0}
        : (x - (*predict_x));

    const Eigen::Vector<double, c> g1 = -M2 * (prev_u ? *prev_u : 0.);

    const Eigen::Vector<double, p * n_out> Yr = FA * y;
    const Eigen::Vector<double, p * n_out> F2 = CC * x + FF * ex - ZZ;
    const auto b1 = Yr - F2;

    const Eigen::Matrix<double, c, c> H2 = DD.transpose() * Q * DD;
    const Eigen::Vector<double, c> g2 = -DD.transpose() * Q * b1;

    const Eigen::Vector<double, c> u_min = decltype (u_min)::Constant(-4);
    const Eigen::Vector<double, c> u_max = decltype (u_max)::Constant(4);

    Eigen::Vector<double, c + p * n_phys> low;
    low.segment<c>(p * n_phys) = u_min;

    Eigen::Vector<double, c + p * n_phys> up;
    up.segment<c>(p * n_phys) = u_max;

    Eigen::Matrix<double, c + p * n_phys, c> aux_constraining_matrix;
    aux_constraining_matrix.block<c, c>(p * n_phys, 0) = Eigen::Matrix<double, c, c>::Identity();

    Eigen::SparseMatrix<double> constraining_matrix;
    constraining_matrix.resize(c + p * n_phys, c);
    constraining_matrix = aux_constraining_matrix.sparseView();

    Eigen::SparseMatrix<double> Hqp;
    Hqp.resize(c, c);
    Hqp = (H1 + H2).sparseView();

    Eigen::VectorXd gqp;
    gqp = Eigen::VectorXd::Zero(c);
    gqp = g1 + g2;

    if (!predict_x) {
        solver.data()->setHessianMatrix(Hqp);
        solver.data()->setGradient(gqp);
        solver.data()->setLinearConstraintsMatrix(constraining_matrix);
        solver.data()->setLowerBound(low);
        solver.data()->setUpperBound(up);

        solver.initSolver();
    } else {
        solver.updateHessianMatrix(Hqp);
        solver.updateGradient(gqp);
        solver.updateLinearConstraintsMatrix(constraining_matrix);
        solver.updateBounds(low, up);
    }

    solver.solveProblem();
    auto res = solver.getSolution()[0];

    if (solver.getStatus() != OsqpEigen::Status::Solved) {
        res = prev_u ? *prev_u : 0.;
        solver.settings()->setWarmStart(false);
    } else
        solver.settings()->setWarmStart(true);

    predict_x = B * res + A * x + H * ex;
    prev_u = res;

    return std::make_pair(res, solver.getStatus());
}

double mpc_controller::get_desired_speed() const noexcept {
    return Z[0];
}

void mpc_controller::update_desired_speed(const double desired_speed) {
    Z = Eigen::Vector<double, n_out>{desired_speed, 0, 0};

    for (unsigned i = 0; i < p; ++i)
        ZZ.segment<n_out>(i * Z.rows()) = Z;
}

void mpc_controller::reset(const std::optional<double> u) {
    if (u)
        prev_u = u;

    if (predict_x) {
        predict_x.reset();

        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();

        solver.clearSolverVariables();
        solver.clearSolver();

        solver.settings()->setWarmStart(false);
    }
}

} } }
