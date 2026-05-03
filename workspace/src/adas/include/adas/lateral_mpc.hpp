#ifndef ADAS_LATERAL_MPC_HPP
#define ADAS_LATERAL_MPC_HPP


#include <optional>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include "adas/limit.hpp"


namespace lateral_control {

class LateralMPC {
private:
    const double ts;
    const double tau;
    const int p;
    const int c;
    const double L;
    const double s;

    static constexpr int n = 3;

    // Constant matrices (precomputed in constructor)
    Eigen::MatrixXd F_hat;  // (n*p × n)  error-correction propagation (C*H tiled)
    Eigen::MatrixXd Q;      // (n*p × n*p) output weight matrix
    Eigen::MatrixXd H1;     // (c × c)    control-smoothness matrix

    // Per-call matrices (rebuilt in calculate_control for LTV model)
    Eigen::MatrixXd A_hat;  // (n*p × n)  compound state-transition
    Eigen::MatrixXd D_hat;  // (n*p × c)  control influence
    Eigen::VectorXd W_hat;  // (n*p)      accumulated affine linearisation offsets

    Limit u_limits;
    double u_prev = 0.0;

    std::optional<Eigen::Vector<double, n>> x_predicted;

    OsqpEigen::Solver solver;

public:
    LateralMPC(
        double ts,
        double tau,
        int p,
        int c,
        double L,
        double s,
        const std::vector<double>& q_vals,
        const Limit& u_limits
    );

    std::pair<double, const Eigen::Vector<double, n>> calculate_control(
        double e, double theta, double delta,
        const std::vector<double>& kappa_ref, double vx);
};

}


#endif  // ADAS_LATERAL_MPC_HPP
