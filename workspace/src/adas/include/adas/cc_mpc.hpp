#ifndef ADAS_CC_MPC_HPP
#define ADAS_CC_MPC_HPP


#include <optional>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include "adas/limit.hpp"


namespace cruise_control {

class CruiseController {
private:
    const double ts;
    const double tau;
    const int p;
    const int c;
    const double s;

    static constexpr int n_in = 3;
    static constexpr int n_out = 3;

    Eigen::Matrix<double, n_in, n_in> A;
    Eigen::Vector<double, n_in> B;
    Eigen::Matrix<double, n_out, n_in> C;
    Eigen::Vector<double, n_out> Z;
    Eigen::Matrix<double, n_in, n_in> H;
    Eigen::Matrix<double, n_out, n_in> F;

    Eigen::MatrixXd A_hat;
    Eigen::MatrixXd B_hat;
    Eigen::MatrixXd C_hat;
    Eigen::MatrixXd H_hat;
    Eigen::VectorXd Z_hat;
    Eigen::MatrixXd D_hat;
    Eigen::MatrixXd F_hat;

    Eigen::MatrixXd FA;
    Eigen::MatrixXd Q;

    Eigen::MatrixXd H1;

    Eigen::VectorXd M2;

    std::optional<Eigen::Vector<double, n_in>> x_predicted;

    Eigen::VectorXd M_hat;
    Eigen::VectorXd N_hat;

    Eigen::VectorXd U_min;
    Eigen::VectorXd U_max;
    
    double a_prev = 0;
    double u_prev = 0;

    OsqpEigen::Solver solver;
     
public:
    CruiseController(
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
    );

    std::pair<double, const Eigen::Vector<double, n_out>> calculate_control(double v_ref, double v, double a);
};

}


#endif  // ADAS_CC_MPC_HPP
