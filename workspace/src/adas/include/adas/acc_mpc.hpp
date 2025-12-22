#ifndef ADAS_ACC_MPC_HPP
#define ADAS_ACC_MPC_HPP


#include <optional>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include "adas/limit.hpp"


namespace adaptive_cruise_control {

class AdaptiveCruiseController {
private:
    const double tau;
    const int p;
    const int c;
    const double s;
    const double d0;
    const double th;

    static constexpr int n_in = 5;
    static constexpr int n_out = 4;

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
    
    double a_prev = 0;
    double u_prev = 0;

    const Limit u_limits;

    OsqpEigen::Solver solver;
     
public:
    AdaptiveCruiseController(
        double tau,
        int p,
        int c,
        double s,
        double d0,
        double th,
        const std::vector<double>& phi_vals,
        const std::vector<double>& q_vals,
        const Limit& u_limits
    );

    std::pair<double, const Eigen::Vector<double, n_out>> calculate_control(double ts, double dx, double v, double v_rel, double a);
};

}


#endif  // ADAS_ACC_MPC_HPP
