#ifndef ADAS_CRUISE_CONTROLLER_HPP
#define ADAS_CRUISE_CONTROLLER_HPP


#include <tuple>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>


namespace cruise_control {

struct Limit {
    double min;
    double max;

    Limit(double min, double max) : min{min}, max{max} {}
};

class CruiseController {
private:
    const double tau;
    const int p;
    const int c;

    static constexpr int n_in = 3;
    static constexpr int n_out = 3;

    Eigen::Matrix<double, n_in, n_in> A;
    Eigen::VectorXd B;
    Eigen::MatrixXd C;
    Eigen::VectorXd Z;
    Eigen::MatrixXd H;
    Eigen::MatrixXd F;

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

    Eigen::MatrixXd M2;

    static constexpr double s = 18;

    Eigen::Vector<double, n_in> x_predicted;
    
    double a_prev = 0;
    double u_prev = 0;

    const Limit v_limits;
    const Limit a_limits;
    const Limit j_limits;
    const Limit u_limits;

    OsqpEigen::Solver solver;

    bool solver_init_flag = false;
     
public:
    CruiseController(
        double tau,
        int p,
        int c,
        const Limit& v_limits,
        const Limit& a_limits,
        const Limit& j_limits,
        const Limit& u_limits
    );

    double calculate_control(double dt, double v_ref, double v, double a);
};

}


#endif  // ADAS_CRUISE_CONTROLLER_HPP
