#ifndef ATOM_CONTROL_CC_MPC_HPP
#define ATOM_CONTROL_CC_MPC_HPP

#include <optional>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

namespace atom { namespace control { namespace cc {

class mpc_controller {
public:
    static constexpr unsigned p = 10;
    static constexpr unsigned c = 5;
    static constexpr unsigned n_in = 3;
    static constexpr unsigned n_out = 3;
    static constexpr unsigned n_phys = 0;

    mpc_controller(
        const double tau,
        const double ts,
        const double dc,
        const double default_speed, // NB! meters per second
        const double time_gap);

    std::pair<double, OsqpEigen::Status>
    operator()(const Eigen::Vector<double, n_in> &x, const Eigen::Vector<double, n_in> &prev_x);

    double get_desired_speed() const noexcept;
    void update_desired_speed(const double desired_speed);  // NB! meters per second
    void reset(const std::optional<double> u = std::nullopt);

private:
    const double tau;
    const double ts;
    const double dc;
    const double time_gap;

    Eigen::Matrix<double, n_in, n_in> A;
    Eigen::Vector<double, n_in> B;
    Eigen::Matrix<double, n_out, n_in> C;
    Eigen::Matrix<double, n_out, n_in> F;
    Eigen::Vector<double, n_in> G;
    Eigen::Matrix<double, n_in, n_in> H;
    Eigen::Matrix<double, n_out * p, n_out * p> Q;
    Eigen::Vector<double, n_out> Z;

    Eigen::Matrix<double, n_in * p, n_in> AA;
    Eigen::Matrix<double, n_in * p, c> BB;
    Eigen::Matrix<double, n_out * p, n_in> CC;
    Eigen::Matrix<double, n_out * p, c> DD;
    Eigen::Matrix<double, n_out * p, n_in> FF;
    Eigen::Matrix<double, n_in * p, n_in> HH;
    Eigen::Vector<double, n_out * p> ZZ;

    Eigen::Vector<double, n_phys * p> MM;
    Eigen::Vector<double, n_phys * p> NN;
    Eigen::Matrix<double, n_phys * p, n_in * p> LL;
    Eigen::Matrix<double, n_phys * p, c> LL_BB;

    Eigen::Matrix<double, c, c> H1;
    Eigen::Vector<double, c> M2;
    Eigen::Matrix<double, n_out * p, n_out> FA;

    std::optional<Eigen::Vector<double, n_in>> predict_x;
    std::optional<double> prev_u;

    OsqpEigen::Solver solver;
};

} } }

#endif // ATOM_CONTROL_ACC_MPC_HPP
