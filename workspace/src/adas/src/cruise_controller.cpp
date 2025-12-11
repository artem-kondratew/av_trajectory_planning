#include "adas/cruise_controller.hpp"


CruiseController::CruiseController(
    double tau,
    const std::tuple<double, double>& v_constr,
    const std::tuple<double, double>& a_constr,
    const std::tuple<double, double>& j_constr,
    const std::tuple<double, double>& u_constr
)
                : tau_{tau}
                , v_constr_{v_constr}
                , a_constr_{a_constr}
                , j_constr_{j_constr}
                , u_constr_{u_constr}
{}


std::tuple<double, double> CruiseController::calculate_control(double dt, double v_ref, double v, double a) {
    double j = (a - a_prev_) / dt;

    double a01 = dt;
    double a11 = 1 - dt / tau_;
    double a21 = -1 / tau_;

    double A_data[] = {
        1, a01, 0,
        0, a11, 0,
        0, a21, 0,
    };
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> A(A_data);

    Eigen::Matrix3d C = Eigen::Matrix3d::Identity();

    Eigen::Vector3d B(
        0,
        dt / tau_,
        1 / tau_
    );

    Eigen::Vector3d Z(
        v_ref,
        0,
        0
    );

    // auto Y = C * x - Z;

    a_prev_ = a;

    return std::tuple<double, double>();
}
