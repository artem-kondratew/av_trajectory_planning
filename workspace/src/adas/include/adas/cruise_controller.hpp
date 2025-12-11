#ifndef ADAS_CRUISE_CONTROLLER_HPP
#define ADAS_CRUISE_CONTROLLER_HPP


#include <tuple>

#include <Eigen/Dense>


class CruiseController {
private:
    double tau_;
    std::tuple<double, double> v_constr_;
    std::tuple<double, double> a_constr_;
    std::tuple<double, double> j_constr_;
    std::tuple<double, double> u_constr_;

    double a_prev_ = 0;
     
public:
    CruiseController() = default;

    CruiseController(
        double tau,
        const std::tuple<double, double>& v_constr,
        const std::tuple<double, double>& a_constr,
        const std::tuple<double, double>& j_constr,
        const std::tuple<double, double>& u_constr
    );

    double calculate_control(double dt, double v_ref, double v, double a);
};


#endif  // ADAS_CRUISE_CONTROLLER_HPP
