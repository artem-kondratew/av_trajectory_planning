#include <chrono>
#include <iostream>
#include <limits>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "adas/cruise_controller.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class ADASNode : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr linear_velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    CruiseController cruise_controller_;

public:
    ADASNode();

private:
    void linearVelocityCallback(const geometry_msgs::msg::Twist& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);
};


ADASNode::ADASNode() : Node("cruise_control_node") {
    linear_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/carla/hero/local_velocity", 10, std::bind(&ADASNode::linearVelocityCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/carla/hero/imu", 10, std::bind(&ADASNode::imuCallback, this, _1));

    const double tau = 0.1;

    const std::tuple<double, double> v_constr(0., 15.);
    const std::tuple<double, double> a_constr{-15, 15};
    const std::tuple<double, double> j_constr{-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    const std::tuple<double, double> u_constr = a_constr;

    CruiseController cruise_controller_(tau, v_constr, a_constr, j_constr, u_constr);
}


void ADASNode::linearVelocityCallback(const geometry_msgs::msg::Twist& msg) {
    cruise_controller_.calculate_control(1, 1, 2, 3);
}

void ADASNode::imuCallback(const sensor_msgs::msg::Imu& msg) {
    
}


// int main(int argc, char * argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ADASNode>()); 
//     rclcpp::shutdown();

//     return 0;
// }

#include <iostream>
#include <vector>
#include "osqp/osqp.h"

int main() {
    // Problem size
    OSQPInt n = 2;  // variables
    OSQPInt m = 3;  // constraints

    // P = [4 1; 1 2] (upper triangular in CSC)
    std::vector<OSQPFloat> P_val = {4.0, 1.0, 2.0};
    std::vector<OSQPInt> P_row = {0, 1, 1};
    std::vector<OSQPInt> P_col = {0, 2, 3}; // CSC: col pointers (length n+1)

    // q = [1, 1]
    std::vector<OSQPFloat> q = {1.0, 1.0};

    // A = [1 1; 1 0; 0 1]
    std::vector<OSQPFloat> A_val = {1.0, 1.0, 1.0, 1.0};
    std::vector<OSQPInt> A_row = {0, 1, 0, 2};
    std::vector<OSQPInt> A_col = {0, 2, 4}; // length n+1

    // Bounds
    std::vector<OSQPFloat> l = {1.0, 0.0, 0.0};
    std::vector<OSQPFloat> u = {1.0, 0.7, 0.7};

    // Create solver
    OSQPSolver* solver = osqp_new();

    // Set problem data
    osqp_set_data(solver,
                  n, m,
                  P_val.data(), P_row.data(), P_col.data(),
                  q.data(),
                  A_val.data(), A_row.data(), A_col.data(),
                  l.data(), u.data());

    // (Optional) Set settings
    osqp_set_int_param(solver, "max_iter", 2000);
    osqp_set_float_param(solver, "eps_abs", 1e-5);
    osqp_set_float_param(solver, "eps_rel", 1e-5);

    // Solve
    osqp_solve(solver);

    // Get solution status and result
    OSQPInt status = osqp_get_info(solver)->status_val;
    if (status == OSQP_SOLVED) {
        const OSQPFloat* x = osqp_get_solution(solver);
        std::cout << "Solution: " << x[0] << " " << x[1] << std::endl;
    } else {
        std::cout << "OSQP failed to solve. Status: " << osqp_get_info(solver)->status << std::endl;
    }

    // Cleanup
    osqp_free(solver);
    return 0;
}