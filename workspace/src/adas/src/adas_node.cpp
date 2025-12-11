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

    double v_ref_;
    
    sensor_msgs::msg::Imu last_imu_msg_;
    double t_last_;

public:
    ADASNode();

private:
    void linearVelocityCallback(const geometry_msgs::msg::Twist& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);

    std::tuple<double, double> vectorToTuple(const std::vector<double>& vec) {return std::tuple<double, double>{vec[0], vec[1]};}
};


ADASNode::ADASNode() : Node("adas_node") {
    this->declare_parameter("local_velocity_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("imu_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("tau", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("v_ref", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("v_constr", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("a_constr", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("j_constr", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("u_constr", rclcpp::PARAMETER_DOUBLE_ARRAY);

    std::string local_velocity_topic = this->get_parameter("local_velocity_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    double tau = this->get_parameter("tau").as_double();
    v_ref_ = this->get_parameter("v_ref").as_double();
    std::tuple<double, double> v_constr = vectorToTuple(this->get_parameter("v_constr").as_double_array());
    std::tuple<double, double> a_constr = vectorToTuple(this->get_parameter("a_constr").as_double_array());
    std::tuple<double, double> j_constr = vectorToTuple(this->get_parameter("j_constr").as_double_array());
    std::tuple<double, double> u_constr = vectorToTuple(this->get_parameter("u_constr").as_double_array());

    RCLCPP_INFO(this->get_logger(), "local_velocity_topic: '%s'", local_velocity_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: '%s'", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "tau: %lf", tau);
    RCLCPP_INFO(this->get_logger(), "v_ref: %lf", v_ref_);
    RCLCPP_INFO(this->get_logger(), "v_constr: [%lf, %lf]", std::get<0>(v_constr), std::get<1>(v_constr));
    RCLCPP_INFO(this->get_logger(), "a_constr: [%lf, %lf]", std::get<0>(a_constr), std::get<1>(a_constr));
    RCLCPP_INFO(this->get_logger(), "j_constr: [%lf, %lf]", std::get<0>(j_constr), std::get<1>(j_constr));
    RCLCPP_INFO(this->get_logger(), "u_constr: [%lf, %lf]", std::get<0>(u_constr), std::get<1>(u_constr));

    linear_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(local_velocity_topic, 10, std::bind(&ADASNode::linearVelocityCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&ADASNode::imuCallback, this, _1));

    cruise_controller_ = CruiseController(tau, v_constr, a_constr, j_constr, u_constr);

    t_last_ = this->get_clock()->now().seconds();
}


void ADASNode::linearVelocityCallback(const geometry_msgs::msg::Twist& msg) {
    double t = this->get_clock()->now().seconds();
    double dt = t - t_last_;

    double v = msg.linear.x;

    double a = last_imu_msg_.linear_acceleration.x;

    cruise_controller_.calculate_control(dt, v_ref_, v, a);
}

void ADASNode::imuCallback(const sensor_msgs::msg::Imu& msg) {
    last_imu_msg_ = msg;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ADASNode>()); 
    rclcpp::shutdown();

    return 0;
}

// #include <stdlib.h>
// #include "osqp.h"

// int main(int argc, char **argv) {
//     /* Load problem data */
//     OSQPFloat P_x[3] = {4.0, 1.0, 2.0, };
//     OSQPInt P_nnz = 3;
//     OSQPInt P_i[3] = {0, 0, 1, };
//     OSQPInt P_p[3] = {0, 1, 3, };
//     OSQPFloat q[2] = {1.0, 1.0, };
//     OSQPFloat A_x[4] = {1.0, 1.0, 1.0, 1.0, };
//     OSQPInt A_nnz = 4;
//     OSQPInt A_i[4] = {0, 1, 0, 2, };
//     OSQPInt A_p[3] = {0, 2, 4, };
//     OSQPFloat l[3] = {1.0, 0.0, 0.0, };
//     OSQPFloat u[3] = {1.0, 0.7, 0.7, };
//     OSQPInt n = 2;
//     OSQPInt m = 3;

//     /* Exitflag */
//     OSQPInt exitflag = 0;

//     /* Solver */
//     OSQPSolver *solver;

//     /* Create CSC matrices that are backed by the above data arrays. */
//     OSQPCscMatrix* P = OSQPCscMatrix_new(n, n, P_nnz, P_x, P_i, P_p);
//     OSQPCscMatrix* A = OSQPCscMatrix_new(m, n, A_nnz, A_x, A_i, A_p);

//     /* Setup settings */
//     OSQPSettings *settings = OSQPSettings_new();
//     settings->alpha = 1.0; /* Change alpha parameter */

//     /* Setup solver */
//     exitflag = osqp_setup(&solver, P, q, A, l, u, m, n, settings);

//     /* Solve problem */
//     if (!exitflag) exitflag = osqp_solve(solver);

//     /* Cleanup */
//     osqp_cleanup(solver);
//     OSQPCscMatrix_free(A);
//     OSQPCscMatrix_free(P);
//     OSQPSettings_free(settings);

//     return (int)exitflag;
// }