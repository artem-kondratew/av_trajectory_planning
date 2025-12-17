#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "adas/cruise_controller.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cc = cruise_control;


class ADASNode : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr linear_velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;

    std::unique_ptr<cc::CruiseController> cruise_controller_;

    double v_ref_;
    
    sensor_msgs::msg::Imu last_imu_msg_;
    double t_last_;

    static constexpr double ms2kmh = 3.6;
    static constexpr double kmh2ms = 1 / ms2kmh;

public:
    ADASNode();

private:
    void linearVelocityCallback(const geometry_msgs::msg::Twist& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);

    cc::Limit vectorToLimit(const std::vector<double>& vec) {return cc::Limit{vec[0], vec[1]};}
    std::string vectorToString(const std::vector<double>& vec);
};


ADASNode::ADASNode() : Node("adas_node") {
    this->declare_parameter("local_velocity_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("imu_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("tau", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("c", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("s", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("v_ref", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("v_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("a_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("j_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("u_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("phi_vals", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("q_vals", rclcpp::PARAMETER_DOUBLE_ARRAY);

    std::string local_velocity_topic = this->get_parameter("local_velocity_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string control_topic = this->get_parameter("control_topic").as_string();
    double tau = this->get_parameter("tau").as_double();
    int p = this->get_parameter("p").as_int();
    int c = this->get_parameter("c").as_int();
    double s = this->get_parameter("s").as_double();
    v_ref_ = this->get_parameter("v_ref").as_double();
    cc::Limit v_limits = vectorToLimit(this->get_parameter("v_limits").as_double_array());
    cc::Limit a_limits = vectorToLimit(this->get_parameter("a_limits").as_double_array());
    cc::Limit j_limits = vectorToLimit(this->get_parameter("j_limits").as_double_array());
    cc::Limit u_limits = vectorToLimit(this->get_parameter("u_limits").as_double_array());
    std::vector<double> phi_vals = this->get_parameter("phi_vals").as_double_array();
    std::vector<double> q_vals = this->get_parameter("q_vals").as_double_array();

    RCLCPP_INFO(this->get_logger(), "local_velocity_topic: '%s'", local_velocity_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: '%s'", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic: '%s'", control_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "tau: %lf", tau);
    RCLCPP_INFO(this->get_logger(), "p: %d", p);
    RCLCPP_INFO(this->get_logger(), "c: %d", c);
    RCLCPP_INFO(this->get_logger(), "s: %lf", s);
    RCLCPP_INFO(this->get_logger(), "v_ref: %lf km/h", v_ref_);
    RCLCPP_INFO(this->get_logger(), "v_limits: [%lf, %lf] km/h", v_limits.min, v_limits.max);
    RCLCPP_INFO(this->get_logger(), "a_limits: [%lf, %lf] m/s^2", a_limits.min, a_limits.max);
    RCLCPP_INFO(this->get_logger(), "j_limits: [%lf, %lf] m/s^3", j_limits.min, j_limits.max);
    RCLCPP_INFO(this->get_logger(), "u_limits: [%lf, %lf] m/s^2", u_limits.min, u_limits.max);
    RCLCPP_INFO(this->get_logger(), "phi_vals: %s", vectorToString(phi_vals).c_str());
    RCLCPP_INFO(this->get_logger(), "q_vals: %s", vectorToString(q_vals).c_str());

    v_ref_ *= kmh2ms;
    v_limits.min *= kmh2ms;
    v_limits.max *= kmh2ms;

    linear_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(local_velocity_topic, 10, std::bind(&ADASNode::linearVelocityCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&ADASNode::imuCallback, this, _1));
    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(control_topic, 10);

    cruise_controller_ = std::make_unique<cc::CruiseController>(tau, p, c, s, phi_vals, q_vals, v_limits, a_limits, j_limits, u_limits);

    t_last_ = this->get_clock()->now().seconds();
}


void ADASNode::linearVelocityCallback(const geometry_msgs::msg::Twist& msg) {
    double t = this->get_clock()->now().seconds();
    double dt = t - t_last_;

    double v = msg.linear.x;

    double a = last_imu_msg_.linear_acceleration.x;

    v_ref_ = this->get_parameter("v_ref").as_double() * kmh2ms;

    double u = cruise_controller_->calculate_control(dt, v_ref_, v, a);

    auto control_msg = carla_msgs::msg::CarlaEgoVehicleControl();

    double throttle_hold = 0.1;
    double u_brake_deadzone = 0.2;

    if (u >= 0) {
        control_msg.throttle = std::clamp(u, 0., 1.);
    }
    else {
        control_msg.brake = std::clamp(-u, 0., 1.);
    }

    control_pub_->publish(control_msg);

    t_last_ = t;
}

void ADASNode::imuCallback(const sensor_msgs::msg::Imu& msg) {
    last_imu_msg_ = msg;
}


std::string ADASNode::vectorToString(const std::vector<double>& vec) {
    std::string s{"["};
    for (size_t i = 0; i < vec.size(); i++) {
        s += std::string{std::to_string(vec[i])} + std::string{" "};
    }
    s += "]";
    return s;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ADASNode>()); 
    rclcpp::shutdown();

    return 0;
}
