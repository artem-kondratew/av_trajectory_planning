#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <control_toolbox/pid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include "adas/cc_mpc.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cc = cruise_control;


class ADASNode : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr linear_velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr v_ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr y_pub_;

    std::unique_ptr<cc::CruiseController> cruise_controller_;

    control_toolbox::Pid pid_;

    double ts_;
    double v_ref_;
    
    sensor_msgs::msg::Imu last_imu_msg_;
    double t_last_;
    double e_prev_ = 0.0;

    static constexpr double ms2kmh = 3.6;
    static constexpr double kmh2ms = 1 / ms2kmh;

public:
    ADASNode();

private:
    void linearVelocityCallback(const geometry_msgs::msg::Twist& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);

    cc::Limit vectorToLimit(const std::vector<double>& vec) {return cc::Limit{vec[0], vec[1]};}
};


ADASNode::ADASNode() : Node("adas_node") {
    this->declare_parameter("allow_driving", false);
    this->declare_parameter("host_velocity_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("imu_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("v_ref_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("y_vector_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("tau", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ts", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("c", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("s", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("v_ref", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("u_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("phi_vals", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("q_vals", rclcpp::PARAMETER_DOUBLE_ARRAY);

    std::string host_velocity_topic = this->get_parameter("host_velocity_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string control_topic = this->get_parameter("control_topic").as_string();
    std::string v_ref_topic = this->get_parameter("v_ref_topic").as_string();
    std::string y_vector_topic = this->get_parameter("y_vector_topic").as_string();
    double tau = this->get_parameter("tau").as_double();
    ts_ = this->get_parameter("ts").as_double();
    int p = this->get_parameter("p").as_int();
    int c = this->get_parameter("c").as_int();
    double s = this->get_parameter("s").as_double();
    v_ref_ = this->get_parameter("v_ref").as_double();
    cc::Limit u_limits = vectorToLimit(this->get_parameter("u_limits").as_double_array());
    std::vector<double> phi_vals = this->get_parameter("phi_vals").as_double_array();
    std::vector<double> q_vals = this->get_parameter("q_vals").as_double_array();

    RCLCPP_INFO(this->get_logger(), "host_velocity_topic: '%s'", host_velocity_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: '%s'", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic: '%s'", control_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "v_ref_topic: '%s'", v_ref_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "y_vector_topic: '%s'", y_vector_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "tau: %lf s", tau);
    RCLCPP_INFO(this->get_logger(), "tau: %lf s", ts_);
    RCLCPP_INFO(this->get_logger(), "p: %d", p);
    RCLCPP_INFO(this->get_logger(), "c: %d", c);
    RCLCPP_INFO(this->get_logger(), "s: %lf", s);
    RCLCPP_INFO(this->get_logger(), "v_ref: %lf km/h", v_ref_);
    RCLCPP_INFO(this->get_logger(), "u_limits: [%lf, %lf] m/s^2", u_limits.min, u_limits.max);
    RCLCPP_INFO(this->get_logger(), "phi_vals: [%lf, %lf, %lf]", phi_vals[0], phi_vals[1], phi_vals[2]);
    RCLCPP_INFO(this->get_logger(), "q_vals: [%lf, %lf, %lf]", q_vals[0], q_vals[1], q_vals[2]);

    v_ref_ *= kmh2ms;

    linear_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(host_velocity_topic, 10, std::bind(&ADASNode::linearVelocityCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&ADASNode::imuCallback, this, _1));
    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(control_topic, 10);
    v_ref_pub_ = this->create_publisher<std_msgs::msg::Float64>(v_ref_topic, 10);
    y_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(y_vector_topic, 10);

    cruise_controller_ = std::make_unique<cc::CruiseController>(tau, p, c, s, phi_vals, q_vals, u_limits);

    pid_.initPid(0.0, 0.15, 0.0, 0.2, -0.2);

    t_last_ = this->get_clock()->now().seconds();
}


void ADASNode::linearVelocityCallback(const geometry_msgs::msg::Twist& msg) {
    v_ref_ = this->get_parameter("v_ref").as_double() * kmh2ms;

    if (!this->get_parameter("allow_driving").as_bool()) {
        control_pub_->publish(carla_msgs::msg::CarlaEgoVehicleControl());
        v_ref_pub_->publish(std_msgs::msg::Float64());
        return;
    }

    double t = this->get_clock()->now().seconds();
    double dt = t - t_last_;

    double v = msg.linear.x;

    double a = last_imu_msg_.linear_acceleration.x;

    auto control_output = cruise_controller_->calculate_control(ts_, v_ref_, v, a);

    double u_mpc = control_output.first;
    auto y = control_output.second;

    auto dt_ns = static_cast<uint64_t>(rclcpp::Duration::from_seconds(dt).nanoseconds());

    double e = v_ref_ - v;
    if (e * e_prev_ < 0.0) {
        pid_.reset();
    }
    e_prev_ = e;

    double u_trim = pid_.computeCommand(e, dt_ns);
    double max_trim = 0.3 * std::max(0.1, std::abs(u_mpc));
    u_trim = std::clamp(u_trim, -max_trim, max_trim);

    double u = u_mpc + u_trim;

    auto control_msg = carla_msgs::msg::CarlaEgoVehicleControl();

    if (u >= 0) {
        control_msg.throttle = std::clamp(u, 0., 1.);
    }
    else {
        control_msg.brake = std::clamp(-u, 0., 1.);
    }

    control_pub_->publish(control_msg);

    RCLCPP_INFO(
        get_logger(),
        "e=%.3f u_mpc=%.3f u_trim=%.3f u=%.3f thr=%.3f br=%.3f",
        e, u_mpc, u_trim, u, control_msg.throttle, control_msg.brake
    );
    
    auto v_ref_msg = std_msgs::msg::Float64();
    v_ref_msg.data = v_ref_;
    v_ref_pub_->publish(v_ref_msg);

    auto y_msg = geometry_msgs::msg::Vector3();
    y_msg.x = y[0];
    y_msg.y = y[1];
    y_msg.z = y[2];
    y_pub_->publish(y_msg);

    t_last_ = t;
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
