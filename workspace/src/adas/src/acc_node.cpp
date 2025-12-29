#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <control_toolbox/pid.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include "adas/acc_mpc.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace acc = adaptive_cruise_control;
using ACC = acc::AdaptiveCruiseController;


class AdaptiveCruiseControlNode : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr host_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr dummy_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr host_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dummy_velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_ref_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr y_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allow_sub_;

    std::unique_ptr<ACC> cruise_controller_;

    double ts_;

    std::optional<Eigen::Vector2d> x_host_;
    std::optional<Eigen::Vector2d> x_dummy_;
    std::optional<double> v_dummy_;

    double d0_;
    double th_;
    
    sensor_msgs::msg::Imu last_imu_msg_;
    double t_last_;
    double e_prev_ = 0.0;

    static constexpr double ms2kmh = 3.6;
    static constexpr double kmh2ms = 1 / ms2kmh;

    carla_msgs::msg::CarlaEgoVehicleControl stop_msg_;

    bool allow_acc_ = false;

public:
    AdaptiveCruiseControlNode();

private:
    void hostPositionCallback(const geometry_msgs::msg::Vector3& msg);
    void dummyPositionCallback(const geometry_msgs::msg::Vector3& msg);
    void hostVelocityCallback(const std_msgs::msg::Float64& msg);
    void dummyVelocityCallback(const std_msgs::msg::Float64& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);

    Limit vectorToLimit(const std::vector<double>& vec) {return Limit{vec[0], vec[1]};}

    void allowCallback(const std_msgs::msg::Bool& msg);
};


AdaptiveCruiseControlNode::AdaptiveCruiseControlNode() : Node("acc_node") {
    this->declare_parameter("allow_driving", true);
    this->declare_parameter("host_position_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("dummy_position_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("host_velocity_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("dummy_velocity_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("imu_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("dist_ref_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("dist_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("y_vector_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("tau", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ts", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("c", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("s", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("d0", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("th", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("u_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("phi_vals", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("q_vals", rclcpp::PARAMETER_DOUBLE_ARRAY);

    std::string host_position_topic = this->get_parameter("host_position_topic").as_string();
    std::string dummy_position_topic = this->get_parameter("dummy_position_topic").as_string();
    std::string host_velocity_topic = this->get_parameter("host_velocity_topic").as_string();
    std::string dummy_velocity_topic = this->get_parameter("dummy_velocity_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string control_topic = this->get_parameter("control_topic").as_string();
    std::string dist_ref_topic = this->get_parameter("dist_ref_topic").as_string();
    std::string dist_topic = this->get_parameter("dist_topic").as_string();
    std::string y_vector_topic = this->get_parameter("y_vector_topic").as_string();
    double tau = this->get_parameter("tau").as_double();
    ts_ = this->get_parameter("ts").as_double();
    int p = this->get_parameter("p").as_int();
    int c = this->get_parameter("c").as_int();
    double s = this->get_parameter("s").as_double();
    d0_ = this->get_parameter("d0").as_double();
    th_ = this->get_parameter("th").as_double();
    Limit u_limits = vectorToLimit(this->get_parameter("u_limits").as_double_array());
    std::vector<double> phi_vals = this->get_parameter("phi_vals").as_double_array();
    std::vector<double> q_vals = this->get_parameter("q_vals").as_double_array();

    RCLCPP_INFO(this->get_logger(), "host_position_topic: '%s'", host_position_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dummy_position_topic: '%s'", dummy_position_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "host_velocity_topic: '%s'", host_velocity_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dummy_velocity_topic: '%s'", dummy_velocity_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: '%s'", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic: '%s'", control_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dist_ref_topic: '%s'", dist_ref_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dist_topic: '%s'", dist_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "y_vector_topic: '%s'", y_vector_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "tau: %lf s", tau);
    RCLCPP_INFO(this->get_logger(), "tau: %lf s", ts_);
    RCLCPP_INFO(this->get_logger(), "p: %d", p);
    RCLCPP_INFO(this->get_logger(), "c: %d", c);
    RCLCPP_INFO(this->get_logger(), "s: %lf", s);
    RCLCPP_INFO(this->get_logger(), "d0: %lf", d0_);
    RCLCPP_INFO(this->get_logger(), "th: %lf", th_);
    RCLCPP_INFO(this->get_logger(), "u_limits: [%lf, %lf] m/s^2", u_limits.min, u_limits.max);
    RCLCPP_INFO(this->get_logger(), "phi_vals: [%lf, %lf, %lf]", phi_vals[0], phi_vals[1], phi_vals[2]);
    RCLCPP_INFO(this->get_logger(), "q_vals: [%lf, %lf, %lf]", q_vals[0], q_vals[1], q_vals[2]);

    host_position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(host_position_topic, 10, std::bind(&AdaptiveCruiseControlNode::hostPositionCallback, this, _1));
    dummy_position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(dummy_position_topic, 10, std::bind(&AdaptiveCruiseControlNode::dummyPositionCallback, this, _1));
    host_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(host_velocity_topic, 10, std::bind(&AdaptiveCruiseControlNode::hostVelocityCallback, this, _1));
    dummy_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(dummy_velocity_topic, 10, std::bind(&AdaptiveCruiseControlNode::dummyVelocityCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&AdaptiveCruiseControlNode::imuCallback, this, _1));
    allow_sub_ = this->create_subscription<std_msgs::msg::Bool>("/allow_acc", 10, std::bind(&AdaptiveCruiseControlNode::allowCallback, this, _1));
    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(control_topic, 10);
    dist_ref_pub_ = this->create_publisher<std_msgs::msg::Float64>(dist_ref_topic, 10);
    dist_pub_ = this->create_publisher<std_msgs::msg::Float64>(dist_topic, 10);
    y_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(y_vector_topic, 10);

    cruise_controller_ = std::make_unique<ACC>(tau, p, c, s, d0_, th_, phi_vals, q_vals, u_limits);

    stop_msg_ = carla_msgs::msg::CarlaEgoVehicleControl();
    stop_msg_.brake = 1.;

    t_last_ = this->get_clock()->now().seconds();
}


void AdaptiveCruiseControlNode::allowCallback(const std_msgs::msg::Bool& msg) {
    allow_acc_ = msg.data;
}


void AdaptiveCruiseControlNode::hostVelocityCallback(const std_msgs::msg::Float64& msg) {
    std::cout << "ok0" << std::endl;
    if (!allow_acc_) {
        cruise_controller_->resetSolver();
        return;
    }
    std::cout << "ok1" << std::endl;

    if (!this->get_parameter("allow_driving").as_bool()) {
        control_pub_->publish(stop_msg_);
        return;
    }

    std::cout << "ok2" << std::endl;

    if (!x_host_ || !x_dummy_ || !v_dummy_) {
        return;
    }
    std::cout << "ok3" << std::endl;

    double v = msg.data;  // m/s

    double dist_ref = d0_ + th_ * v;

    Eigen::Vector2d dx_vec = *x_dummy_ - *x_host_;
    double dist = dx_vec.norm();

    x_host_.reset();
    x_dummy_.reset();

    auto dist_ref_msg = std_msgs::msg::Float64();
    dist_ref_msg.data = dist_ref;

    auto dist_msg = std_msgs::msg::Float64();
    dist_msg.data = dist;

    dist_ref_pub_->publish(dist_ref_msg);
    dist_pub_->publish(dist_msg);

#if false
    double t = this->get_clock()->now().seconds();
    double dt = t - t_last_;
#endif

    double a = last_imu_msg_.linear_acceleration.x;

    double v_rel = *v_dummy_ - v;

    std::cout << "ok4" << std::endl;

    auto control_output = cruise_controller_->calculate_control(ts_, dist, v, v_rel, a);

    std::cout << "ok5" << std::endl;

    double u = control_output.first;
    auto y = control_output.second;

    auto control_msg = carla_msgs::msg::CarlaEgoVehicleControl();

    if (u >= 0) {
        control_msg.throttle = std::clamp(u, 0., 1.);
    }
    else {
        control_msg.brake = std::clamp(-u, 0., 1.);
    }

    control_pub_->publish(control_msg);

    double e = dist - dist_ref;

    RCLCPP_DEBUG(
        get_logger(),
        "e=%.3f u=%.3f thr=%.3f br=%.3f",
        e, u, control_msg.throttle, control_msg.brake
    );

    auto y_msg = geometry_msgs::msg::Vector3();
    y_msg.x = y[0];
    y_msg.y = y[1];
    y_msg.z = y[2];
    y_pub_->publish(y_msg);

#if false
    t_last_ = t;
#endif
}


void AdaptiveCruiseControlNode::hostPositionCallback(const geometry_msgs::msg::Vector3& msg) {
    x_host_.emplace(msg.x, msg.y);
}


void AdaptiveCruiseControlNode::dummyPositionCallback(const geometry_msgs::msg::Vector3& msg) {
    x_dummy_.emplace(msg.x, msg.y);
}


void AdaptiveCruiseControlNode::dummyVelocityCallback(const std_msgs::msg::Float64& msg) {
    v_dummy_ = msg.data;  // m/s
}


void AdaptiveCruiseControlNode::imuCallback(const sensor_msgs::msg::Imu& msg) {
    last_imu_msg_ = msg;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveCruiseControlNode>()); 
    rclcpp::shutdown();

    return 0;
}
