#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>


using std::placeholders::_1;


class SteerTestNode : public rclcpp::Node {
private:
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_in_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr steer_stamped_pub_;

    double steer_amplitude_;
    double steer_frequency_;
    double t_{-1.0};
    double steer_base_;

public:
    SteerTestNode();

private:
    void controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg);
};


SteerTestNode::SteerTestNode() : Node("steer_test_node") {
    this->declare_parameter("allow_driving", false);
    this->declare_parameter("control_in_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("steer_amplitude", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("steer_frequency", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("steer_base", rclcpp::PARAMETER_DOUBLE);

    const std::string control_in_topic = this->get_parameter("control_in_topic").as_string();
    const std::string control_topic = this->get_parameter("control_topic").as_string();
    steer_amplitude_ = this->get_parameter("steer_amplitude").as_double();
    steer_frequency_ = this->get_parameter("steer_frequency").as_double();
    steer_base_ = this->get_parameter("steer_base").as_double();

    RCLCPP_INFO(this->get_logger(), "control_in_topic: '%s'", control_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic: '%s'", control_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "steer_amplitude: %lf", steer_amplitude_);
    RCLCPP_INFO(this->get_logger(), "steer_frequency: %lf Hz", steer_frequency_);
    RCLCPP_INFO(this->get_logger(), "steer_base: %lf", steer_base_);

    control_in_sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleControl>(
        control_in_topic,
        10,
        std::bind(&SteerTestNode::controlInCallback, this, _1));

    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
        control_topic,
        10);

    steer_stamped_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/carla/hero/steer_stamped",
        10);
}


void SteerTestNode::controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg) {
    
    if (!this->get_parameter("allow_driving").as_bool()) {
        control_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "allow_driving=false, publishing control_in");
        return;
    }

    const double now = this->get_clock()->now().seconds();

    if (t_ <= 0.0) {
        t_ = now;
    }

    const double elapsed = now - t_;

    const double steer = steer_base_ + steer_amplitude_ * std::sin(2.0 * M_PI * steer_frequency_ * elapsed);

    auto control_msg = msg;
    control_msg.header.stamp = this->get_clock()->now();
    control_msg.steer = steer;

    control_pub_->publish(control_msg);

    geometry_msgs::msg::Vector3Stamped steer_stamped;
    steer_stamped.header.stamp = control_msg.header.stamp;
    steer_stamped.vector.x = steer;
    steer_stamped_pub_->publish(steer_stamped);

    RCLCPP_INFO(this->get_logger(), "steer=%.3f", control_msg.steer);
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerTestNode>());
    rclcpp::shutdown();
    return 0;
}
