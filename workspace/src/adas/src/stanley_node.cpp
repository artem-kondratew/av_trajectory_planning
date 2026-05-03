#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/stanley_state.hpp>


using std::placeholders::_1;


class StanleyNode : public rclcpp::Node {
private:
    rclcpp::Subscription<carla_msgs::msg::StanleyState>::SharedPtr stanley_sub_;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_in_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;

    double max_steer_rad_ = 1.0;
    double k_ = 1.0;

    double last_steer_ = 0.0;
    bool has_steer_ = false;

public:
    StanleyNode();

private:
    void stanleyCallback(const carla_msgs::msg::StanleyState& msg);
    void controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg);
};


StanleyNode::StanleyNode() : Node("stanley_node") {
    this->declare_parameter("allow_driving", false);
    this->declare_parameter("stanley_state_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_in_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("max_steer_rad", 1.0);
    this->declare_parameter("k", 1.0);

    std::string stanley_topic = this->get_parameter("stanley_state_topic").as_string();
    std::string control_in_topic = this->get_parameter("control_in_topic").as_string();
    std::string control_topic = this->get_parameter("control_topic").as_string();

    max_steer_rad_ = this->get_parameter("max_steer_rad").as_double();
    k_ = this->get_parameter("k").as_double();

    RCLCPP_INFO(this->get_logger(), "stanley_state_topic: '%s'", stanley_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_in_topic: '%s'", control_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic: '%s'", control_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "max_steer_rad: %.3f", max_steer_rad_);
    RCLCPP_INFO(this->get_logger(), "k: %.3f", k_);

    stanley_sub_ = this->create_subscription<carla_msgs::msg::StanleyState>(
        stanley_topic,
        10,
        std::bind(&StanleyNode::stanleyCallback, this, _1)
    );

    control_in_sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleControl>(
        control_in_topic,
        10,
        std::bind(&StanleyNode::controlInCallback, this, _1)
    );

    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
        control_topic,
        10
    );
}


void StanleyNode::stanleyCallback(const carla_msgs::msg::StanleyState& msg) {
    if (!std::isfinite(msg.yaw_vehicle) || !std::isfinite(msg.yaw_path) || !std::isfinite(msg.e) ||
        !std::isfinite(msg.v)) {
        return;
    }

    const double v = std::max(msg.v, 1.0);

    double psi = msg.yaw_path - msg.yaw_vehicle;
    psi = std::atan2(std::sin(psi), std::cos(psi));

    const double delta = psi + std::atan2(k_ * msg.e, v);

    double steer_cmd = 0.0;

    if (max_steer_rad_ > 1e-6) {
        steer_cmd = std::clamp(delta / max_steer_rad_, -1.0, 1.0);
    }

    last_steer_ = steer_cmd;
    has_steer_ = true;
}


void StanleyNode::controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg) {
    if (!this->get_parameter("allow_driving").as_bool()) {
        return;
    }

    if (!has_steer_) {
        RCLCPP_WARN(this->get_logger(), "no steer yet");
        return;
    }

    auto control_msg = msg;
    control_msg.steer = last_steer_;

    control_pub_->publish(control_msg);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyNode>());
    rclcpp::shutdown();

    return 0;
}
