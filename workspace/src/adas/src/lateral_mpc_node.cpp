#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/lateral_mpc_state.hpp>

#include "adas/lateral_mpc.hpp"


using std::placeholders::_1;


class LateralMPCNode : public rclcpp::Node {
private:
    rclcpp::Subscription<carla_msgs::msg::LateralMpcState>::SharedPtr lateral_mpc_state_sub_;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_in_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;

    std::unique_ptr<lateral_control::LateralMPC> lateral_mpc_;

    double max_steer_rad_;
    double steer_cmd_ = 0.0;
    bool has_steer_ = false;

public:
    LateralMPCNode();

private:
    void lateralMpcStateCallback(const carla_msgs::msg::LateralMpcState& msg);
    void controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg);
};


LateralMPCNode::LateralMPCNode() : Node("lateral_mpc_node") {
    this->declare_parameter("allow_driving", false);
    this->declare_parameter("lateral_mpc_state_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_in_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("tau", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ts", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("p", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("c", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("L", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("s", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("q_vals", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("u_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("max_steer_rad", rclcpp::PARAMETER_DOUBLE);

    const double tau          = this->get_parameter("tau").as_double();
    const double ts           = this->get_parameter("ts").as_double();
    const int    p            = this->get_parameter("p").as_int();
    const int    c            = this->get_parameter("c").as_int();
    const double L            = this->get_parameter("L").as_double();
    const double s            = this->get_parameter("s").as_double();
    const auto   q_vals       = this->get_parameter("q_vals").as_double_array();
    const Limit  u_limits     = Limit::vectorToLimit(this->get_parameter("u_limits").as_double_array());
    max_steer_rad_             = this->get_parameter("max_steer_rad").as_double();

    const std::string lateral_mpc_state_topic = this->get_parameter("lateral_mpc_state_topic").as_string();
    const std::string control_in_topic        = this->get_parameter("control_in_topic").as_string();
    const std::string control_topic           = this->get_parameter("control_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "tau:            %.3f s",   tau);
    RCLCPP_INFO(this->get_logger(), "ts:             %.3f s",   ts);
    RCLCPP_INFO(this->get_logger(), "p:              %d",       p);
    RCLCPP_INFO(this->get_logger(), "c:              %d",       c);
    RCLCPP_INFO(this->get_logger(), "L:              %.3f m",   L);
    RCLCPP_INFO(this->get_logger(), "s:              %.3f",     s);
    RCLCPP_INFO(this->get_logger(), "q_vals:         [%.1f, %.1f, %.1f]", q_vals[0], q_vals[1], q_vals[2]);
    RCLCPP_INFO(this->get_logger(), "u_limits:       [%.3f, %.3f] rad", u_limits.min, u_limits.max);
    RCLCPP_INFO(this->get_logger(), "max_steer_rad:  %.4f rad", max_steer_rad_);
    RCLCPP_INFO(this->get_logger(), "lateral_mpc_state_topic: '%s'", lateral_mpc_state_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_in_topic:        '%s'", control_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic:           '%s'", control_topic.c_str());

    lateral_mpc_state_sub_ = this->create_subscription<carla_msgs::msg::LateralMpcState>(
        lateral_mpc_state_topic,
        10,
        std::bind(&LateralMPCNode::lateralMpcStateCallback, this, _1)
    );

    control_in_sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleControl>(
        control_in_topic,
        10,
        std::bind(&LateralMPCNode::controlInCallback, this, _1)
    );

    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
        control_topic,
        10
    );

    lateral_mpc_ = std::make_unique<lateral_control::LateralMPC>(ts, tau, p, c, L, s, q_vals, u_limits);
}


void LateralMPCNode::lateralMpcStateCallback(const carla_msgs::msg::LateralMpcState& msg) {
    const auto [delta_rad, state] = lateral_mpc_->calculate_control(
        msg.e, msg.theta, msg.delta, msg.kappa_ref, msg.vx
    );

    // Convert steering angle (rad) → Carla normalised steer [-1, 1]
    steer_cmd_ = std::clamp(delta_rad / max_steer_rad_, -1.0, 1.0);
    has_steer_ = true;
}


void LateralMPCNode::controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg) {
    if (!this->get_parameter("allow_driving").as_bool()) {
        control_pub_->publish(msg);
        return;
    }

    if (!has_steer_) {
        RCLCPP_WARN(this->get_logger(), "no steer computed yet");
        return;
    }

    auto control_msg = msg;
    control_msg.steer = steer_cmd_;
    control_pub_->publish(control_msg);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LateralMPCNode>());
    rclcpp::shutdown();

    return 0;
}
