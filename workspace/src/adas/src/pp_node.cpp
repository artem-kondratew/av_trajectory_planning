#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/pp_state.hpp>


using std::placeholders::_1;


class PurePursuitNode : public rclcpp::Node {
private:
    rclcpp::Subscription<carla_msgs::msg::PPState>::SharedPtr pp_state_sub_;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_in_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;

    double max_steer_rad_ = 1.0;

    double last_steer_ = 0.0;
    bool has_steer_ = false;

public:
    PurePursuitNode();

private:
    void ppStateCallback(const carla_msgs::msg::PPState& msg);
    void controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg);
};


PurePursuitNode::PurePursuitNode() : Node("pp_node") {
    this->declare_parameter("allow_driving", false);
    this->declare_parameter("pp_state_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_in_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("max_steer_rad", 1.0);

    std::string pp_state_topic = this->get_parameter("pp_state_topic").as_string();
    std::string control_in_topic = this->get_parameter("control_in_topic").as_string();
    std::string control_topic = this->get_parameter("control_topic").as_string();
    max_steer_rad_ = this->get_parameter("max_steer_rad").as_double();

    RCLCPP_INFO(this->get_logger(), "pp_state_topic: '%s'", pp_state_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_in_topic: '%s'", control_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic: '%s'", control_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "max_steer_rad: %.3f", max_steer_rad_);

    pp_state_sub_ = this->create_subscription<carla_msgs::msg::PPState>(
        pp_state_topic,
        10,
        std::bind(&PurePursuitNode::ppStateCallback, this, _1)
    );
    
    control_in_sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleControl>
    (
        control_in_topic,
        10,
        std::bind(&PurePursuitNode::controlInCallback, this, _1)
    );

    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(control_topic, 10);
}


void PurePursuitNode::ppStateCallback(const carla_msgs::msg::PPState& msg) {
    const double lx = msg.xt - msg.xr;
    const double ly = msg.yt - msg.yr;

    if (!std::isfinite(lx) || !std::isfinite(ly)) {
        return;
    }

    const double l = std::hypot(lx, ly);

    double steer_cmd = 0.0;

    if (l > 1e-6 && msg.wheelbase > 1e-6 && max_steer_rad_ > 1e-6) {
        const double alpha = std::atan2(ly, lx) - msg.yaw;
        const double delta = std::atan2(2.0 * msg.wheelbase * std::sin(alpha), l);
        steer_cmd = std::clamp(delta / max_steer_rad_, -1.0, 1.0);
    }

    last_steer_ = steer_cmd;
    has_steer_ = true;
}


void PurePursuitNode::controlInCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg) {
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
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();

    return 0;
}
