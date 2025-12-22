#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <std_msgs/msg/float64.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;


class ControlWrapper : public rclcpp::Node {
private:
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr cc_control_sub_;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr acc_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_sub_;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handler_;

    bool use_acc_ = false;
    bool allow_acc_;

    std::optional<double> dist_ref_;
    std::optional<double> dist_;
    std::optional<double> delta_;

public:
    ControlWrapper();

private:
    void calcDelta();
    void checkSwitchCondition();
    void logState();
    void cruiseControlCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg);
    void adaptiveCruiseControlCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg);
    void distRefCallback(const std_msgs::msg::Float64& msg);
    void distCallback(const std_msgs::msg::Float64& msg);

    rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> & params);
};


ControlWrapper::ControlWrapper() : Node("control_wrapper") {
    this->declare_parameter("cc_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("acc_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("control_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("dist_ref_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("dist_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("allow_acc", false);

    std::string cc_topic = this->get_parameter("cc_topic").as_string();
    std::string acc_topic = this->get_parameter("acc_topic").as_string();
    std::string control_topic = this->get_parameter("control_topic").as_string();
    std::string dist_ref_topic = this->get_parameter("dist_ref_topic").as_string();
    std::string dist_topic = this->get_parameter("dist_topic").as_string();
    allow_acc_ = this->get_parameter("allow_acc").as_bool();

    RCLCPP_INFO(this->get_logger(), "cc_topic: '%s'", cc_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "acc_topic: '%s'", acc_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_topic: '%s'", control_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dist_ref_topic: '%s'", dist_ref_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dist_topic: '%s'", dist_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "allow_acc: '%s'", allow_acc_ ? "true" : "false");

    cc_control_sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleControl>(cc_topic, 10, std::bind(&ControlWrapper::cruiseControlCallback, this, _1));
    acc_control_sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleControl>(acc_topic, 10, std::bind(&ControlWrapper::adaptiveCruiseControlCallback, this, _1));
    dist_ref_sub_ = this->create_subscription<std_msgs::msg::Float64>(dist_ref_topic, 10, std::bind(&ControlWrapper::distRefCallback, this, _1));
    dist_sub_ = this->create_subscription<std_msgs::msg::Float64>(dist_topic, 10, std::bind(&ControlWrapper::distCallback, this, _1));
    control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(control_topic, 10);

    cb_handler_ = this->add_on_set_parameters_callback(std::bind(&ControlWrapper::onParamChange, this, std::placeholders::_1));
}


void ControlWrapper::calcDelta() {
    double d = *dist_ - 2 * (*dist_ref_);
    delta_.emplace(d);
}


void ControlWrapper::checkSwitchCondition() {
    if (!dist_ref_ || !dist_) {
        return;
    }

    calcDelta();

    use_acc_ = (allow_acc_ && (*delta_ < 0)) ? true : false;
}


void ControlWrapper::logState() {
    if (!dist_ref_ || !dist_) {
        return;
    }

    auto mode = (use_acc_) ? "acc" : "cc";
    RCLCPP_INFO(this->get_logger(), "mode: %s, dist_ref: %lf m, dist: %lf m, dist - 2*dist_ref: %lf", mode, *dist_ref_, *dist_, *delta_);
}


void ControlWrapper::cruiseControlCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg) {
    if (use_acc_) {
        return;
    }

    control_pub_->publish(msg);
}


void ControlWrapper::adaptiveCruiseControlCallback(const carla_msgs::msg::CarlaEgoVehicleControl& msg) {
    if (!use_acc_) {
        return;
    }

    control_pub_->publish(msg);
}


void ControlWrapper::distRefCallback(const std_msgs::msg::Float64& msg) {
    dist_ref_.emplace(msg.data);
    checkSwitchCondition();
    logState();
}


void ControlWrapper::distCallback(const std_msgs::msg::Float64& msg) {
    dist_.emplace(msg.data);
    checkSwitchCondition();
}


rcl_interfaces::msg::SetParametersResult ControlWrapper::onParamChange(const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
        if (p.get_name() == "allow_acc") {
            allow_acc_ = p.as_bool();
            checkSwitchCondition();
        }
    }

    return result;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlWrapper>()); 
    rclcpp::shutdown();

    return 0;
}
