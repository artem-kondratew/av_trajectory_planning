#include <chrono>
#include <iostream>
#include <memory>

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

public:
    ADASNode();

private:
    void linearVelocityCallback(const geometry_msgs::msg::Twist& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);
};


ADASNode::ADASNode() : Node("cruise_control_node") {
    linear_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/carla/hero/local_velocity", 10, std::bind(&ADASNode::linearVelocityCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/carla/hero/local_velocity", 10, std::bind(&ADASNode::imuCallback, this, _1));
}


void ADASNode::linearVelocityCallback(const geometry_msgs::msg::Twist& msg) {
    
}

void ADASNode::imuCallback(const sensor_msgs::msg::Imu& msg) {

}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ADASNode>()); 
    rclcpp::shutdown();

    return 0;
}