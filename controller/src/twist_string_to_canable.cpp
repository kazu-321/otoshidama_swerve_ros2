#include "otoshidama_swerve_controller/twist_string_to_canable.hpp"

namespace otoshidama_swerve_controller {
    twist_string_to_canable::twist_string_to_canable(const rclcpp::NodeOptions & options)
    : Node("twist_string_to_canable", options)
    {
        canable_pub_ = this->create_publisher<canable_msgs::msg::Can>("/can/transmit", 10);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&twist_string_to_canable::twist_callback, this, std::placeholders::_1));
        string_sub_ = this->create_subscription<std_msgs::msg::String>(
            "cmd", 10, std::bind(&twist_string_to_canable::string_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300), [this]() {
                canable_msgs::msg::Can can_msg;
                can_msg.id = 0x001;
                can_msg.dlc = 1;
                memcpy(&can_msg.data, &canmsg.data, sizeof(canmsg.data));
                canable_pub_->publish(can_msg);
            });
        canmsg.data.emg = 0;
        canmsg.data.reset = 0;
        canmsg.x = 0.0;
        canmsg.y = 0.0;
        canmsg.z = 0.0;
        canmsg.data.reserved = 0;
    }

    void twist_string_to_canable::string_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "continue") {
            canmsg.data.emg = 0;
        } else if( msg->data == "pause") {
            canmsg.data.emg = 1;
        } else if (msg->data == "reset") {
            canmsg.data.reset = 1;
        }
    }

    void twist_string_to_canable::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        canmsg.x = msg->linear.x;
        canmsg.y = msg->linear.y;
        canmsg.z = msg->angular.z;
        canmsg.data.reserved = 0;
        canable_msgs::msg::Can can_msg;
        can_msg.id = 0x000;
        can_msg.dlc = 1;
        memcpy(&can_msg.data, &canmsg.data, sizeof(canmsg.data));
        canable_pub_->publish(can_msg);
        canmsg.data.reset = 0;

        can_msg.id = 0x001;
        can_msg.dlc = 8;
        memcpy(&can_msg.data[0], &canmsg.x, sizeof(canmsg.x));
        memcpy(&can_msg.data[4], &canmsg.y, sizeof(canmsg.y));
        canable_pub_->publish(can_msg);

        can_msg.id = 0x002;
        can_msg.dlc = 4;
        memcpy(&can_msg.data[0], &canmsg.z, sizeof(canmsg.z));
        canable_pub_->publish(can_msg);
    }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(otoshidama_swerve_controller::twist_string_to_canable)