#ifndef __TWIST_STRING_TO_CANABLE_HPP__
#define __TWIST_STRING_TO_CANABLE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <canable_msgs/msg/can.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

namespace otoshidama_swerve_controller {
    class twist_string_to_canable : public rclcpp::Node {
    public:
        twist_string_to_canable(const rclcpp::NodeOptions & options);

    private:
        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void string_callback(const std_msgs::msg::String::SharedPtr msg);
        rclcpp::Publisher<canable_msgs::msg::Can>::SharedPtr canable_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        struct canmsg_s {
            struct {
                uint8_t emg : 1;
                uint8_t reset : 1;
                uint8_t reserved : 6;
            } data;
            float x;
            float y;
            float z;
        } canmsg;

        union float_bytes {
            uint8_t bytes[4];
            float value;
        };
    };
};

#endif//__TWIST_STRING_TO_CANABLE_HPP__