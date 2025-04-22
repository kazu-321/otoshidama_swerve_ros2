#ifndef __TWIST_TO_CANABLE_HPP__
#define __TWIST_TO_CANABLE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <canable_msgs/msg/can.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace otoshidama_swerve_controller {
    class twist_to_canable : public rclcpp::Node {
    public:
        twist_to_canable(const rclcpp::NodeOptions & options);

    private:
        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        rclcpp::Publisher<canable_msgs::msg::Can>::SharedPtr canable_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

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

#endif//__TWIST_TO_CANABLE_HPP__