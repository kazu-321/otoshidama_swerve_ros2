#ifndef __OTOSHIDAMA_SWERVE_VISUALIZATION__VISUALIZE_SWERVE_HPP__
#define __OTOSHIDAMA_SWERVE_VISUALIZATION__VISUALIZE_SWERVE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <canable_msgs/msg/can.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace otoshidama_swerve_visualization {
    class visualize_swerve : public rclcpp::Node {
    public:
        visualize_swerve(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        const float wheel_radius = 0.03;
        const float wheel_position = 0.2;
        const float wheel_positions[4][2] = {
            {+ wheel_position, + wheel_position},
            {- wheel_position, + wheel_position},
            {- wheel_position, - wheel_position},
            {+ wheel_position, - wheel_position}
        };
        union float_bytes {
            uint8_t bytes[4];
            float value;
        };

        float angle[4] = {0.0, 0.0, 0.0, 0.0};
        float speed[4] = {0.0, 0.0, 0.0, 0.0};

        void visualize_swerve_callback(const canable_msgs::msg::Can::SharedPtr msg);
        rclcpp::Subscription<canable_msgs::msg::Can>::SharedPtr can_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        visualization_msgs::msg::MarkerArray marker_array_;
    };
};

#endif//__OTOSHIDAMA_SWERVE_VISUALIZATION__VISUALIZE_SWERVE_HPP__