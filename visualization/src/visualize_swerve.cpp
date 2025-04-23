#include "otoshidama_swerve_visualization/visualize_swerve.hpp"

namespace otoshidama_swerve_visualization {
    visualize_swerve::visualize_swerve(const rclcpp::NodeOptions & options)
        : Node("visualize_swerve", options) {
        can_sub_ = this->create_subscription<canable_msgs::msg::Can>(
            "/can/receive", 10,
            std::bind(&visualize_swerve::visualize_swerve_callback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/swerve", 10);
        twist_pub_  = this->create_publisher<geometry_msgs::msg::TwistStamped>("/visualization/twist", 10);
        marker_array_.markers.resize(4);
    }

    void visualize_swerve::visualize_swerve_callback(const canable_msgs::msg::Can::SharedPtr msg) {
        // msg.id (0x101~0x104)
        // msg.data[0~3] float angle
        // msg.data[4~7] float speed
        
        float_bytes angle_bytes, speed_bytes;
        memcpy(&angle_bytes.bytes, msg->data.data()+0, sizeof(float));
        memcpy(&speed_bytes.bytes, msg->data.data()+4, sizeof(float));
        
        angle[msg->id - 0x101] = angle_bytes.value;
        speed[msg->id - 0x101] = speed_bytes.value;
        
        marker_array_.markers.clear();
        for(int i = 0; i < 4; i++) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "swerve";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wheel_positions[marker.id][0];
            marker.pose.position.y = wheel_positions[marker.id][1];
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = sin(angle[i] / 2.);
            marker.pose.orientation.w = cos(angle[i] / 2.);
            marker.scale.x = wheel_radius + speed[i] / 1000.f;
            marker.scale.y = wheel_radius;
            marker.scale.z = wheel_radius;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(0, 0);
            marker_array_.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array_);


        float ATA[3][3] = {};  // A^T * A
        float ATb[3] = {};     // A^T * b
    
        for (int i = 0; i < 4; ++i) {
            float theta = angle[i];
            float v = speed[i] / 60.0f * 2.0f * M_PI * wheel_radius;
    
            float dir_x = std::cos(theta);
            float dir_y = std::sin(theta);
            float rx = wheel_positions[i][0];
            float ry = wheel_positions[i][1];
    
            float ax[3] = {1.0f, 0.0f, -ry};
            float ay[3] = {0.0f, 1.0f, +rx};
    
            float bx = v * dir_x;
            float by = v * dir_y;
    
            // ATA += ax * ax^T + ay * ay^T
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    ATA[r][c] += ax[r] * ax[c] + ay[r] * ay[c];
                }
                ATb[r] += ax[r] * bx + ay[r] * by;
            }
        }
    
        // 解く: ATA * x = ATb をガウス消去法で解く（サイズ小さいので直接展開）
        float x[3] = {};
        // まず ATA をコピー（簡潔化のため）
        float A[3][4] = {
            {ATA[0][0], ATA[0][1], ATA[0][2], ATb[0]},
            {ATA[1][0], ATA[1][1], ATA[1][2], ATb[1]},
            {ATA[2][0], ATA[2][1], ATA[2][2], ATb[2]}
        };
    
        // ガウス消去法
        for (int i = 0; i < 3; ++i) {
            // ピボットの正規化
            float pivot = A[i][i];
            for (int j = i; j < 4; ++j) A[i][j] /= pivot;
    
            // 他の行の消去
            for (int k = 0; k < 3; ++k) {
                if (k == i) continue;
                float factor = A[k][i];
                for (int j = i; j < 4; ++j) {
                    A[k][j] -= factor * A[i][j];
                }
            }
        }
    
        x[0] = A[0][3];
        x[1] = A[1][3];
        x[2] = A[2][3];
    
        // 結果をTwistStampedでパブリッシュ
        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = "base_link";
        twist.header.stamp = this->now();
        twist.twist.linear.x = x[0];
        twist.twist.linear.y = x[1];
        twist.twist.angular.z = x[2];
    
        twist_pub_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "x: %.3f, y: %.3f, z: %.3f",
                    x[0], x[1], x[2]);
        
    }
} // namespace otoshidama_swerve_visualization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(otoshidama_swerve_visualization::visualize_swerve)