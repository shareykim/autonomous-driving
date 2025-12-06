#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <algorithm>

class CenterlineGeneratorNode : public rclcpp::Node {
public:
    CenterlineGeneratorNode() : Node("centerline_generator_node") {
        left_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/left_wall", 10, std::bind(&CenterlineGeneratorNode::left_callback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/right_wall", 10, std::bind(&CenterlineGeneratorNode::right_callback, this, std::placeholders::_1));

        center_pub_ = this->create_publisher<nav_msgs::msg::Path>("/centerline_raw", 10);
    }

private:
    nav_msgs::msg::Path left_wall_, right_wall_;
    bool left_ready_ = false, right_ready_ = false;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_pub_;

    void left_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        left_wall_ = *msg;
        left_ready_ = true;
        generate_centerline();
    }

    void right_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        right_wall_ = *msg;
        right_ready_ = true;
        generate_centerline();
    }

    void generate_centerline() {
        if (!left_ready_ || !right_ready_) return;

        int n = std::min(left_wall_.poses.size(), right_wall_.poses.size());
        if (n < 10) return;

        nav_msgs::msg::Path centerline;
        centerline.header.frame_id = "map";

        for (int i = 0; i < n; i++) {
            float lx = left_wall_.poses[i].pose.position.x;
            float ly = left_wall_.poses[i].pose.position.y;
            float rx = right_wall_.poses[i].pose.position.x;
            float ry = right_wall_.poses[i].pose.position.y;

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = (lx + rx) / 2.0f;
            pose.pose.position.y = (ly + ry) / 2.0f;
            centerline.poses.push_back(pose);
        }

        center_pub_->publish(centerline);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CenterlineGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
