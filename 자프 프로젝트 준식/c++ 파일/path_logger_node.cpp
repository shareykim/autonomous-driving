#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class PathLoggerNode : public rclcpp::Node
{
public:
    PathLoggerNode()
        : Node("path_logger_node")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/raw_path", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(50ms, std::bind(&PathLoggerNode::timerCallback, this));

        path_msg_.header.frame_id = "map";

        min_dist_ = 0.05;
        last_x_ = std::numeric_limits<double>::quiet_NaN();
        last_y_ = std::numeric_limits<double>::quiet_NaN();

        csv_path_ = "/home/misys/shared_dir/raw_path.csv";
        std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());

        RCLCPP_INFO(this->get_logger(), "PathLoggerNode started");
    }

    ~PathLoggerNode()
    {
        saveCSV();
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero, 100ms);
        }
        catch (tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "TF error: %s", ex.what());
            return;
        }

        double x = tf_msg.transform.translation.x;
        double y = tf_msg.transform.translation.y;

        tf2::Quaternion q(
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w
        );
        double yaw = tf2::getYaw(q);

        // 최소 이동 거리 체크
        if (!std::isnan(last_x_)) {
            double dist = std::hypot(x - last_x_, y - last_y_);
            if (dist < min_dist_)
                return;
        }

        last_x_ = x;
        last_y_ = y;

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = this->get_clock()->now();
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = tf_msg.transform.rotation;

        path_msg_.header.stamp = ps.header.stamp;
        path_msg_.poses.push_back(ps);

        points_.push_back({ x, y, yaw });

        path_pub_->publish(path_msg_);
    }

    void saveCSV()
    {
        RCLCPP_INFO(this->get_logger(), "Saving raw_path to %s", csv_path_.c_str());

        std::ofstream file(csv_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write CSV!");
            return;
        }

        file << "x,y,yaw\n";
        for (const auto& p : points_) {
            file << p[0] << "," << p[1] << "," << p[2] << "\n";
        }

        file.close();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::Path path_msg_;

    std::vector<std::array<double, 3>> points_;
    double min_dist_;
    double last_x_;
    double last_y_;

    std::string csv_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathLoggerNode>();

    try {
        rclcpp::spin(node);
    }
    catch (...) {}

    rclcpp::shutdown();
    return 0;
}
