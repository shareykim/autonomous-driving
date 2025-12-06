#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <opencv2/opencv.hpp>

class BoundaryExtractorNode : public rclcpp::Node {
public:
    BoundaryExtractorNode() : Node("boundary_extractor_node") {

        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/map_np", 10, std::bind(&BoundaryExtractorNode::map_callback, this, std::placeholders::_1));

        left_pub_  = this->create_publisher<nav_msgs::msg::Path>("/left_wall", 10);
        right_pub_ = this->create_publisher<nav_msgs::msg::Path>("/right_wall", 10);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr left_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr right_pub_;

    void map_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        int h = msg->layout.dim[0].size;
        int w = msg->layout.dim[1].size;

        cv::Mat img(h, w, CV_32FC1, (void*)msg->data.data());
        cv::Mat img_u8;
        img.convertTo(img_u8, CV_8UC1, 255.0);
        cv::flip(img_u8, img_u8, 0);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img_u8, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough contours for left/right wall extraction");
            return;
        }

        std::sort(contours.begin(), contours.end(), 
            [](auto &a, auto &b){ return cv::contourArea(a) > cv::contourArea(b); });

        publish_path(contours[0], left_pub_);
        publish_path(contours[1], right_pub_);
    }

    void publish_path(const std::vector<cv::Point>& contour, 
                      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";

        for (const cv::Point& p : contour) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            path.poses.push_back(pose);
        }
        pub->publish(path);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoundaryExtractorNode>());
    rclcpp::shutdown();
    return 0;
}
