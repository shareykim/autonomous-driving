#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

class MapLoaderNode : public rclcpp::Node {
public:
    MapLoaderNode() : Node("map_loader_node") {
        this->declare_parameter<std::string>("map_yaml", "/home/misys/shared_dir/map.yaml");
        std::string yaml_path = this->get_parameter("map_yaml").as_string();

        map_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/map_np", 10);

        load_map(yaml_path);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MapLoaderNode::publish_map, this)
        );
    }

private:
    cv::Mat map_img_;
    float resolution_;
    std::vector<double> origin_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void load_map(const std::string& yaml_path) {
        YAML::Node doc = YAML::LoadFile(yaml_path);

        std::string image_path = doc["image"].as<std::string>();
        resolution_ = doc["resolution"].as<double>();
        origin_ = doc["origin"].as<std::vector<double>>();

        cv::Mat raw = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        cv::flip(raw, map_img_, 0);

        // convert to occupancy-like representation: 0 wall, 1 free
        map_img_.convertTo(map_img_, CV_32FC1, 1.0 / 255.0);

        RCLCPP_INFO(this->get_logger(), "Loaded map: %s (%dx%d)", 
            image_path.c_str(), map_img_.rows, map_img_.cols);
    }

    void publish_map() {
        if (map_img_.empty()) return;

        std_msgs::msg::Float32MultiArray msg;
        msg.layout.dim.resize(2);
        msg.layout.dim[0].label = "height";
        msg.layout.dim[0].size = map_img_.rows;
        msg.layout.dim[1].label = "width";
        msg.layout.dim[1].size = map_img_.cols;

        msg.data.assign((float*)map_img_.datastart, (float*)map_img_.dataend);

        map_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLoaderNode>());
    rclcpp::shutdown();
    return 0;
}
