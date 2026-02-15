#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <utility>

class CatmullRomRacelineNode : public rclcpp::Node
{
public:
    CatmullRomRacelineNode()
        : Node("catmullrom_raceline_node"), generated_(false)
    {
        sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/raw_path", 10,
            std::bind(&CatmullRomRacelineNode::pathCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        output_csv_ = "/home/misys/shared_dir/raceline_catmullrom.csv";
        std::filesystem::create_directories(std::filesystem::path(output_csv_).parent_path());

        RCLCPP_INFO(this->get_logger(), "CatmullRomRacelineNode started");
    }

private:
    // ===============================
    // Catmull-Rom Interpolation
    // ===============================
    std::vector<std::pair<double, double>>
        catmullRom(const std::vector<std::pair<double, double>>& pts, int num_per_seg = 10)
    {
        int n = pts.size();
        if (n < 4)
            return pts; // �״�� ��ȯ

        std::vector<std::pair<double, double>> out;
        out.reserve(n * num_per_seg);

        for (int i = 1; i < n - 2; i++)
        {
            auto p0 = pts[i - 1];
            auto p1 = pts[i];
            auto p2 = pts[i + 1];
            auto p3 = pts[i + 2];

            for (int j = 0; j < num_per_seg; j++)
            {
                double u = static_cast<double>(j) / num_per_seg;
                double u2 = u * u;
                double u3 = u2 * u;

                double a_x = 2 * p1.first;
                double b_x = -p0.first + p2.first;
                double c_x = 2 * p0.first - 5 * p1.first + 4 * p2.first - p3.first;
                double d_x = -p0.first + 3 * p1.first - 3 * p2.first + p3.first;

                double a_y = 2 * p1.second;
                double b_y = -p0.second + p2.second;
                double c_y = 2 * p0.second - 5 * p1.second + 4 * p2.second - p3.second;
                double d_y = -p0.second + 3 * p1.second - 3 * p2.second + p3.second;

                double x = 0.5 * (a_x + b_x * u + c_x * u2 + d_x * u3);
                double y = 0.5 * (a_y + b_y * u + c_y * u2 + d_y * u3);

                out.emplace_back(x, y);
            }
        }

        // ������ �� �� �߰�
        out.push_back(pts[n - 2]);
        out.push_back(pts[n - 1]);

        return out;
    }

    // ===============================
    // Path Callback
    // ===============================
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (generated_)
            return;

        int N = msg->poses.size();
        if (N < 4)
        {
            RCLCPP_WARN(this->get_logger(), "Need at least 4 points for Catmull-Rom.");
            return;
        }

        std::vector<std::pair<double, double>> pts;
        pts.reserve(N);

        for (int i = 0; i < N; i++)
        {
            pts.emplace_back(
                msg->poses[i].pose.position.x,
                msg->poses[i].pose.position.y
            );
        }

        auto cr_pts = catmullRom(pts, 10);

        // Path �޽��� ����
        nav_msgs::msg::Path raceline;
        raceline.header.frame_id = msg->header.frame_id;
        raceline.header.stamp = msg->header.stamp;

        for (auto& p : cr_pts)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = raceline.header;
            ps.pose.position.x = p.first;
            ps.pose.position.y = p.second;
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            raceline.poses.push_back(ps);
        }

        pub_->publish(raceline);
        saveCSV(cr_pts);

        RCLCPP_INFO(this->get_logger(), "Catmull-Rom raceline generated. points=%ld", cr_pts.size());
        generated_ = true;
    }

    // ===============================
    // CSV ����
    // ===============================
    void saveCSV(const std::vector<std::pair<double, double>>& pts)
    {
        std::ofstream file(output_csv_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write CSV.");
            return;
        }

        file << "x,y\n";
        for (auto& p : pts)
            file << p.first << "," << p.second << "\n";

        file.close();
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;

    bool generated_;
    std::string output_csv_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CatmullRomRacelineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
