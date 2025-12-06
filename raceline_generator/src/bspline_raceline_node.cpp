#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <algorithm>

class BSplineRacelineNode : public rclcpp::Node
{
public:
    BSplineRacelineNode()
        : Node("bspline_raceline_node"), generated_(false)
    {
        sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/raw_path", 10,
            std::bind(&BSplineRacelineNode::pathCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        output_csv_ = "/home/misys/shared_dir/raceline_bspline.csv";
        std::filesystem::create_directories(std::filesystem::path(output_csv_).parent_path());

        eps_ = 0.02; // Douglas-Peucker tolerance

        RCLCPP_INFO(this->get_logger(), "BSplineRacelineNode started");
    }

private:
    // ============================================================
    // Douglas-Peucker Simplification
    // ============================================================
    std::vector<std::pair<double, double>>
        dpSimplify(const std::vector<std::pair<double, double>>& pts, double eps)
    {
        if (pts.size() < 3)
            return pts;

        size_t n = pts.size();
        auto start = pts.front();
        auto end = pts.back();

        double sx = start.first;
        double sy = start.second;
        double ex = end.first;
        double ey = end.second;

        double vx = ex - sx;
        double vy = ey - sy;
        double v_len2 = vx * vx + vy * vy + 1e-9;

        double max_dist = -1.0;
        int index = -1;

        for (size_t i = 1; i < n - 1; i++)
        {
            double px = pts[i].first;
            double py = pts[i].second;

            double t = ((px - sx) * vx + (py - sy) * vy) / v_len2;
            t = std::max(0.0, std::min(1.0, t));

            double projx = sx + t * vx;
            double projy = sy + t * vy;

            double dist = std::hypot(px - projx, py - projy);
            if (dist > max_dist)
            {
                max_dist = dist;
                index = i;
            }
        }

        if (max_dist > eps)
        {
            std::vector<std::pair<double, double>> left(pts.begin(), pts.begin() + index + 1);
            std::vector<std::pair<double, double>> right(pts.begin() + index, pts.end());

            auto res_left = dpSimplify(left, eps);
            auto res_right = dpSimplify(right, eps);

            res_left.pop_back();
            res_left.insert(res_left.end(), res_right.begin(), res_right.end());
            return res_left;
        }
        else
        {
            return { start, end };
        }
    }

    // ============================================================
    // Uniform Cubic B-Spline
    // ============================================================
    std::vector<std::pair<double, double>>
        bsplineUniformCubic(const std::vector<std::pair<double, double>>& ctrl_pts,
            int num_per_seg = 10)
    {
        int n = ctrl_pts.size();
        if (n < 4)
            return ctrl_pts;

        std::vector<std::pair<double, double>> out;
        out.reserve(n * num_per_seg);

        for (int i = 1; i < n - 2; i++)
        {
            auto p0 = ctrl_pts[i - 1];
            auto p1 = ctrl_pts[i];
            auto p2 = ctrl_pts[i + 1];
            auto p3 = ctrl_pts[i + 2];

            for (int j = 0; j < num_per_seg; j++)
            {
                double u = static_cast<double>(j) / num_per_seg;
                double u2 = u * u;
                double u3 = u2 * u;

                double b0 = (-u3 + 3 * u2 - 3 * u + 1) / 6.0;
                double b1 = (3 * u3 - 6 * u2 + 4) / 6.0;
                double b2 = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6.0;
                double b3 = u3 / 6.0;

                double x = b0 * p0.first + b1 * p1.first + b2 * p2.first + b3 * p3.first;
                double y = b0 * p0.second + b1 * p1.second + b2 * p2.second + b3 * p3.second;

                out.emplace_back(x, y);
            }
        }

        out.emplace_back(ctrl_pts[n - 2]);
        out.emplace_back(ctrl_pts[n - 1]);

        return out;
    }

    // ============================================================
    // Path Callback
    // ============================================================
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (generated_)
            return;

        int N = msg->poses.size();
        if (N < 4)
        {
            RCLCPP_WARN(this->get_logger(), "Need at least 4 points for B-spline.");
            return;
        }

        std::vector<std::pair<double, double>> raw_pts;
        raw_pts.reserve(N);

        for (int i = 0; i < N; i++)
        {
            raw_pts.emplace_back(
                msg->poses[i].pose.position.x,
                msg->poses[i].pose.position.y
            );
        }

        // 1) DP simplify
        auto simplified = dpSimplify(raw_pts, eps_);
        RCLCPP_INFO(this->get_logger(),
            "DP simplify: %d -> %d pts", N, (int)simplified.size());

        // 2) B-spline interpolation
        auto bspline_pts = bsplineUniformCubic(simplified, 10);

        nav_msgs::msg::Path raceline;
        raceline.header = msg->header;

        for (auto& p : bspline_pts)
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
        saveCSV(bspline_pts);

        RCLCPP_INFO(this->get_logger(),
            "B-spline raceline generated. points=%ld", bspline_pts.size());
        generated_ = true;
    }

    // ============================================================
    // CSV ����
    // ============================================================
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

    double eps_;
    bool generated_;
    std::string output_csv_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BSplineRacelineNode>());
    rclcpp::shutdown();
    return 0;
}
