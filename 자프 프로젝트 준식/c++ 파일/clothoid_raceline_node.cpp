#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <string>
#include <algorithm>

class ClothoidRacelineNode : public rclcpp::Node
{
public:
    ClothoidRacelineNode()
        : Node("clothoid_raceline_node"), generated_(false)
    {
        sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/raw_path", 10,
            std::bind(&ClothoidRacelineNode::pathCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Path>("/raceline_clothoid", 10);

        output_csv_ = "/home/misys/shared_dir/raceline_clothoid.csv";
        std::filesystem::create_directories(std::filesystem::path(output_csv_).parent_path());

        RCLCPP_INFO(this->get_logger(), "ClothoidRacelineNode started");
    }

private:
    // heading 추정
    std::vector<double> estimateHeading(const std::vector<double>& xs,
        const std::vector<double>& ys)
    {
        size_t n = xs.size();
        std::vector<double> headings(n, 0.0);

        if (n < 2) return headings;

        for (size_t i = 0; i < n; i++)
        {
            double dx, dy;
            if (i == 0)
            {
                dx = xs[1] - xs[0];
                dy = ys[1] - ys[0];
            }
            else if (i == n - 1)
            {
                dx = xs[n - 1] - xs[n - 2];
                dy = ys[n - 1] - ys[n - 2];
            }
            else
            {
                dx = xs[i + 1] - xs[i - 1];
                dy = ys[i + 1] - ys[i - 1];
            }
            headings[i] = std::atan2(dy, dx);
        }

        // np.unwrap 과 비슷하게 angle unwrap
        std::vector<double> unwrapped(n, 0.0);
        unwrapped[0] = headings[0];
        double two_pi = 2.0 * M_PI;

        for (size_t i = 1; i < n; i++)
        {
            double diff = headings[i] - headings[i - 1];
            while (diff > M_PI)  diff -= two_pi;
            while (diff < -M_PI) diff += two_pi;
            unwrapped[i] = unwrapped[i - 1] + diff;
        }
        return unwrapped;
    }

    // curvature 추정
    std::vector<double> estimateCurvature(const std::vector<double>& xs,
        const std::vector<double>& ys,
        const std::vector<double>& headings)
    {
        size_t n = xs.size();
        std::vector<double> curv(n, 0.0);
        if (n < 2) return curv;

        curv[0] = 0.0;
        for (size_t i = 1; i < n; i++)
        {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            double ds = std::hypot(dx, dy);
            if (ds < 1e-6) ds = 1e-6;

            double dtheta = headings[i] - headings[i - 1];
            curv[i] = dtheta / ds;
        }
        return curv;
    }

    // 1D 선형보간 (np.interp 비슷)
    std::vector<double> interp1d(const std::vector<double>& x,
        const std::vector<double>& y,
        const std::vector<double>& x_new)
    {
        std::vector<double> out(x_new.size(), 0.0);
        if (x.size() < 2) return out;

        size_t n = x.size();
        size_t j = 0;

        for (size_t i = 0; i < x_new.size(); i++)
        {
            double xn = x_new[i];

            if (xn <= x.front())
            {
                out[i] = y.front();
                continue;
            }
            if (xn >= x.back())
            {
                out[i] = y.back();
                continue;
            }

            while (j + 1 < n && x[j + 1] < xn)
                j++;

            double x0 = x[j];
            double x1 = x[j + 1];
            double y0 = y[j];
            double y1 = y[j + 1];

            double t = (xn - x0) / (x1 - x0);
            out[i] = y0 + t * (y1 - y0);
        }

        return out;
    }

    // clothoid-like path 생성
    void generateClothoidLike(const std::vector<double>& xs,
        const std::vector<double>& ys,
        std::vector<double>& xs_out,
        std::vector<double>& ys_out,
        double step = 0.05)
    {
        size_t n = xs.size();
        if (n < 2) return;

        // arc-length s
        std::vector<double> s(n, 0.0);
        for (size_t i = 1; i < n; i++)
        {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
        }
        double total_len = s.back();

        // heading / curvature
        auto headings = estimateHeading(xs, ys);
        auto curv = estimateCurvature(xs, ys, headings);

        // s_new 샘플링
        int num_samples = static_cast<int>(total_len / step) + 1;
        std::vector<double> s_new(num_samples, 0.0);
        for (int i = 0; i < num_samples; i++)
            s_new[i] = step * i;

        // 곡률 보간
        auto curv_interp = interp1d(s, curv, s_new);
        // head_interp = np.interp(s_new, s, headings) 이 있었지만 실제 적분엔 사용 안 함

        // clothoid-like 적분
        double x0 = xs[0];
        double y0 = ys[0];
        double theta = headings[0];

        xs_out.clear();
        ys_out.clear();
        xs_out.reserve(num_samples);
        ys_out.reserve(num_samples);

        xs_out.push_back(x0);
        ys_out.push_back(y0);

        for (int i = 1; i < num_samples; i++)
        {
            double ds_i = s_new[i] - s_new[i - 1];
            double k = curv_interp[i];

            theta += k * ds_i;

            double x_new = xs_out.back() + ds_i * std::cos(theta);
            double y_new = ys_out.back() + ds_i * std::sin(theta);

            xs_out.push_back(x_new);
            ys_out.push_back(y_new);
        }
    }

    // Path 콜백
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (generated_)
            return;

        size_t N = msg->poses.size();
        if (N < 4)
        {
            RCLCPP_WARN(this->get_logger(), "Need at least 4 points for clothoid-like raceline.");
            return;
        }

        std::vector<double> xs(N), ys(N);
        for (size_t i = 0; i < N; i++)
        {
            xs[i] = msg->poses[i].pose.position.x;
            ys[i] = msg->poses[i].pose.position.y;
        }

        std::vector<double> xs_new, ys_new;
        generateClothoidLike(xs, ys, xs_new, ys_new, 0.05);

        nav_msgs::msg::Path raceline;
        raceline.header.frame_id = msg->header.frame_id;
        raceline.header.stamp = msg->header.stamp;

        for (size_t i = 0; i < xs_new.size(); i++)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = raceline.header;
            ps.pose.position.x = xs_new[i];
            ps.pose.position.y = ys_new[i];
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            raceline.poses.push_back(ps);
        }

        pub_->publish(raceline);
        saveCSV(xs_new, ys_new);

        RCLCPP_INFO(this->get_logger(), "Clothoid-like raceline generated. points=%ld", xs_new.size());
        generated_ = true;
    }

    void saveCSV(const std::vector<double>& xs, const std::vector<double>& ys)
    {
        std::ofstream file(output_csv_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write CSV");
            return;
        }

        file << "x,y\n";
        for (size_t i = 0; i < xs.size(); i++)
            file << xs[i] << "," << ys[i] << "\n";

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
    auto node = std::make_shared<ClothoidRacelineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
