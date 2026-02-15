#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <algorithm>

class CubicSplineRacelineNode : public rclcpp::Node
{
public:
    CubicSplineRacelineNode()
        : Node("cubic_spline_raceline_node"), generated_(false)
    {
        sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/raw_path", 10,
            std::bind(&CubicSplineRacelineNode::pathCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        output_csv_ = "/home/misys/shared_dir/raceline_cubic.csv";
        std::filesystem::create_directories(std::filesystem::path(output_csv_).parent_path());

        RCLCPP_INFO(this->get_logger(), "CubicSplineRacelineNode started");
    }

private:
    // ============================
    // 1D Natural Cubic Spline ����
    // ============================
    void computeSpline(
        const std::vector<double>& t,
        const std::vector<double>& y,
        std::vector<double>& a,
        std::vector<double>& b,
        std::vector<double>& c,
        std::vector<double>& d)
    {
        int n = t.size() - 1;
        std::vector<double> h(n);
        for (int i = 0; i < n; i++)
            h[i] = t[i + 1] - t[i];

        std::vector<double> alpha(n + 1, 0.0);
        for (int i = 1; i < n; i++)
        {
            alpha[i] =
                (3.0 / h[i]) * (y[i + 1] - y[i]) -
                (3.0 / h[i - 1]) * (y[i] - y[i - 1]);
        }

        std::vector<double> l(n + 1, 0.0), mu(n + 1, 0.0), z(n + 1, 0.0);
        l[0] = 1.0;

        for (int i = 1; i < n; i++)
        {
            l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n] = 1.0;
        c.assign(n + 1, 0.0);
        b.assign(n, 0.0);
        d.assign(n, 0.0);

        for (int j = n - 1; j >= 0; j--)
        {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }

        a = y;     // a[j] = y[j]
        a.pop_back();  // ������ ���� �ʿ� ����
        c.pop_back();  // ������ c ����
    }

    // Spline ��
    std::vector<double> evalSpline(
        const std::vector<double>& t,
        const std::vector<double>& a,
        const std::vector<double>& b,
        const std::vector<double>& c,
        const std::vector<double>& d,
        const std::vector<double>& t_new)
    {
        std::vector<double> out(t_new.size());

        int n = a.size();

        for (size_t idx = 0; idx < t_new.size(); idx++)
        {
            double tn = t_new[idx];

            // ���� ã��
            int i = std::upper_bound(t.begin(), t.end(), tn) - t.begin() - 1;
            if (i < 0) i = 0;
            if (i >= n) i = n - 1;

            double dt = tn - t[i];
            out[idx] = a[i] + b[i] * dt + c[i] * dt * dt + d[i] * dt * dt * dt;
        }
        return out;
    }

    // ==================================
    // Path Callback
    // ==================================
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (generated_) return;

        int N = msg->poses.size();
        if (N < 4)
        {
            RCLCPP_WARN(this->get_logger(), "Need at least 4 points for cubic spline.");
            return;
        }

        // ���� ��ǥ
        std::vector<double> xs(N), ys(N);
        for (int i = 0; i < N; i++)
        {
            xs[i] = msg->poses[i].pose.position.x;
            ys[i] = msg->poses[i].pose.position.y;
        }

        // �Ÿ� ��� �Ķ���� s
        std::vector<double> s(N);
        s[0] = 0.0;

        for (int i = 1; i < N; i++)
        {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
        }

        double total_length = s.back();
        double step = 0.05;  // 5cm ���ø�
        int samples = total_length / step + 1;

        std::vector<double> s_new(samples);
        for (int i = 0; i < samples; i++)
            s_new[i] = step * i;

        // Cubic Spline ����
        std::vector<double> ax, bx, cx, dx;
        std::vector<double> ay, by, cy, dy;

        computeSpline(s, xs, ax, bx, cx, dx);
        computeSpline(s, ys, ay, by, cy, dy);

        std::vector<double> xs_new = evalSpline(s, ax, bx, cx, dx, s_new);
        std::vector<double> ys_new = evalSpline(s, ay, by, cy, dy, s_new);

        // Path �޽��� ����
        nav_msgs::msg::Path raceline;
        raceline.header = msg->header;
        raceline.header.frame_id = msg->header.frame_id;

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

        RCLCPP_INFO(this->get_logger(), "Cubic spline raceline generated. points=%ld", xs_new.size());
        generated_ = true;
    }

    // CSV ����
    void saveCSV(const std::vector<double>& xs, const std::vector<double>& ys)
    {
        std::ofstream file(output_csv_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write CSV.");
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
    auto node = std::make_shared<CubicSplineRacelineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
