// speed_profiler_node.cpp

#include <memory>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

class SpeedProfiler : public rclcpp::Node
{
public:
  SpeedProfiler()
  : Node("speed_profiler"),
    has_path_(false),
    has_curvature_(false)
  {
    // ----- 파라미터 선언 -----
    // 최대 lateral 가속도 [m/s^2]
    a_lat_max_ = this->declare_parameter("a_lat_max", 6.0);         // 필요에 따라 조정
    // 최대 종방향 가속도(가속) [m/s^2]
    a_lon_accel_max_ = this->declare_parameter("a_lon_accel_max", 3.0);
    // 최대 종방향 감속(제동) [m/s^2] - 양수로 넣고 내부에서 사용
    a_lon_decel_max_ = this->declare_parameter("a_lon_decel_max", 5.0);
    // 최고 속도 [m/s]
    v_max_ = this->declare_parameter("v_max", 12.0);                // 예: 약 43km/h
    epsilon_kappa_ = this->declare_parameter("epsilon_kappa", 1e-6);

    // ----- Subscribers -----
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path", 10,
      std::bind(&SpeedProfiler::pathCallback, this, std::placeholders::_1));

    curvature_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/curvature_profile", 10,
      std::bind(&SpeedProfiler::curvatureCallback, this, std::placeholders::_1));

    sector_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/sector_boundaries", 10,
      std::bind(&SpeedProfiler::sectorCallback, this, std::placeholders::_1));

    // 현재 차량 위치(옵션) - 토픽 이름은 프로젝트에 맞게 바꿔도 됨
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/current_pose", 10,
      std::bind(&SpeedProfiler::poseCallback, this, std::placeholders::_1));

    // ----- Publishers -----
    speed_profile_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/target_speed_profile", 10);
    target_speed_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/target_speed", 10);

    RCLCPP_INFO(this->get_logger(), "SpeedProfiler node initialized.");
  }

private:
  // ========== 콜백들 ==========

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Received global_path with too few points.");
      return;
    }

    global_path_ = *msg;
    has_path_ = true;

    // 거리(아크 길이) 전처리
    computeArcLengths();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received global_path with %zu points.", global_path_.poses.size());

    tryComputeSpeedProfile();
  }

  void curvatureCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    curvature_profile_ = msg->data;
    has_curvature_ = true;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received curvature_profile with %zu elements.", curvature_profile_.size());

    tryComputeSpeedProfile();
  }

  void sectorCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    sector_boundaries_ = msg->data;
    // 지금은 섹터 정보는 저장만 하고, 향후 속도 제한 디테일에 쓸 수 있음
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received sector_boundaries with %zu indices.", sector_boundaries_.size());
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
    if (speed_profile_.empty() || arc_lengths_.empty()) {
      return;
    }

    // 가장 가까운 path index 찾기
    const int idx = findNearestIndex(current_pose_);
    if (idx < 0 || static_cast<size_t>(idx) >= speed_profile_.size()) {
      return;
    }

    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = speed_profile_[idx];
    target_speed_pub_->publish(speed_msg);
  }

  // ========== 핵심 로직 ==========

  void tryComputeSpeedProfile()
  {
    if (!has_path_ || !has_curvature_) {
      // 둘 다 들어오기 전까지는 계산 X
      return;
    }

    const size_t n_path = global_path_.poses.size();
    const size_t n_kappa = curvature_profile_.size();

    if (n_path != n_kappa) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Size mismatch: path(%zu) vs curvature(%zu). Skipping speed profile computation.",
        n_path, n_kappa);
      return;
    }

    if (arc_lengths_.size() != n_path) {
      RCLCPP_WARN(
        this->get_logger(), "Arc length size(%zu) mismatch with path(%zu).",
        arc_lengths_.size(), n_path);
      return;
    }

    computeSpeedProfile();
    publishSpeedProfile();
  }

  void computeArcLengths()
  {
    const size_t n = global_path_.poses.size();
    arc_lengths_.assign(n, 0.0);

    for (size_t i = 1; i < n; ++i) {
      const auto & p_prev = global_path_.poses[i - 1].pose.position;
      const auto & p_curr = global_path_.poses[i].pose.position;

      const double dx = p_curr.x - p_prev.x;
      const double dy = p_curr.y - p_prev.y;
      const double ds = std::sqrt(dx * dx + dy * dy);

      arc_lengths_[i] = arc_lengths_[i - 1] + ds;
    }
  }

  void computeSpeedProfile()
  {
    const size_t n = global_path_.poses.size();
    if (n == 0) {
      return;
    }

    // --- 1. 곡률 기반 최대 속도 v_kappa 계산 ---
    std::vector<double> v_kappa(n, v_max_);

    for (size_t i = 0; i < n; ++i) {
      const double k = curvature_profile_[i];
      const double abs_k = std::abs(k);

      if (abs_k < 1e-9) {
        // 거의 직선: v_max로 설정
        v_kappa[i] = v_max_;
      } else {
        const double v_lim = std::sqrt(a_lat_max_ / (abs_k + epsilon_kappa_));
        v_kappa[i] = std::min(v_lim, v_max_);
      }
    }

    // --- 2. Forward pass (가속 제한) ---
    std::vector<double> v_forward(n, 0.0);
    v_forward[0] = v_kappa[0];  // 출발 속도(필요시 0으로 수정 가능)

    for (size_t i = 1; i < n; ++i) {
      const double ds = arc_lengths_[i] - arc_lengths_[i - 1];
      const double v_prev = v_forward[i - 1];

      // v^2 = v_prev^2 + 2 * a * ds
      const double v_max_accel = std::sqrt(
        std::max(0.0, v_prev * v_prev + 2.0 * a_lon_accel_max_ * ds));

      v_forward[i] = std::min(v_kappa[i], v_max_accel);
    }

    // --- 3. Backward pass (감속 제한) ---
    std::vector<double> v_backward(n, 0.0);
    v_backward[n - 1] = v_forward[n - 1];  // 마지막 점 속도 그대로

    for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
      const double ds = arc_lengths_[i + 1] - arc_lengths_[i];
      const double v_next = v_backward[i + 1];

      const double v_max_brake = std::sqrt(
        std::max(0.0, v_next * v_next + 2.0 * a_lon_decel_max_ * ds));

      v_backward[i] = std::min(v_forward[i], v_max_brake);
    }

    speed_profile_ = v_backward;

    RCLCPP_INFO(
      this->get_logger(),
      "Computed speed profile for %zu points. v_min=%.2f, v_max=%.2f [m/s]",
      speed_profile_.size(),
      *std::min_element(speed_profile_.begin(), speed_profile_.end()),
      *std::max_element(speed_profile_.begin(), speed_profile_.end()));
  }

  void publishSpeedProfile()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = speed_profile_;
    speed_profile_pub_->publish(msg);
  }

  int findNearestIndex(const geometry_msgs::msg::PoseStamped & pose) const
  {
    if (global_path_.poses.empty()) {
      return -1;
    }

    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;

    double best_dist2 = std::numeric_limits<double>::max();
    int best_idx = -1;

    for (size_t i = 0; i < global_path_.poses.size(); ++i) {
      const auto & p = global_path_.poses[i].pose.position;
      const double dx = p.x - x;
      const double dy = p.y - y;
      const double d2 = dx * dx + dy * dy;

      if (d2 < best_dist2) {
        best_dist2 = d2;
        best_idx = static_cast<int>(i);
      }
    }

    return best_idx;
  }

  // ========== 멤버 변수들 ==========

  // 입력 데이터
  nav_msgs::msg::Path global_path_;
  std::vector<double> curvature_profile_;
  std::vector<int32_t> sector_boundaries_;  // 현재는 저장만

  geometry_msgs::msg::PoseStamped current_pose_;

  // 전처리: 각 점까지의 아크 길이(누적 거리)
  std::vector<double> arc_lengths_;

  // 최종 속도 프로파일
  std::vector<double> speed_profile_;

  // 구독자/퍼블리셔
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr curvature_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sector_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr speed_profile_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_speed_pub_;

  // 플래그
  bool has_path_;
  bool has_curvature_;

  // 파라미터들
  double a_lat_max_;
  double a_lon_accel_max_;
  double a_lon_decel_max_;
  double v_max_;
  double epsilon_kappa_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeedProfiler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
