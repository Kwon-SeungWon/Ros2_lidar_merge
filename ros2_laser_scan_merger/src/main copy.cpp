#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

class ScanMerger : public rclcpp::Node
{
public:
  ScanMerger() : Node("ros2_laser_scan_merger"),
                 new_data_available_(false),
                 worker_shutdown_(false)
  {
    initialize_params();
    refresh_params();

    laser1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic1_);
    laser2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic2_);
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *laser1_sub_, *laser2_sub_);
    sync_->registerCallback(std::bind(&ScanMerger::synchronized_callback, this,
                                        std::placeholders::_1, std::placeholders::_2));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS().best_effort());

    // 워커 스레드 시작: 새로운 데이터가 도착하면 update_point_cloud_rgb()를 호출
    worker_thread_ = std::thread(&ScanMerger::workerLoop, this);
  }

  ~ScanMerger()
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      worker_shutdown_ = true;
      worker_cv_.notify_one();
    }
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  // Message Filter 관련 멤버
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser1_sub_, laser2_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  // laser 데이터 보호를 위한 뮤텍스
  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan laser1_;
  sensor_msgs::msg::LaserScan laser2_;

  // 워커 스레드 및 동기화 변수
  std::thread worker_thread_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  bool new_data_available_;
  bool worker_shutdown_;

  // 파라미터 변수
  std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_;
  float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  uint8_t laser1R_, laser1G_, laser1B_;
  float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  uint8_t laser2R_, laser2G_, laser2B_;

  // 콜백: 동기화된 laser 메시지가 도착하면 데이터 업데이트 후 워커에게 알림
  void synchronized_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg,
                               const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      laser1_ = *laser1_msg;
      laser2_ = *laser2_msg;
    }
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      new_data_available_ = true;
      worker_cv_.notify_one();
    }
  }

  // 워커 스레드 루프: 새로운 데이터가 도착하면 update_point_cloud_rgb() 실행
  void workerLoop()
  {
    while (true) {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_cv_.wait(lock, [this] { return new_data_available_ || worker_shutdown_; });
      if (worker_shutdown_) {
        break;
      }
      new_data_available_ = false;
      lock.unlock();

      update_point_cloud_rgb();
    }
  }

  // PointCloud 갱신 처리
  void update_point_cloud_rgb()
  {
    // laser 메시지들을 안전하게 복사
    sensor_msgs::msg::LaserScan local_laser1, local_laser2;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      local_laser1 = laser1_;
      local_laser2 = laser2_;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<std::array<float, 2>> scan_data;
    float min_theta = std::numeric_limits<float>::max();
    float max_theta = std::numeric_limits<float>::lowest();

    auto process_laser = [&](const sensor_msgs::msg::LaserScan &laser, 
                             float x_off, float y_off, float z_off, float alpha, 
                             float angle_min_deg, float angle_max_deg, 
                             bool flip, bool inverse, bool show) {
      if (!show || laser.ranges.empty()) return;

      float temp_min = std::min(laser.angle_min, laser.angle_max);
      float temp_max = std::max(laser.angle_min, laser.angle_max);
      float alpha_rad = alpha * M_PI / 180.0;
      float cos_alpha = std::cos(alpha_rad);
      float sin_alpha = std::sin(alpha_rad);
      float angle_min_rad = angle_min_deg * M_PI / 180.0;
      float angle_max_rad = angle_max_deg * M_PI / 180.0;

      for (size_t i = 0; i < laser.ranges.size(); ++i) {
        float angle = temp_min + i * laser.angle_increment;
        if (angle > temp_max) break;

        size_t idx = flip ? laser.ranges.size() - 1 - i : i;
        float range = laser.ranges[idx];

        if (std::isnan(range) || range < laser.range_min || range > laser.range_max) continue;

        bool is_in_range = (angle >= angle_min_rad && angle <= angle_max_rad);
        if (inverse == is_in_range) continue;

        pcl::PointXYZ pt; 
        float x = range * std::cos(angle);
        float y = range * std::sin(angle);

        pt.x = x * cos_alpha - y * sin_alpha + x_off;
        pt.y = x * sin_alpha + y * cos_alpha + y_off;
        pt.z = z_off;

        cloud.points.push_back(pt);

        float r_ = std::hypot(pt.x, pt.y);
        float theta_ = std::atan2(pt.y, pt.x);
        scan_data.push_back({theta_, r_});

        min_theta = std::min(min_theta, theta_);
        max_theta = std::max(max_theta, theta_);
      }
    };

    // 두 레이저 스캔 처리
    process_laser(local_laser1, laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_,
                  laser1AngleMin_, laser1AngleMax_, flip1_, inverse1_, show1_);
    process_laser(local_laser2, laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_,
                  laser2AngleMin_, laser2AngleMax_, flip2_, inverse2_, show2_);

    // 가까운 점 제거
    removePointsWithinRadius(cloud, 0.2);
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *pc2_msg);
    pc2_msg->header.frame_id = cloudFrameId_;

    // 두 laser 중 늦은 시간 사용
    rclcpp::Time time1(local_laser1.header.stamp);
    rclcpp::Time time2(local_laser2.header.stamp);
    pc2_msg->header.stamp = (time1 < time2) ? local_laser2.header.stamp : local_laser1.header.stamp;
    pc2_msg->is_dense = false;

    point_cloud_pub_->publish(*pc2_msg);
  }

  float GET_R(float x, float y)
  {
    return std::sqrt(x * x + y * y);
  }

  float GET_THETA(float x, float y)
  {
    float temp_res;
    if (x != 0) {
      temp_res = std::atan(y / x);
    } else {
      temp_res = (y >= 0) ? M_PI / 2 : -M_PI / 2;
    }
    if (temp_res > 0 && y < 0) {
      temp_res -= M_PI;
    } else if (temp_res < 0 && x < 0) {
      temp_res += M_PI;
    }
    return temp_res;
  }

  float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle)
  {
    return (magnitude_1 + current_angle * ((magnitude_2 - magnitude_1) / (angle_2 - angle_1)));
  }

  void initialize_params()
  {
    this->declare_parameter("pointCloudTopic", "base/custom_cloud");
    this->declare_parameter("pointCloutFrameId", "laser");  // 오타 주의: 나중에 "pointCloudFrameId"로 수정 권장

    this->declare_parameter("scanTopic1", "lidar_front_right/scan");
    this->declare_parameter("laser1XOff", -0.45);
    this->declare_parameter("laser1YOff", 0.24);
    this->declare_parameter("laser1ZOff", 0.0);
    this->declare_parameter("laser1Alpha", 45.0);
    this->declare_parameter("laser1AngleMin", -181.0);
    this->declare_parameter("laser1AngleMax", 181.0);
    this->declare_parameter("laser1R", 255);
    this->declare_parameter("laser1G", 0);
    this->declare_parameter("laser1B", 0);
    this->declare_parameter("show1", true);
    this->declare_parameter("flip1", false);
    this->declare_parameter("inverse1", false);

    this->declare_parameter("scanTopic2", "lidar_rear_left/scan");
    this->declare_parameter("laser2XOff", 0.315);
    this->declare_parameter("laser2YOff", -0.24);
    this->declare_parameter("laser2ZOff", 0.0);
    this->declare_parameter("laser2Alpha", 225.0);
    this->declare_parameter("laser2AngleMin", -181.0);
    this->declare_parameter("laser2AngleMax", 181.0);
    this->declare_parameter("laser2R", 0);
    this->declare_parameter("laser2G", 0);
    this->declare_parameter("laser2B", 255);
    this->declare_parameter("show2", true);
    this->declare_parameter("flip2", false);
    this->declare_parameter("inverse2", false);
  }

  void refresh_params()
  {
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "pointCloud");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "laser");
    this->get_parameter_or<std::string>("scanTopic1", topic1_, "lidar_front_right/scan");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.0);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 0.0);
    this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, -181.0);
    this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser1R", laser1R_, 0);
    this->get_parameter_or<uint8_t>("laser1G", laser1G_, 0);
    this->get_parameter_or<uint8_t>("laser1B", laser1B_, 0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<bool>("flip1", flip1_, false);
    this->get_parameter_or<bool>("inverse1", inverse1_, false);
    this->get_parameter_or<std::string>("scanTopic2", topic2_, "lidar_rear_left/scan");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, 0.0);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, -181.0);
    this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser2R", laser2R_, 0);
    this->get_parameter_or<uint8_t>("laser2G", laser2G_, 0);
    this->get_parameter_or<uint8_t>("laser2B", laser2B_, 0);
    this->get_parameter_or<bool>("show2", show2_, false);
    this->get_parameter_or<bool>("flip2", flip2_, false);
    this->get_parameter_or<bool>("inverse2", inverse2_, false);
  }

  void removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ>& cloud, float radius, float center_x = 0.0f, float center_y = 0.0f) {
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;
    for (const auto& point : cloud.points) {
      float distance = std::sqrt((point.x - center_x) * (point.x - center_x) +
                                 (point.y - center_y) * (point.y - center_y));
      if (distance >= radius) {
        filteredCloud.points.push_back(point);
      }
    }
    cloud.points.swap(filteredCloud.points);
  }

  // 중복된 변수 선언 제거 (이미 상단에 선언됨)
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMerger>());
  rclcpp::shutdown();
  return 0;
}
