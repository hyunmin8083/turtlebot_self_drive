#include "self_drive.hpp"

SelfDrive::SelfDrive() : rclcpp::Node("self_drive"), step_(0)
{
  auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
  lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", lidar_qos_profile, callback);

  auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
  pose_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", vel_qos_profile);
}

void SelfDrive::subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
 // 센서 데이터 처리
  float front_dist = get_range_avg(scan, 0, 20);         //전방 0도 기준 +-20도
  float left_dist = get_range_avg(scan, 40, 15);         //40도 기준 +-15도
  float right_dist = get_range_avg(scan, 320, 15);       //320도 기준 +-15도

// 주행 명령 결정
  geometry_msgs::msg::TwistStamped vel = decide_movement(front_dist, left_dist, right_dist);
// 로그 출력 및 명령 발행
  RCLCPP_INFO(this->get_logger(),
    "Step: %d | F: %.2f L: %.2f R: %.2f | Lin: %.2f Ang: %.2f",
    step_, front_dist, left_dist, right_dist, vel.twist.linear.x, vel.twist.angular.z);

  pose_pub_->publish(vel);
  step_++;
}

// 특정 각도 범위의 평균 거리를 구하는 헬퍼 함수 
// center_angle: 중심 각도 (Degree), window: 양쪽 범위 (Degree)
float SelfDrive::get_range_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
{
  int size = scan->ranges.size();
  float sum = 0.0;
  int count = 0;

  for (int i = -window; i <= window; i++)
  {
    int idx = (center_angle + i + size) % size;
    float r = scan->ranges[idx];

    if (!std::isinf(r) && !std::isnan(r) && r > 0.01) {
      sum += r;
      count++;
    }
  }

  if (count == 0) return 2.0;
  return sum / count;
}

geometry_msgs::msg::TwistStamped SelfDrive::decide_movement(float front, float left, float right)
{
  geometry_msgs::msg::TwistStamped vel;
  vel.header.stamp = this->now();
  vel.header.frame_id = "base_link";

  vel.twist.linear.x = TARGET_SPEED;

  if (front < SAFE_DISTANCE)
  {
    if (left > right) {
      vel.twist.angular.z = TARGET_ANGULAR;   //( = 1.3 )
    } else {
      vel.twist.angular.z = -TARGET_ANGULAR;  //( = -1.3 )
    }
  }
  else
  {
    float error = left - right;
    error = std::clamp(error, -1.0f, 1.0f);    //clamp -1 ~ 1
    vel.twist.angular.z = error * KP_ANGULAR;
  }

  return vel;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

