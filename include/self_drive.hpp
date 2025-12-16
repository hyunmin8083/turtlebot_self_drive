#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_pub_;
  int step_;

  const float TARGET_SPEED = 0.22;
  const float SAFE_DISTANCE = 0.55;
  const float ROBOT_WIDTH = 0.28;
  const float CORRIDOR_WIDTH = 0.4;
  const float KP_ANGULAR = 2.0;   // sensitivity (P제어 상수)

public:
  SelfDrive();

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

private:
  float get_range_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window);

  geometry_msgs::msg::TwistStamped decide_movement(float front, float left, float right);
};
