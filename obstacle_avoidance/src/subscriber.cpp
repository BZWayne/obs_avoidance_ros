#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_pub;

void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ROS_INFO("Front scan callback triggered.");
  geometry_msgs::Twist cmd;
  bool obstacle_detected = false;

  for (const auto& range : scan->ranges) {
    if (range < 1.0) {  // Threshold for obstacle detection
      obstacle_detected = true;
      break;
    }
  }

  if (obstacle_detected) {
    ROS_WARN("Obstacle detected in front.");
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.5;  // Turn to avoid obstacle
  } else {
    cmd.linear.x = 0.5;   // Move forward
    cmd.angular.z = 0.0;
  }

  cmd_pub.publish(cmd);
}

void rearScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ROS_INFO("Rear scan callback triggered.");
  geometry_msgs::Twist cmd;
  bool obstacle_detected = false;

  for (const auto& range : scan->ranges) {
    if (range < 1.0) {  // Threshold for obstacle detection
      obstacle_detected = true;
      break;
    }
  }

  if (obstacle_detected) {
    ROS_WARN("Obstacle detected in rear.");
    cmd.linear.x = 0.0;
    cmd.angular.z = -0.5;  // Turn to avoid obstacle
  } else {
    cmd.linear.x = 0.5;   // Move forward
    cmd.angular.z = 0.0;
  }

  cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_sensor_obstacle_avoidance_node");
  ros::NodeHandle nh;

  cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber front_sub = nh.subscribe("front_scan", 10, frontScanCallback);
  ros::Subscriber rear_sub = nh.subscribe("rear_scan", 10, rearScanCallback);

  ROS_INFO("Multi-sensor obstacle avoidance node started.");

  ros::spin();
  return 0;
}
