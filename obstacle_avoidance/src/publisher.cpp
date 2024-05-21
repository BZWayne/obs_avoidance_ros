#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

void publishFakeLaserData(ros::Publisher& laser_pub, const std::string& frame_id)
{
  sensor_msgs::LaserScan scan;
  scan.header.stamp = ros::Time::now();
  scan.header.frame_id = frame_id;
  scan.angle_min = -1.57; // -90 degrees
  scan.angle_max = 1.57;  // 90 degrees
  scan.angle_increment = 3.14 / 180; // 1 degree
  scan.time_increment = (1 / 10.0) / (2 * M_PI); // assuming 10Hz and 360 readings
  scan.range_min = 0.2;
  scan.range_max = 10.0;

  int num_readings = (scan.angle_max - scan.angle_min) / scan.angle_increment;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);

  // Generate fake data (e.g., a simple sine wave pattern)
  for (int i = 0; i < num_readings; ++i)
  {
    scan.ranges[i] = 1.0 + 0.5 * std::sin(ros::Time::now().toSec() + i * scan.angle_increment);
    scan.intensities[i] = 100; // arbitrary intensity value
  }

  laser_pub.publish(scan);
  ROS_INFO("Published fake laser data on %s", frame_id.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_laser_publishers");
  ros::NodeHandle nh;

  ros::Publisher front_laser_pub = nh.advertise<sensor_msgs::LaserScan>("front_scan", 10);
  ros::Publisher rear_laser_pub = nh.advertise<sensor_msgs::LaserScan>("rear_scan", 10);

  ros::Rate loop_rate(10); // 10 Hz

  while (ros::ok())
  {
    publishFakeLaserData(front_laser_pub, "front_laser_frame");
    publishFakeLaserData(rear_laser_pub, "rear_laser_frame");

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
