#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ydlidar/ydlidar.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ydlidar_node");
  ros::NodeHandle nh;
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);

  YDLidar laser;
  if (!laser.init("/dev/ydlidar", 128000)) {
    ROS_ERROR("Failed to initialize YDLidar");
    return -1;
  }

  while (ros::ok()) {
    laser_scan scan;
    if (laser.doProcessSimple(scan)) {
      sensor_msgs::LaserScan msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "laser_frame";
      msg.angle_min = scan.config.min_angle;
      msg.angle_max = scan.config.max_angle;
      msg.angle_increment = scan.config.ang_increment;
      msg.time_increment = scan.config.time_increment;
      msg.scan_time = scan.config.scan_time;
      msg.range_min = scan.config.min_range;
      msg.range_max = scan.config.max_range;
      msg.ranges.resize(scan.ranges.size());
      msg.intensities.resize(scan.ranges.size());
      for (size_t i = 0; i < scan.ranges.size(); i++) {
        msg.ranges[i] = scan.ranges[i];
        msg.intensities[i] = scan.intensities[i];
      }
      laser_pub.publish(msg);
    }
    ros::spinOnce();
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
