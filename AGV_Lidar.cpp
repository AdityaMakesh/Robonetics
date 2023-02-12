#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class AGV {
  public:
    AGV();

  private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber scan_sub;
};

AGV::AGV() {
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, &AGV::scanCallback, this);
}

void AGV::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  geometry_msgs::Twist cmd_vel;

  // Find the closest obstacle in front of the AGV
  int min_index = -1;
  float min_distance = std::numeric_limits<float>::infinity();
  for (int i = 0; i < scan->ranges.size(); i++) {
    float angle = scan->angle_min + i * scan->angle_increment;
    if (angle > -0.5 * M_PI && angle < 0.5 * M_PI && scan->ranges[i] < min_distance) {
      min_index = i;
      min_distance = scan->ranges[i];
    }
  }

  // If an obstacle is found, adjust the AGV's trajectory
  if (min_index != -1) {
    float angle = scan->angle_min + min_index * scan->angle_increment;
    cmd_vel.angular.z = angle;
  } else {
    cmd_vel.linear.x = 0.5;
  }

  cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "agv_node");
  AGV agv;
  ros::spin();
  return 0;
}
