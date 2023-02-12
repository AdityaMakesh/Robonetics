#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class LineFollower {
 public:
  LineFollower() {
    laser_sub_ = nh_.subscribe("/scan", 10, &LineFollower::scanCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    int mid_index = scan->ranges.size() / 2;
    float range = scan->ranges[mid_index];
    geometry_msgs::Twist cmd_vel;
    if (range < 0.5) {
      cmd_vel.linear.x = 0.2;
      cmd_vel.angular.z = 0.5;
    } else {
      cmd_vel.linear.x = 0.5;
      cmd_vel.angular.z = 0.0;
    }
    cmd_vel_pub_.publish(cmd_vel);
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_sub_;
  ros::Publisher cmd_vel_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "line_follower");
  LineFollower line_follower;
  ros::spin();
  return 0;
}
