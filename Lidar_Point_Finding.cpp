#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

// Callback function for the lidar data
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg, tf::TransformListener& listener)
{
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    float angle = msg->angle_min + i * msg->angle_increment;
    float range = msg->ranges[i];

    if (range < msg->range_min || range > msg->range_max)
    {
      continue;
    }

    // Convert the polar coordinates to Cartesian coordinates
    float x = range * cos(angle);
    float y = range * sin(angle);

    // Transform the point to the map frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_laser";
    point.header.stamp = ros::Time();
    point.point.x = x;
    point.point.y = y;
    point.point.z = 0.0;

    try
    {
      listener.transformPoint("map", point, point);
      // Do something with the point in the map frame
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "lidar_point_finding");
  ros::NodeHandle nh;

  // Subscribe to the lidar data
  ros::Subscriber laser_sub = nh.subscribe("scan", 10, boost::bind(laserCallback, _1, boost::ref(listener)));

  // Initialize the transform listener
  tf::TransformListener listener;

  // Spin the node
  ros::spin();

  return 0;
}
