#include <cstdlib>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>

#define random(x) (rand() % x)

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub0 = n.advertise<nav_msgs::Odometry>("/car0/odom", 50);
  ros::Publisher odom_pub1 = n.advertise<nav_msgs::Odometry>("/car1/odom", 50);
  ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("/car2/odom", 50);
  //   tf::TransformBroadcaster odom_broadcaster;

  ros::Publisher grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);

  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double th = 0.0;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  int odo_index = 0;
  ros::Rate r(2);
  while (n.ok()) {
    /*
    -- A --
    this part is for odometry test
    publish three cars odometry for test
    */
    ros::spinOnce(); // check for incoming messages
    current_time = ros::Time::now();

    // compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    // publish the message
    odo_index = random(10);
    switch (odo_index % 3) {
    case 0:
      odom.pose.pose.position.x = x + 0.2 * random(8);
      odom.pose.pose.position.y = y + 0.3 * random(8);
      odom_pub0.publish(odom);
      break;
    case 1:
      odom.pose.pose.position.x = x + 0.4 * random(5);
      odom.pose.pose.position.y = y + 0.5 * random(6) + 2;
      odom_pub1.publish(odom);
      break;
    case 2:
      odom.pose.pose.position.x = x + 0.5 * random(10);
      odom.pose.pose.position.y = y + 0.1 * random(10);
      odom_pub2.publish(odom);
      break;
    }
    /*
    -- B --
    this part is for grid map test
    publish gridmap for test
    */
    nav_msgs::OccupancyGrid gridmap;
    gridmap.header.frame_id = "grid";
    gridmap.header.stamp = ros::Time::now();
    gridmap.info.resolution = 0.5; // float32 size of grid
    gridmap.info.width = 20;       // actrual map is width*resolution
    gridmap.info.height = 20;

    int p[gridmap.info.width * gridmap.info.height] = {-1}; // [0,100]
    p[10] = 100;
    std::vector<signed char> a(p, p + 400);
    gridmap.data = a;
    grid_pub.publish(gridmap);

    std::cout << "publishing odometry info: " << odo_index << std::endl;
    last_time = current_time;
    r.sleep();
    
  }
}