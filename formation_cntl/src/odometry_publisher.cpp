#include <cstdlib>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <stdio.h>

#define random(x) (rand() % x)

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub0 = n.advertise<nav_msgs::Odometry>("/car0/odom", 50);
  ros::Publisher odom_pub1 = n.advertise<nav_msgs::Odometry>("/car1/odom", 50);
  ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("/car2/odom", 50);
  ros::Publisher odom_pub3 = n.advertise<nav_msgs::Odometry>("/car3/odom", 50);

  ros::Publisher grid_pub =
      n.advertise<nav_msgs::OccupancyGrid>("/gridmap", 10);
  ros::Publisher form_pub =
      n.advertise<sensor_msgs::PointCloud>("/expect_formation", 1);

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
  int index_c = 0;
  ros::Rate r(1);
  while (n.ok()) {
    index_c += 1;
    /*
        -- A --
        this part is for odometry test
        publish three cars odometry for test
    */
    if (index_c % 2 == 0) { // for pub rate control

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
      switch (odo_index % 4) {
      case 0:
        odom.pose.pose.position.x = x + 0.4 * random(5); // just random set
        odom.pose.pose.position.y =
            y + 0.3 * random(8); // x and y have no meaning
        odom_pub0.publish(odom);
        break;
      case 1:
        odom.pose.pose.position.x = x + 0.4 * random(5);
        odom.pose.pose.position.y = y + 0.5 * random(6) + 2;
        odom_pub1.publish(odom);
        break;
      case 2:
        odom.pose.pose.position.x = x + 0.5 * random(4);
        odom.pose.pose.position.y = y + 0.1 * random(3);
        odom_pub2.publish(odom);
        break;
      case 3:
        odom.pose.pose.position.x = x + 0.5 * random(2);
        odom.pose.pose.position.y = y + 0.1 * random(4);
        odom_pub3.publish(odom);
        break;
      }
      std::cout << "publishing odometry info: " << odo_index % 4 << std::endl;
    }
    /*
        -- B --
        this part is for grid map test
        publish gridmap for test
    */

    if (index_c % 15 == 0) { // for pub rate control

      float map_width = 5.5;     // m
      float map_height = 4.5;    // m
      float m_resolution = 0.05; // size of grid

      // left_up position and right_down position of rect obstacle
      int obstacle_num = 2;
      float obstacle_list[obstacle_num][4] = {{1.0, 1.0, 3.0, 1.5},
                                              {2.8, 3, 3.9, 4.4}};
      // int obstacle_num = 1;
      // float obstacle_list[obstacle_num][4] = {{1, 1, 1.2, 1.4}};
      // std::cout << int((obstacle_list[0][1] + 0.5*m_resolution) /
      // m_resolution) << std::endl;

      nav_msgs::OccupancyGrid gridmap;
      gridmap.header.frame_id = "map";
      gridmap.header.stamp = ros::Time::now();
      gridmap.info.resolution = m_resolution; // float32 size of grid
      // actrual map is width*resolution x height*resolution
      gridmap.info.width = int(map_width / m_resolution); // 5.5m x 4.5m
      gridmap.info.height = int(map_height / m_resolution);

      int p[gridmap.info.width * gridmap.info.height] =
          {}; // [0, width x height]

      // obstacle draw in grid map
      for (int ind_ob = 0; ind_ob < obstacle_num; ind_ob++) {
        int rect_xs = int(obstacle_list[ind_ob][0] / m_resolution + 0.5);
        int rect_ys = int(obstacle_list[ind_ob][1] / m_resolution + 0.5);
        int rect_xe = int(obstacle_list[ind_ob][2] / m_resolution + 0.5);
        int rect_ye = int(obstacle_list[ind_ob][3] / m_resolution + 0.5);
        // make sure the orders of x and y index
        if (rect_xs > rect_xe) {
          int temp_x = rect_xe;
          rect_xe = rect_xs;
          rect_xs = temp_x;
        }
        if (rect_ys > rect_ye) {
          int temp_x = rect_ye;
          rect_ye = rect_ys;
          rect_ys = temp_x;
        }

        for (int re_draw_x = rect_xs; re_draw_x <= rect_xe; re_draw_x++) {
          p[re_draw_x + rect_ys * gridmap.info.width] = 100;
          p[re_draw_x + rect_ye * gridmap.info.width] = 100;
          // std::cout << "obstacle info: " << re_draw_x << ","<< rect_ys <<
          // std::endl;
          // std::cout << "obstacle info: " << re_draw_x << ","<< rect_ye <<
          // std::endl;
        }
        for (int re_draw_y = rect_ys; re_draw_y <= rect_ye; re_draw_y++) {
          p[rect_xs + re_draw_y * gridmap.info.width] = 100;
          p[rect_xe + re_draw_y * gridmap.info.width] = 100;
          // std::cout << "obstacle info: " << rect_xs << ","<< re_draw_y <<
          // std::endl;
          // std::cout << "obstacle info: " << rect_xe << ","<< re_draw_y <<
          // std::endl;
        }
      }
      // p[10] = 100; // if is obastcle, set to 100;

      /*
          area remap: provided ind = 10

          area y-dir: [(ind/width)*resolution, (ind/width + 1)*resolution]
          = [0*0.05 1*0.05] = [0 0.05]

          area x-dir: [(ind%width)*resolution, (ind/width + 1)*resolution]
          = [10*0.05 11*0.05] = [0.5 0.55]
      */

      std::vector<signed char> a(
          p,
          p + gridmap.info.width *
                  gridmap.info.height); // remap to vector::char
      gridmap.data = a;
      grid_pub.publish(gridmap);
      std::cout << "publishing gridmap size: " << gridmap.info.width << " x "
                << gridmap.info.height << std::endl;
    }
    /*
        -- C --
        this part is for formation sending test
        publish formation via sensor::PointCloud type for test
    */

    // tra_index = random(10);
    if (index_c % 10 == 0) { // for pub rate control
      sensor_msgs::PointCloud expect_trag;
      expect_trag.header.stamp = ros::Time::now();
      expect_trag.header.frame_id = "formation_what";

      // no points, just use the channels with this pointcloud object
      expect_trag.points.resize(0);

      // four channels for x0 y0 x1 y1
      expect_trag.channels.resize(4);
      expect_trag.channels[0].name = "x0";
      expect_trag.channels[1].name = "y0";
      expect_trag.channels[2].name = "x1";
      expect_trag.channels[3].name = "y1";

      // formation type : formation_x[num_line][x_y_index]
      // here is your difinition of formation
      int num_lines = 4;
      float formation[num_lines][4] = {{1, 2, 4, 2},{1, 2, 1, 4},{1, 4, 4, 4},{4, 4, 4, 2}}; // line
      // float formation[num_lines][4] = {{1, 2, 10, 2}}; // line

      // float formation[3][4] = {
      //     {2, 3, 4, 3}, {2, 3, 3, 1.5}, {3, 1.5, 4, 3}}; // triangle
      // int num_lines = 3;

      for (int line_ind = 0; line_ind < num_lines; line_ind++)
        for (int pos_xy = 0; pos_xy < 4; pos_xy++)
          expect_trag.channels[pos_xy].values.push_back(
              formation[line_ind][pos_xy]);

      form_pub.publish(expect_trag);
      std::cout << "------------------publishing expect_trag size: "
                << expect_trag.channels[0].values.size() << std::endl;
    }
    r.sleep();
    last_time = current_time;
  }
}