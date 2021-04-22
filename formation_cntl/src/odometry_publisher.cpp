#include <stdio.h>
#include <cstdlib>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define random(x) (rand() % x)

int main(int argc, char **argv)
{

    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub0 = n.advertise<nav_msgs::Odometry>("/car0/odom", 50);
    ros::Publisher odom_pub1 = n.advertise<nav_msgs::Odometry>("/car1/odom", 50);
    ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("/car2/odom", 50);
    //   tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    int odo_index = 0;
    ros::Rate r(2);
    while (n.ok())
    {

        ros::spinOnce(); // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        // th += delta_th;

        // //since all odometry is 6DOF we'll need a quaternion created from yaw
        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // //first, we'll publish the transform over tf
        // geometry_msgs::TransformStamped odom_trans;
        // odom_trans.header.stamp = current_time;
        // odom_trans.header.frame_id = "odom";
        // odom_trans.child_frame_id = "base_link";

        // odom_trans.transform.translation.x = x;
        // odom_trans.transform.translation.y = y;
        // odom_trans.transform.translation.z = 0.0;
        // odom_trans.transform.rotation = odom_quat;

        // //send the transform
        // odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        // odom.pose.pose.orientation = odom_quat;

        // //set the velocity
        // odom.child_frame_id = "base_link";
        // odom.twist.twist.linear.x = vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = vth;

        //publish the message
        odo_index = random(10);
        if (odo_index % 3 == 0)
        {
            odom.pose.pose.position.x = x + 0.2 * random(8);
            odom.pose.pose.position.y = y + 0.3 * random(8);
            odom_pub0.publish(odom);
        }
        if (odo_index % 3 == 1)
        {
            odom.pose.pose.position.x = x + 0.4 * random(5);
            odom.pose.pose.position.y = y + 0.5 * random(6) + 2;
            odom_pub1.publish(odom);
        }

        if (odo_index % 3 == 2)
        {
            odom.pose.pose.position.x = x + 0.5 * random(10);
            odom.pose.pose.position.y = y + 0.1 * random(10);
            odom_pub2.publish(odom);
        }

        std::cout << "publishing odometry info: " << odo_index << std::endl;
        last_time = current_time;
        r.sleep();
    }
}