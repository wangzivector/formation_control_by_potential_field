#!/usr/bin/env python

import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry


class data_merge:
    def data_merge_node(self):
        rospy.init_node('data_merge_node')

        total_odos = 900
        # estimate
        car0_odom_name = '/car0/vins_estimator/odometry'
        car1_odom_name = '/car1/vins_estimator/odometry'
        car2_odom_name = '/car2/vins_estimator/odometry'
        self.car0_odom = rospy.Subscriber(
            car0_odom_name, Odometry, self.callback_car0)
        self.car1_odom = rospy.Subscriber(
            car1_odom_name, Odometry, self.callback_car1)
        self.car2_odom = rospy.Subscriber(
            car2_odom_name, Odometry, self.callback_car2)

        # ground true
        car0_odom_name_true = '/car0/benchmark_publisher/odometry'
        car1_odom_name_true = '/car1/benchmark_publisher/odometry'
        car2_odom_name_true = '/car2/benchmark_publisher/odometry'
        self.car0_odom_true = rospy.Subscriber(
            car0_odom_name_true, Odometry, self.callback_car0_true)
        self.car1_odom_true = rospy.Subscriber(
            car1_odom_name_true, Odometry, self.callback_car1_true)
        self.car2_odom_true = rospy.Subscriber(
            car2_odom_name_true, Odometry, self.callback_car2_true)

        car0_odom_name_pub = '/car0/odometry'
        car1_odom_name_pub = '/car1/odometry'
        car2_odom_name_pub = '/car2/odometry'
        dist_odom_name_pub = '/dist/odometry'

        self.car0_odom_pub = rospy.Publisher(car0_odom_name_pub, Odometry,queue_size = 1000)
        self.car1_odom_pub = rospy.Publisher(car1_odom_name_pub, Odometry,queue_size = 1000)
        self.car2_odom_pub = rospy.Publisher(car2_odom_name_pub, Odometry,queue_size = 1000)
        self.dist_odom_pub = rospy.Publisher(dist_odom_name_pub, Odometry,queue_size = 1000)

        car0_odom_name_pub_true = '/car0/odometry_true'
        car1_odom_name_pub_true = '/car1/odometry_true'
        car2_odom_name_pub_true = '/car2/odometry_true'

        self.car0_odom_true_pub = rospy.Publisher(car0_odom_name_pub_true, Odometry,queue_size = 1000)
        self.car1_odom_true_pub = rospy.Publisher(car1_odom_name_pub_true, Odometry,queue_size = 1000)
        self.car2_odom_true_pub = rospy.Publisher(car2_odom_name_pub_true, Odometry,queue_size = 1000)

        self.pos_car = [[], [], []]
        self.pos_car_true = [[], [], []]
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()
            print(len(self.pos_car[0]))
            print(len(self.pos_car[1]))
            print(len(self.pos_car[2]))
            print(len(self.pos_car_true[0]))
            print(len(self.pos_car_true[1]))
            print(len(self.pos_car_true[2]))
            if len(self.pos_car[0]) < total_odos:
                continue
            if len(self.pos_car[1]) < total_odos:
                continue
            if len(self.pos_car[2]) < total_odos:
                continue
            if len(self.pos_car_true[0]) < total_odos:
                continue
            if len(self.pos_car_true[1]) < total_odos:
                continue
            if len(self.pos_car_true[2]) < total_odos:
                continue
            # self.do_what_you_need()
            break

        self.distance_UWB = []
        for ind_cc in range(total_odos):
            # original
            header_each = self.pos_car[0][ind_cc].header

            self.pos_car[1][ind_cc].header = header_each
            self.pos_car[2][ind_cc].header = header_each
            
            self.pos_car[0][ind_cc].header = header_each
            self.pos_car_true[1][ind_cc].header = header_each
            self.pos_car_true[2][ind_cc].header = header_each

            # distance
            distance = Odometry()
            distance.header = header_each
            pos_0 = self.pos_car_true[0][ind_cc].pose.pose.position
            pos_1 = self.pos_car_true[1][ind_cc].pose.pose.position
            pos_2 = self.pos_car_true[2][ind_cc].pose.pose.position
            distance.pose.pose.position.x = math.sqrt(
                (pos_1.x - pos_0.x)**2 + (pos_1.y - pos_0.y)**2 + (pos_1.z - pos_0.z)**2)
            distance.pose.pose.position.y = math.sqrt(
                (pos_2.x - pos_0.x)**2 + (pos_2.y - pos_0.y)**2 + (pos_2.z - pos_0.z)**2)
            distance.pose.pose.position.z = math.sqrt(
                (pos_1.x - pos_2.x)**2 + (pos_1.y - pos_2.y)**2 + (pos_1.z - pos_2.z)**2)
            self.distance_UWB.append(distance)

        rate = rospy.Rate(10)
        for ind_cc in range(total_odos):
            rate.sleep()
            self.car0_odom_pub.publish(self.pos_car[0][ind_cc])
            self.car1_odom_pub.publish(self.pos_car[1][ind_cc])
            self.car2_odom_pub.publish(self.pos_car[2][ind_cc])
            self.car0_odom_true_pub.publish(self.pos_car_true[0][ind_cc])
            self.car1_odom_true_pub.publish(self.pos_car_true[1][ind_cc])
            self.car2_odom_true_pub.publish(self.pos_car_true[2][ind_cc])
            self.dist_odom_pub.publish(self.distance_UWB[ind_cc])
            print('pub:', ind_cc)

    def do_what_you_need(self):
        for ind_c in range(3):
            print(ind_c, 'est', self.pos_car[ind_c][0])
            print(ind_c, 'tru', self.pos_car_true[ind_c][0])

    # estimate
    def callback_car0(self, Odometry):
        ind_car = 0
        self.store_car_pose(ind_car, Odometry)

    def callback_car1(self, Odometry):
        ind_car = 1
        self.store_car_pose(ind_car, Odometry)

    def callback_car2(self, Odometry):
        ind_car = 2
        self.store_car_pose(ind_car, Odometry)

    # TRUE
    def callback_car0_true(self, Odometry):
        ind_car = 0
        self.store_car_pose_true(ind_car, Odometry)

    def callback_car1_true(self, Odometry):
        ind_car = 1
        self.store_car_pose_true(ind_car, Odometry)

    def callback_car2_true(self, Odometry):
        ind_car = 2
        self.store_car_pose_true(ind_car, Odometry)

    def store_car_pose(self, index_car, odo):
        self.pos_car[index_car].append(odo)
        # print('receiving car num: ', index_car, len(self.pos_car[index_car]))

    def store_car_pose_true(self, index_car, odo):
        self.pos_car_true[index_car].append(odo)
        # print('receiving true car num: ', index_car, len(self.pos_car_true[index_car]))


if __name__ == '__main__':
    try:
        dm = data_merge()
        dm.data_merge_node()
    except rospy.ROSInterruptException:
        pass


# header:
#   seq: 474
#   stamp:
#     secs: 1403636633
#     nsecs: 463555574
#   frame_id: "world"
# child_frame_id: "world"
# pose:
#   pose:
#     position:
#       x: 2.12993619998
#       y: -1.55004956516
#       z: 0.0833387733647
#     orientation:
#       x: 0.823779338648
#       y: 0.0368069158202
#       z: 0.564827445594
#       w: -0.0315491176086
# twist:
#   twist:
#     linear:
#       x: 0.557061200307
#       y: -0.0857184146256
#       z: -0.16942499729
#     angular:
#       x: 0.0
#       y: 0.0
#       z: 0.0
