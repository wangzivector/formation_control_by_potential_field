#!/usr/bin/env python

# 1. ros grid map to set the boundary line and obstacles and then avoid them
# 2.<done> setup interface with cpp in sending gridmap and formation
# 3.<done> receive one formation via interface and give calculation result

import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from numpy.lib.function_base import _interp_dispatcher
import yaml

import rospy
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud


class Formation_cars:
    def formation_node(self):
        yaml_path = sys.path[0] + '/../yaml/formation_param.yaml'
        print(yaml_path)
        with open(yaml_path, 'r') as f:
            yaml_params = yaml.load(f.read())
            print(yaml_params)
        self.num_car = yaml_params['num_car']

        self.update_car = np.tile(False, [self.num_car])
        self.pos_car = np.zeros([self.num_car, 2])
        self.update_exp_trag = False
        rospy.init_node('formation_node')

        self.fc = formation(yaml_params)
        print('created formation object, initial pose of car:')
        print(self.fc.pos)
        path_name = yaml_params['path_name']
        self.PathPublisher = rospy.Publisher(
            path_name, Path, queue_size=5)  # trag

        # subscriber of position of cars
        car0_odom_name = yaml_params['car0_odom_name']
        car1_odom_name = yaml_params['car1_odom_name']
        car2_odom_name = yaml_params['car2_odom_name']
        expect_formation_name = yaml_params['expect_formation_name']
        gridmap_name = yaml_params['gridmap_name']
        self.car0_odom = rospy.Subscriber(
            car0_odom_name, Odometry, self.callback_car0)
        self.car1_odom = rospy.Subscriber(
            car1_odom_name, Odometry, self.callback_car1)
        self.car2_odom = rospy.Subscriber(
            car2_odom_name, Odometry, self.callback_car2)
        self.expect_formation = rospy.Subscriber(
            expect_formation_name, PointCloud, self.callback_expect_formation)
        self.gridmap_get = rospy.Subscriber(
            gridmap_name, OccupancyGrid, self.callback_gridmap)

        # close loop repeating
        rate = rospy.Rate(5)  # several hz
        while not rospy.is_shutdown():

            # if self.check_update():
            #     hello_str = "check success, formation node running ... %s" % rospy.get_time()
            #     rospy.loginfo(hello_str)
            #     print(self.pos_car)
            #     for ind_c in range(0, self.num_car):
            #         fc.pos[ind_c, 0] = self.pos_car[ind_c, 0]
            #         fc.pos[ind_c, 1] = self.pos_car[ind_c, 1]

            if self.update_exp_trag:
                print("got trag!")
                self.fc.shape_in = self.exp_formation
                print(self.fc.shape_in)

                for ind_c in range(0, self.num_car):
                    self.fc.pos[ind_c, 0] = self.pos_car[ind_c, 0]
                    self.fc.pos[ind_c, 1] = self.pos_car[ind_c, 1]
                print(self.fc.pos)

                [t_get, trag_get] = self.fc.formation_cal()
                self.publish_pose(t_get, trag_get)
                self.update_car = np.tile(False, [self.num_car])

                self.update_exp_trag = False

            rate.sleep()

    def check_update(self):
        if (self.update_car == True).all():
            return True

    def callback_expect_formation(self, ex_form_msg):
        lines_size = len(ex_form_msg.channels[0].values)
        self.exp_formation = np.zeros((lines_size, 4))
        for ind_e in range(0, lines_size):
            for ind_p in range(0, 4):
                self.exp_formation[ind_e][ind_p] = ex_form_msg.channels[ind_p].values[ind_e]
        self.update_exp_trag = True

    def callback_gridmap(self, gridmap):
        self.extract_rects(gridmap)
        self.fc.cen_o = self.cen_l
        self.fc.rect_o = self.rect_l

    def extract_rects(self, gridmap):
        m_resolution = gridmap.info.resolution
        m_width = gridmap.info.width
        m_height = gridmap.info.height
        m_gridmap = list()
        for m_index in range(0, m_width*m_height):
            if gridmap.data[m_index] == 100:
                m_gridmap.append(
                    [int(m_index % m_width), int(m_index / m_width)])
        # print(m_gridmap)

        # start combining!!!
        arrange_s = list()
        while (len(m_gridmap) != 0):
            arrange_s.append([m_gridmap.pop(0)])
            m_gridmap_size = 0
            while m_gridmap_size != len(m_gridmap):
                m_gridmap_size = len(m_gridmap)
                for inte_sc in arrange_s[-1]:
                    flag_mer = False
                    for inte_p in m_gridmap:
                        if abs(inte_p[0] - inte_sc[0]) <= 1 and abs(inte_p[1] - inte_sc[1]) <= 1:
                            arrange_s[-1].append(m_gridmap.pop(m_gridmap.index(inte_p)))
                            flag_mer = True
                            break
                    if flag_mer == True:
                        break
            # print(arrange_s)

        # extract rect params
        cen_list = list()
        rect_list = list()
        for inter_ar in arrange_s:
            x_sort = sorted(inter_ar, key=lambda inter_ar: inter_ar[0])
            y_sort = sorted(inter_ar, key=lambda inter_ar: inter_ar[1])
            x_min = x_sort[0][0] * m_resolution
            x_max = x_sort[-1][0] * m_resolution
            y_min = y_sort[0][1] * m_resolution
            y_max = y_sort[-1][1] * m_resolution
            cen_list.append([(x_min+x_max)/2, (y_min+y_max)/2])
            rect_list.append([(-x_min+x_max)/2, (-y_min+y_max)/2])
        self.cen_l = np.array(cen_list)
        self.rect_l = np.array(rect_list)
        self.rect_l[self.rect_l <= 0.1] = 0.1

        print('self.cen_l')
        print(self.cen_l)
        print('self.rect_l')
        print(self.rect_l)

    def callback_car0(self, Odometry):
        # if Odometry.header.frame_id == '0':
        ind_car = 0
        self.store_car_pose(ind_car, Odometry)

    def callback_car1(self, Odometry):
        ind_car = 1
        self.store_car_pose(ind_car, Odometry)

    def callback_car2(self, Odometry):
        ind_car = 2
        self.store_car_pose(ind_car, Odometry)

    def store_car_pose(self, index_car, odo):
        self.update_car[index_car] = True
        self.pos_car[index_car, 0] = odo.pose.pose.position.x
        self.pos_car[index_car, 1] = odo.pose.pose.position.y
        print('receiving car num: ', index_car, self.pos_car[index_car, :])

    def publish_pose(self, time_s, cars_poses):
        path_cars = Path()
        # path.header.stamp.secs save the num of cars
        path_cars.header.stamp.secs = cars_poses.shape[1]
        # path.header.stamp.nsecs save the size of time
        path_cars.header.stamp.nsecs = time_s.shape[0]
        for index_car in range(0, cars_poses.shape[1]):
            for index_tt in range(0, time_s.shape[0]):
                pose_car = PoseStamped()
                pose_car.header.stamp.secs = time_s[index_tt]  # ms
                pose_car.header.stamp.secs = time_s[index_tt]  # ms
                pose_car.header.frame_id = str(index_car)
                pose_car.pose.position.x = cars_poses[index_tt, index_car, 0]
                pose_car.pose.position.y = cars_poses[index_tt, index_car, 1]
                path_cars.poses.append(pose_car)
        self.PathPublisher.publish(path_cars)


class formation:
    def __init__(self, yaml_params=None):
        # important params

        self.dt = yaml_params['dt']
        self.max_time = yaml_params['max_time']
        self.max_F = yaml_params['max_F']
        # self.change_diff_thred = 0.005

        # robot position
        self.num_robot = yaml_params['num_car']  # num of robots
        # no use, just for initial robot pos
        self.initpos = np.array([[1., 1.], [1., 1.5], [1., 2.0],
                                [2., 1.], [2., 1.5], [2., 2.]])
        self.initpos = self.initpos[0: self.num_robot]

        # initial position
        # self.initial_pos = (
        #     self.initpos + np.random.random_sample(np.shape(self.initpos))) + 1
        self.pos = self.initpos
        self.vel = np.zeros(np.shape(self.pos))

        # obstacle info.
        self.bool_obstacle = yaml_params['is_obstacle']
        # you can initial obstacle in yaml file, but you shouldnt
        self.cen_o = np.array([[]])
        self.rect_o = np.array([[]])

        # initial values
        self.d_dist = yaml_params['d_dist']  # m
        self.M_m = yaml_params['M_m']  # car dynamics
        self.D_d = yaml_params['D_d']
        self.k_k = yaml_params['k_k']

        # algorithm params
        self.kr = yaml_params['kr']
        self.ks = yaml_params['ks']
        self.ko = yaml_params['ko']

        # self.F_last_last = 0.1
        # self.F_last = 0.2
        self.R_s = 1.5
        self.pos_size = self.pos.shape[0]
        self.cen_vl = np.array([0., 0.])  # tragetary of virtual leader
        self.shape_ind = 0
        self.time_stamp = [
            x*self.dt for x in range(0, int((self.max_time-0)/self.dt))]
        self.show_img = yaml_params['show_img']
        self.save_img = yaml_params['save_img']
        self.select_pose = yaml_params['select_pose']
        self.shape_in = np.array([])

    def refresh_params(self):
        # obstacle info.
        # self.bool_obstacle = True
        # self.cen_o = np.array([[]])
        # self.rect_o = np.array([[]])
        self.pos_size = self.pos.shape[0]
        self.cen_vl = np.array([0., 0.])  # tragetary of virtual leader
        self.shape_ind = 0
        self.time_stamp = [
            x*self.dt for x in range(0, int((self.max_time-0)/self.dt))]

    def dynamics_d(self, F_f, vel, pos, dt):
        P_v = self.D_d + self.k_k
        dd = 10
        ddt = dt / dd
        pos_f = np.zeros(np.shape(pos))
        vel_f = np.zeros(np.shape(vel))
        for index in range(0, pos.shape[0]):
            acc_i = np.tile(np.zeros(np.shape(vel[index, :])), (dd, 1))
            vel_i = np.tile(vel[index, :], (dd, 1))
            pos_i = np.tile(pos[index, :], (dd, 1))
            for i in range(1, dd):
                acc_i[i, :] = (F_f[index, :] - P_v*vel_i[i, :])/self.M_m
                vel_i[i, :] = vel_i[i-1, :] + acc_i[i-1, :]*ddt
                pos_i[i, :] = pos_i[i-1, :] + vel_i[i-1, :] * \
                    ddt + acc_i[i-1, :]*(0.5*(ddt**2))
            pos_f[index, :] = pos_i[dd-1, :]
            vel_f[index, :] = vel_i[dd-1, :]
        return [pos_f, vel_f]

    def norm_row(self, input):
        row_re = np.sum(input**2, 1)**0.5
        return self.row_T(row_re)

    def row_T(self, input_row):  # input is one dim only.
        return input_row.reshape(input_row.shape[0], 1)

    def for_repulsive(self, pos_in):
        dim = pos_in.shape[0]
        quantity = np.ones((1, dim))*self.kr + np.arange(0., dim).transpose() * 0.5
        quantity_sq = np.matmul(quantity.T, quantity)
        distance_sq = np.zeros((dim, dim))
        for index in range(0, dim):
            distance_sq[index, :] = np.sum(
                (np.tile(pos_in[index, :], (dim, 1)) - pos_in)**2, 1)
        # distance_sq_inv = distance_sq**(-1)
        distance_sq_inv = np.divide(np.ones(distance_sq.shape), distance_sq, out=np.zeros_like(
            np.ones(distance_sq.shape), dtype=np.float64), where=distance_sq != 0)
        distance_sq_inv[np.isinf(distance_sq_inv)] = 0
        distance_n = distance_sq**(0.5)
        pos_x = pos_in[:, 0]
        pos_x = self.row_T(pos_x)
        pos_y = self.pos[:, 1]
        pos_y = self.row_T(pos_y)
        diff_pos_x = np.tile(pos_x, (1, dim)) - np.tile(pos_x.T, [dim, 1])
        diff_pos_y = np.tile(pos_y, (1, dim)) - np.tile(pos_y.T, [dim, 1])
        # pos_x_cos = diff_pos_x / distance_n
        pos_x_cos = np.divide(diff_pos_x, distance_n, out=np.zeros_like(
            diff_pos_x, dtype=np.float64), where=distance_n != 0)
        # pos_y_sin = diff_pos_y / distance_n
        pos_y_sin = np.divide(diff_pos_y, distance_n, out=np.zeros_like(
            diff_pos_y, dtype=np.float64), where=distance_n != 0)

        pos_x_cos[np.isnan(pos_x_cos)] = 0
        pos_y_sin[np.isnan(pos_y_sin)] = 0
        f_n_x = self.kr*quantity_sq*distance_sq_inv*pos_x_cos
        f_n_y = self.kr*quantity_sq*distance_sq_inv*pos_y_sin
        F_n = np.array([np.sum(f_n_x, 1), np.sum(f_n_y, 1)]).T
        return F_n

    def for_spherical(self, cen_s, R_s, pos):
        diff_p_c = pos - np.tile(cen_s, [pos.shape[0], 1])
        sum_p_c = self.row_T(np.sum(diff_p_c**2, 1))
        diff_sq = sum_p_c - np.ones(sum_p_c.shape)*(R_s**2)
        f_n = diff_p_c*(np.tile(diff_sq, [1, 2])*(-self.ks))
        return f_n

    def for_shape(self, pos_in, tar_shape):
        # print(pos_in,tar_shape)
        f_n = np.zeros(np.shape(pos_in))
        for index_pos in range(0, pos_in.shape[0]):
            pos_i = pos_in[index_pos, :]
            diff_p_a = np.tile(pos_i, (tar_shape.shape[0], 2)) - tar_shape
            diff_p_a1 = diff_p_a[:, 0:2]
            diff_p_a2 = diff_p_a[:, 2:4]
            diff_a2_a1 = tar_shape[:, 2:4] - tar_shape[:, 0:2]
            proj_re = np.sum(diff_p_a1*diff_a2_a1, 1)
            len_proj_a1_pp = self.row_T(proj_re)/self.norm_row(diff_a2_a1)
            dir_a1_pp = diff_a2_a1 / \
                np.tile(self.norm_row(diff_a2_a1), (1, 2)) * \
                np.tile(len_proj_a1_pp, (1, 2))
            proj_p_pp = dir_a1_pp - diff_p_a1
            bool_is_acute = (np.tile(np.sum(diff_p_a1*diff_a2_a1, 1)
                                     > 0, (2, 1)).T).astype(int)
            bool_isnot_acute = 1-bool_is_acute
            result_p_a1 = (bool_is_acute*proj_p_pp +
                           bool_isnot_acute*(-diff_p_a1))

            bool_is_acute = (
                np.tile(np.sum(diff_p_a2*(-diff_a2_a1), 1) > 0, (2, 1)).T).astype(int)
            bool_isnot_acute = 1 - bool_is_acute
            result_pp = (bool_is_acute*result_p_a1 +
                         bool_isnot_acute*(-diff_p_a2))
            ind = np.argmin(self.norm_row(result_pp))
            vec_i = result_pp[ind, :]
            min_v = np.linalg.norm(vec_i, 2)
            f_i = self.ks * min_v * vec_i
            f_n[index_pos, :] = f_i
            # print(result_p_a1)
        return f_n

    def for_obstacle(self, cen_o, rect_o, pos, vel):
        add_barr = 0  # add some m to avoid the obstacle.
        barrier = 3  # or add threshold.

        F_o = np.zeros(np.shape(pos))
        # print(cen_o.shape[1])
        if cen_o.shape[1] == 0:
            print("no obstacle!!")
            return F_o

        for index_o in range(0, cen_o.shape[0]):
            x0 = cen_o[index_o, 0]
            y0 = cen_o[index_o, 1]
            v1 = rect_o[index_o, 0]
            v2 = rect_o[index_o, 1]
            x = self.row_T(pos[:, 0])
            y = self.row_T(pos[:, 1])
            A_sq = 1/(2*((v1+add_barr)**2))
            B_sq = 1/(2*((v2+add_barr)**2))
            B_b = B_sq**(0.5)
            A_a = A_sq**(0.5)
            need_avoid = (((x-x0)**2)*(A_sq)+((y-y0)**2)*(B_sq)) < barrier
            dis2ob = np.abs((((x-x0)**2)*(A_sq)+((y-y0)**2)*(B_sq))-1)
            dis2ob[dis2ob == 0] = 0.0001
            dis2ob_inv = dis2ob**(-1) - 1/barrier
            inv_factor = dis2ob**(-2)*5
            f_cw = np.block([- (B_b)*(y - y0), (A_a)*(x - x0)])
            f_ccw = np.block([(B_b)*(y - y0), -(A_a)*(x - x0)])

            dir_sel_cw = self.row_T(np.sum(vel*f_cw, 1))
            dir_sel_cw = dir_sel_cw > 0
            dir_sel_ccw = dir_sel_cw == False
            dir = f_cw*np.tile(dir_sel_cw, [1, 2]) + \
                f_ccw*np.tile(dir_sel_ccw, [1, 2])
            dir = dir/np.tile((self.norm_row(dir)), [1, 2])
            dir = dir*np.tile(need_avoid, [1, 2])
            vel_norm = self.norm_row(vel)
            factors = np.tile(dis2ob_inv, [1, 2])*np.tile(inv_factor, [1, 2])
            Fs_max = 2
            norm_F = self.norm_row(factors)
            bool_bi = norm_F > Fs_max
            bool_sm = norm_F <= Fs_max
            factors_norm_max = factors/np.tile(norm_F, [1, 2])*Fs_max
            factors = factors * \
                np.tile(bool_sm, [1, 2]) + factors_norm_max * \
                np.tile(bool_bi, [1, 2])

            F_o_i = dir * np.tile(vel_norm, [1, 2]) * factors * self.ko
            print('F_o_i')
            print(F_o_i)
            F_o = F_o + F_o_i
            # you can cosider sum them up or just choose the biggest one.
        return F_o

    def formation_shape(self, sha_ind):
        if sha_ind == 0:
            shape_i = np.array([[-1, 0, 1, 0], [-1, 0, 0, -1.5],
                                [0, -1.5, 1, 0]])  # triangle
        if sha_ind == 3:
            shape_i = np.array([[0, 0, 0, 1], [0, 0, 1, -1.5],
                                [0, 0, -1, -1.5]])  # person
        if sha_ind == 2:
            shape_i = np.array([[-1, 0, 1, 0], [-1, -2, 1, -2],
                                [-1, -2, -1, 0], [1, -2, 1, 0]])  # square
        if sha_ind == 1:
            shape_i = np.array([[-2, 0, 2, 0]])  # line
        if sha_ind == 4:
            shape_i = np.array([])  # none
        return shape_i

    def selectPose(self, trag_get):

        pos_select = np.zeros((0, trag_get.shape[1], 2))
        t_select = np.zeros(0)
        pos_last = trag_get[0, :, :]
        # t_last = 0
        for index_s in range(1, trag_get.shape[0]):
            pos_diff = trag_get[index_s, :, :] - pos_last
            if ((self.norm_row(pos_diff) > self.d_dist) == True).any():
                pos_select = np.append(pos_select, trag_get[index_s, :, :].reshape(
                    (1, trag_get[index_s, :, :].shape[0], 2), order='A'), axis=0)
                t_select = np.append(t_select, np.array(
                    [int(index_s*(self.dt*1000))]), axis=0)
                # t_last = index_s
                pos_last = trag_get[index_s, :, :]
        print('previous pose shape: ')
        print(trag_get.shape)
        print('after selection pose shape: ')
        print(pos_select.shape)

        return [t_select, pos_select]

    def formation_plot(self, pos_save, pos_save_for):
        # plt.clf()
        for index_time in range(0, pos_save.shape[1]):
            pos_i = pos_save[:, index_time, :]
            plt.plot(pos_i[:, 0], pos_i[:, 1], '.')

        for index_time in range(0, pos_save_for.shape[0]):
            pos_i = pos_save_for[index_time, :, :]
            plt.plot(pos_i[:, 0], pos_i[:, 1], '-^')

        if self.bool_obstacle:
            if self.cen_o.shape[1] == 0:
                print("no obstacle!!")
                return
            for index_o in range(0, self.cen_o.shape[0]):
                x0 = self.cen_o[index_o, 0]
                y0 = self.cen_o[index_o, 1]
                v1 = self.rect_o[index_o, 0]
                v2 = self.rect_o[index_o, 1]
                plt.plot([x0 - v1, x0 - v1, x0 + v1, x0 + v1, x0 - v1],
                         [y0 - v2, y0 + v2, y0 + v2, y0 - v2, y0 - v2], '-.s')
            # plot obstacle here

        # plt.pause(0.001)
        # plt.ioff()
        # print(pos)
        # plt.show()

    def formation_cal(self):
        pos_save = np.zeros((0, self.pos_size, 2))
        pos_save_for = np.zeros((0, self.pos_size, 2))
        vel_save = np.zeros((0, self.pos_size, 2))

        for index_t in self.time_stamp:
            print('index_t')
            print(index_t)
            # calculate of forces
            # # here for virtual leaders dynamics
            # [cen_vl, ~] = dynamics(np.zeros(np.shape(cen)), np.array([0., 0.]), cen_vl, dt)
            F_r = self.for_repulsive(self.pos)

            # f_s = for_spherical(cen_vl, R_s, pos) # circle formation
            # shape_in = self.formation_shape(self.shape_ind)  # custom formation
            f_s = self.for_shape(self.pos, self.shape_in)

            # print(F_r)
            # print(f_s)

            Fr_max = self.max_F
            norm_F = self.norm_row(F_r)
            bool_bi = norm_F > Fr_max
            bool_sm = norm_F <= Fr_max

            F_r_norm_max = F_r/np.tile(norm_F, [1, 2])*Fr_max
            F_r = F_r*np.tile(bool_sm, [1, 2]) + \
                F_r_norm_max*np.tile(bool_bi, [1, 2])

            Fs_max = self.max_F
            norm_F = self.norm_row(f_s)
            bool_bi = norm_F > Fs_max
            bool_sm = norm_F <= Fs_max
            f_s_norm_max = f_s/np.tile(norm_F, [1, 2])*Fs_max
            f_s = f_s*np.tile(bool_sm, [1, 2]) + \
                f_s_norm_max*np.tile(bool_bi, [1, 2])
            f_s[np.isnan(f_s)] = 0
            f_a = F_r + f_s

            # mute obstcle force here if you want
            if self.bool_obstacle:
                f_o = self.for_obstacle(self.cen_o, self.rect_o, self.pos, f_a)
                f_a = f_a + f_o

            F_f = f_a

            min_norm_thred = 0.2  # should be in front of min_norm = 4
            print('norm(F_f, 2)')
            print(np.linalg.norm(F_f, 2))
            if np.linalg.norm(F_f, 2) < min_norm_thred:
                print('shape done!')
                print(self.shape_ind)
                self.shape_ind = self.shape_ind + 1
                pos_save_for = np.append(pos_save_for, pos_f.reshape(
                    (1, pos_f.shape[0], 2), order='A'), axis=0)

            min_norm = 4
            if np.linalg.norm(F_f, 2) < min_norm:
                F_f = (F_f/np.linalg.norm(F_f, 2))*min_norm

            # F_last_last = F_last
            # F_last = F_f

            [pos_f, vel_f] = self.dynamics_d(F_f, self.vel, self.pos, self.dt)
            self.pos = pos_f
            self.vel = vel_f
            pos_save = np.append(pos_save, pos_f.reshape(
                (1, pos_f.shape[0], 2), order='A'), axis=0)
            vel_save = np.append(vel_save, vel_f.reshape(
                (1, vel_f.shape[0], 2), order='A'), axis=0)

            # print('self.pos')
            # print(self.pos)
            # print('vel')
            # print(vel)

            # plot the result in matplot
            # self.formation_plot( pos_save, pos_save_for)
            if self.shape_ind == 1 or index_t == self.time_stamp[-1]:
                if self.select_pose:
                    [t, pos_save] = self.selectPose(pos_save)
                    # print(t)
                else:
                    t = np.array(self.time_stamp[0:(pos_save.shape[0])])*(1000)
                    # print(t)
                if self.show_img:
                    self.formation_plot(pos_save, pos_save_for)
                    if self.save_img:
                        file_dir = './tra_output/'
                        if not os.path.isdir(file_dir):
                            os.makedirs(file_dir)

                        plt.savefig('./tra_output/image' + str(np.random.randint(1,
                                    1000, [1], dtype=np.uint32)[0]) + '.png')
                    plt.pause(2.5)
                    plt.clf()
                    plt.close()
                self.refresh_params()
                return [t, pos_save]


if __name__ == '__main__':
    try:
        fc_object = Formation_cars()
        fc_object.formation_node()
    except rospy.ROSInterruptException:
        pass
