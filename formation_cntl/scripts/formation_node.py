#!/usr/bin/env python
import numpy as np
import math, os
import matplotlib.pyplot as plt
from numpy.core.defchararray import array

import rospy
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist


class Formation_cars:
    def formation_node(self):
        self.num_car = 3
        self.update_car = np.tile(False, [self.num_car])
        self.pos_car = np.zeros([self.num_car, 2])
        rospy.init_node('formation_node')

        fc = formation(self.num_car)
        print('created formation object, initial pose of car:')
        print(fc.pos)
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)# parameters

        # subscriber of position of cars
        self.car0_odom = rospy.Subscriber(
            '/car0/odom', Odometry, self.callback_car0)
        self.car1_odom = rospy.Subscriber(
            '/car1/odom', Odometry, self.callback_car1)
        self.car2_odom = rospy.Subscriber(
            '/car2/odom', Odometry, self.callback_car2)

        # close loop repeating
        rate = rospy.Rate(5)  # several hz
        while not rospy.is_shutdown():
            if self.check_update():
                hello_str = "check success, formation node running ... %s" % rospy.get_time()
                rospy.loginfo(hello_str)
                print(self.pos_car)
                for ind_c in range(0, self.num_car):
                    fc.pos[ind_c, 0] = self.pos_car[ind_c, 0]
                    fc.pos[ind_c, 1] = self.pos_car[ind_c, 1]
                fc.formation_cal()
                self.update_car = np.tile(False, [self.num_car])
            rate.sleep()

    def check_update(self):
        if (self.update_car == True).all():
            return True

    def callback_car0(self, Odometry):
        # if Odometry.header.frame_id == '0':
        ind_car = 0
        self.store_car_pose(ind_car, Odometry)

    def callback_car1(self, Odometry):
        # if Odometry.header.frame_id == '0':
        ind_car = 1
        self.store_car_pose(ind_car, Odometry)

    def callback_car2(self, Odometry):
        # if Odometry.header.frame_id == '0':
        ind_car = 2
        self.store_car_pose(ind_car, Odometry)

    def store_car_pose(self, index_car, odo):
        self.update_car[index_car] = True
        self.pos_car[index_car, 0] = odo.pose.pose.position.x
        self.pos_car[index_car, 1] = odo.pose.pose.position.y
        print('receiving car num: ', index_car, self.pos_car[index_car, :])


class formation:
    def __init__(self, car_num=3):
        # important params
        self.dt = 0.01
        self.time = 15
        self.max_F = 30
        self.a = math.sqrt(3)
        # self.change_diff_thred = 0.005

        # robot position
        self.num_robot = car_num  # num of robots
        self.target = np.array([[2., 0.], [1., self.a], [-1., self.a],
                                [-1., -self.a], [1., 0.1], [1., 3.]])  # no use, just for initial robot pos
        self.target = self.target[0:self.num_robot]

        # initial position
        self.initial_pos = (
            self.target*0.2 + np.random.random_sample(np.shape(self.target))) + 2
        self.pos = self.initial_pos
        self.vel = np.zeros(np.shape(self.target))

        # obstacle info.
        self.bool_obstacle = True
        self.cen_o = np.array([[0., 2.]])
        self.rect_o = np.array([[0.5, 0.8]])

        # initial values

        # self.F_last_last = 0.1
        # self.F_last = 0.2
        self.R_s = 1.5
        self.pos_size = self.pos.shape[0]
        self.pos_save = np.zeros((0, self.pos_size, 2))
        self.pos_save_for = np.zeros((0, self.pos_size, 2))
        self.vel_save = np.zeros((0, self.pos_size, 2))
        self.cen_vl = np.array([0., 0.])  # tragetary of virtual leader
        self.shape_ind = 0
        self.time_stamp = [
            x*self.dt for x in range(0, int((self.time-0)/self.dt))]

    def refresh_params(self):
        # obstacle info.
        self.bool_obstacle = True
        self.cen_o = np.array([[0., 2.]])
        self.rect_o = np.array([[0.5, 0.8]])
        self.pos_size = self.pos.shape[0]
        self.pos_save = np.zeros((0, self.pos_size, 2))
        self.pos_save_for = np.zeros((0, self.pos_size, 2))
        self.vel_save = np.zeros((0, self.pos_size, 2))
        self.cen_vl = np.array([0., 0.])  # tragetary of virtual leader
        self.shape_ind = 0
        self.time_stamp = [
            x*self.dt for x in range(0, int((self.time-0)/self.dt))]
 
    def dynamics_d(self, F_f, vel, pos, dt):
        M_m = 1
        D_d = 1
        k_k = 9
        P_v = D_d + k_k

        dd = 10
        ddt = dt / dd
        pos_f = np.zeros(np.shape(pos))
        vel_f = np.zeros(np.shape(vel))
        for index in range(0, pos.shape[0]):
            acc_i = np.tile(np.zeros(np.shape(vel[index, :])), (dd, 1))
            vel_i = np.tile(vel[index, :], (dd, 1))
            pos_i = np.tile(pos[index, :], (dd, 1))
            for i in range(1, dd):
                acc_i[i, :] = (F_f[index, :] - P_v*vel_i[i, :])/M_m
                vel_i[i, :] = vel_i[i-1, :] + acc_i[i-1, :]*ddt
                pos_i[i, :] = pos_i[i-1, :] + vel_i[i-1, :] * \
                    ddt + acc_i[i-1, :]*(0.5*(ddt**2))
            pos_f[index, :] = pos_i[dd-1, :]
            vel_f[index, :] = vel_i[dd-1, :]
        return [pos_f, vel_f]

    def norm_row(self, input):
        row_re = np.sum(input**2, 1)**0.5
        return self.row_T(row_re)

    def row_T(self, input_row):
        return input_row.reshape(input_row.shape[0], 1)

    def for_repulsive(self, pos_in):
        dim = pos_in.shape[0]
        quantity = np.ones((1, dim))*4 + np.arange(0., dim).transpose() * 0.5
        quantity_sq = np.matmul(quantity.T, quantity)
        kr = 1
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
        f_n_x = kr*quantity_sq*distance_sq_inv*pos_x_cos
        f_n_y = kr*quantity_sq*distance_sq_inv*pos_y_sin
        F_n = np.array([np.sum(f_n_x, 1), np.sum(f_n_y, 1)]).T
        return F_n

    def for_spherical(self, cen_s, R_s, pos):
        ks = 50
        diff_p_c = pos - np.tile(cen_s, [pos.shape[0], 1])
        sum_p_c = self.row_T(np.sum(diff_p_c**2, 1))
        diff_sq = sum_p_c - np.ones(sum_p_c.shape)*(R_s**2)
        f_n = diff_p_c*(np.tile(diff_sq, [1, 2])*(-ks))
        return f_n

    def for_shape(self, pos_in, tar_shape):
        # print(pos_in,tar_shape)
        ks = 100
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
            f_i = ks * min_v * vec_i
            f_n[index_pos, :] = f_i
            # print(result_p_a1)
        return f_n

    def for_obstacle(self, cen_o, rect_o, pos, vel):
        add_barr = 0  # add some m to avoid the obstacle.
        barrier = 3  # or add threshold.

        F_o = np.zeros(np.shape(pos))
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
            print(factors)
            F_o_i = dir*np.tile(vel_norm, [1, 2])*factors
            F_o = F_o + F_o_i
            # you can cosider sum them up or just choose the biggest one.
        return F_o

    def formation_shape(self, sha_ind):
        if sha_ind == 0:
            shape_ind = np.array([[-1, 0, 1, 0], [-1, 0, 0, -1.5],
                                  [0, -1.5, 1, 0]])  # triangle
        if sha_ind == 3:
            shape_ind = np.array([[0, 0, 0, 1], [0, 0, 1, -1.5],
                                  [0, 0, -1, -1.5]])  # person
        if sha_ind == 2:
            shape_ind = np.array([[-1, 0, 1, 0], [-1, -2, 1, -2],
                                  [-1, -2, -1, 0], [1, -2, 1, 0]])  # square
        if sha_ind == 1:
            shape_ind = np.array([[-2, 0, 2, 0]])  # line
        if sha_ind == 4:
            shape_ind = np.array([])  # none
        return shape_ind

    def formation_plot(self):
        # plt.clf()
        for index_time in range(0, self.pos_save.shape[1]):
            pos_i = self.pos_save[:, index_time, :]
            plt.plot(pos_i[:, 0], pos_i[:, 1], '.')

        for index_time in range(0, self.pos_save_for.shape[0]):
            pos_i = self.pos_save_for[index_time, :, :]
            plt.plot(pos_i[:, 0], pos_i[:, 1], '-^')

        if self.bool_obstacle:
            for index_o in range(0,):
                x0 = self.cen_o(index_o, 1)
                y0 = self.cen_o(index_o, 2)
                v1 = self.rect_o(index_o, 1)
                v2 = self.rect_o(index_o, 2)
            # plot obstacle here

        # plt.pause(0.001)
        # plt.ioff()
        print(self.pos)
        # plt.show()

    def formation_cal(self):
        for index_t in self.time_stamp:
            print('index_t')
            print(index_t)
            # calculate of forces
            # # here for virtual leaders dynamics
            # [cen_vl, ~] = dynamics(np.zeros(np.shape(cen)), np.array([0., 0.]), cen_vl, dt)
            F_r = self.for_repulsive(self.pos)

            # f_s = for_spherical(cen_vl, R_s, pos) # circle formation

            shape_in = self.formation_shape(self.shape_ind)  # custom formation
            f_s = self.for_shape(self.pos, shape_in)

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

            min_norm_thred = 0.3  # should be in front of min_norm = 4
            print('norm(F_f, 2)')
            print(np.linalg.norm(F_f, 2))
            if np.linalg.norm(F_f, 2) < min_norm_thred:
                print('shape done!')
                print(self.shape_ind)
                self.shape_ind = self.shape_ind + 1
                self.pos_save_for = np.append(self.pos_save_for, pos_f.reshape(
                    (1, pos_f.shape[0], 2), order='A'), axis=0)

            min_norm = 4
            if np.linalg.norm(F_f, 2) < min_norm:
                F_f = (F_f/np.linalg.norm(F_f, 2))*min_norm

            # F_last_last = F_last
            # F_last = F_f

            [pos_f, vel_f] = self.dynamics_d(F_f, self.vel, self.pos, self.dt)
            self.pos = pos_f
            self.vel = vel_f
            self.pos_save = np.append(self.pos_save, pos_f.reshape(
                (1, pos_f.shape[0], 2), order='A'), axis=0)
            self.vel_save = np.append(self.vel_save, vel_f.reshape(
                (1, vel_f.shape[0], 2), order='A'), axis=0)

            print('self.pos')
            print(self.pos)
            # print('vel')
            # print(vel)

            # plot the result in matplot
            # self.formation_plot()
            if self.shape_ind == 2 or index_t == self.time_stamp[-1]:
                self.formation_plot()
                plt.ion()
                file_dir = './tra_output/'
                if not os.path.isdir(file_dir):
                        os.makedirs(file_dir)
                
                plt.savefig('./tra_output/image' + str(np.random.randint(1,
                            1000, [1], dtype=np.uint32)[0]) + '.png')
                plt.pause(0.01)
                plt.clf()
                plt.close()
                self.refresh_params()
                break


if __name__ == '__main__':
    try:
        fc_object = Formation_cars()
        fc_object.formation_node()
    except rospy.ROSInterruptException:
        pass
