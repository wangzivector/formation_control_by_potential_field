#!/usr/bin/env python2.7
import numpy as np
import math
import random

dt = 0.01
max_F = 30
a = math.sqrt(3)
change_diff_thred = 0.005
target = np.array([[2., 0.], [1., a], [-1., a],
                   [-1., -a], [1., 0.1], [1., 3.]])

num_robot = 4
target = target[0:num_robot]
# initial position
initial_pos = (target + np.random.random_sample(np.shape(target))) + 2
pos = initial_pos
vel = np.zeros(np.shape(target))

# initial values
F_last_last = 0.1
F_last = 0.2
pos_size = pos.shape[0]
pos_save = np.zeros((0, pos_size, 2))
vel_save = np.zeros((0, pos_size, 2))
cen_vl = np.array([0., 0.])  # tragetary of virtual leader
shape_ind = 0


def dynamics_d(F, vel, pos, dt):
    M = 1
    D = 1
    k = 9
    P_v = D + k

    dd = 100
    ddt = dt / dd
    pos_f = np.zeros(np.shape(pos))
    vel_f = np.zeros(np.shape(vel))
    for index in range(0, pos.shape[0]):
        acc_i = np.tile(np.zeros(np.shape(vel[index, :])), (dd, 1))
        vel_i = np.tile(vel[index, :], (dd, 1))
        pos_i = np.tile(pos[index, :], (dd, 1))
        for i in range(1, dd):
            acc_i[i, :] = (F[index, :] - P_v*vel_i[i, :])/M
            vel_i[i, :] = vel_i[i-1, :] + acc_i[i-1, :]*ddt
            pos_i[i, :] = pos_i[i-1, :] + vel_i[i-1, :] * \
                ddt + acc_i[i-1, :]*(0.5*(ddt**2))
        pos_f[index, :] = pos_i[dd, :]
        vel_f[index, :] = vel_i[dd, :]
    return [pos_f, vel_f]


def norm_row(input):
    row_re = np.sum(input**2, 1)**0.5
    return row_T(row_re)


def row_T(input_row):
    return input_row.reshape(input_row.shape[0], 1)


def for_repulsive(pos_in):
    dim = pos_in.shape[0]
    quantity = np.ones((1, dim))*2 + np.arange(0., dim).transpose() * 0.5
    quantity_sq = np.matmul(quantity.T, quantity)
    kr = 1
    distance_sq = np.zeros((dim, dim))
    for index in range(0, dim):
        distance_sq[index, :] = np.sum(
            (np.tile(pos_in[index, :], (dim, 1)) - pos_in)**2, 1)
    distance_sq_inv = distance_sq**(-1)
    distance_sq_inv[np.isinf(distance_sq_inv)] = 0
    distance_n = distance_sq**(0.5)
    pos_x = pos_in[:, 0]
    pos_x = row_T(pos_x)
    pos_y = pos[:, 1]
    pos_y = row_T(pos_y)
    diff_pos_x = np.tile(pos_x, (1, dim)) - np.tile(pos_x.T, [dim, 1])
    diff_pos_y = np.tile(pos_y, (1, dim)) - np.tile(pos_y.T, [dim, 1])
    pos_x_cos = diff_pos_x / distance_n
    pos_y_sin = diff_pos_y / distance_n
    pos_x_cos[np.isnan(pos_x_cos)] = 0
    pos_y_sin[np.isnan(pos_y_sin)] = 0
    f_n_x = kr*quantity_sq*distance_sq_inv*pos_x_cos
    f_n_y = kr*quantity_sq*distance_sq_inv*pos_y_sin
    F_n = np.array([np.sum(f_n_x, 1), np.sum(f_n_y, 1)]).T
    return F_n


def for_shape(pos_in, tar_shape):
    ks = 500
    f_n = np.zeros(np.shape(pos_in))
    for index_pos in range(0, pos_in.shape[0]):
        pos_i = pos_in[index_pos, :]
        diff_p_a = np.tile(pos_i, (tar_shape.shape[0], 2)) - tar_shape
        diff_p_a1 = diff_p_a[:, 0:2]
        diff_p_a2 = diff_p_a[:, 2:4]
        diff_a2_a1 = tar_shape[:, 2:4] - tar_shape[:, 0:2]
        proj_re = np.sum(diff_p_a1*diff_a2_a1, 1)
        len_proj_a1_pp = row_T(proj_re)/norm_row(diff_a2_a1)
        dir_a1_pp = diff_a2_a1 / \
            np.tile(norm_row(diff_a2_a1), (1, 2)) * \
            np.tile(len_proj_a1_pp, (1, 2))
        proj_p_pp = dir_a1_pp - diff_p_a1
        bool_is_acute = (np.tile(np.sum(diff_p_a1*diff_a2_a1, 1)
                         > 0, (2, 1)).T).astype(int)
        bool_isnot_acute = 1-bool_is_acute
        result_p_a1 = (bool_is_acute*proj_p_pp + bool_isnot_acute*(-diff_p_a1))

        bool_is_acute = (
            np.tile(np.sum(diff_p_a2*(-diff_a2_a1), 1) > 0, (2, 1)).T).astype(int)
        bool_isnot_acute = 1 - bool_is_acute
        result_pp = (bool_is_acute*result_p_a1 + bool_isnot_acute*(-diff_p_a2))
        ind = np.argmax(norm_row(result_pp), axis=0)
        vec_i = result_pp[ind, :]
        min_v = np.linalg.norm(vec_i, 2)
        f_i = ks * min_v * vec_i
        f_n[index_pos, :] = f_i
        # print(result_p_a1)
    return f_n


def for_obstacle(cen_o, rect_o, pos, vel):
    add_barr = 0  # add some m to avoid the obstacle.
    barrier = 3  # or add threshold.

    F_o = np.zeros(np.shape(pos))
    for index_o in range(0, cen_o.shape[0]):
        x0 = cen_o[index_o, 0]
        y0 = cen_o[index_o, 1]
        v1 = rect_o[index_o, 0]
        v2 = rect_o[index_o, 1]
        x = row_T(pos[:, 0])
        y = row_T(pos[:, 1])
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

        dir_sel_cw = row_T(np.sum(vel*f_cw, 1))
        dir_sel_cw = dir_sel_cw > 0
        dir_sel_ccw = dir_sel_cw == False
        dir = f_cw*np.tile(dir_sel_cw, [1, 2]) + \
            f_ccw*np.tile(dir_sel_ccw, [1, 2])
        dir = dir/np.tile((norm_row(dir)), [1, 2])
        dir = dir*np.tile(need_avoid, [1, 2])
        vel_norm = norm_row(vel)
        factors = np.tile(dis2ob_inv, [1, 2])*np.tile(inv_factor, [1, 2])
        Fs_max = 2
        norm_F = norm_row(factors)
        bool_bi = norm_F > Fs_max
        bool_sm = norm_F <= Fs_max
        factors_norm_max = factors/np.tile(norm_F, [1, 2])*Fs_max
        factors = factors * \
            np.tile(bool_sm, [1, 2]) + factors_norm_max * \
            np.tile(bool_bi, [1, 2])
        print(factors)
        F_o_i = dir*np.tile(vel_norm, [1,2])*factors
        F_o = F_o + F_o_i; 
        # you can cosider sum them up or just choose the biggest one.
    return F_o


tar = np.array([[0., 0., 1., 0.]])
pos = np.array([[1., 1.], [0.5, -1.5]])
vel = np.array([[-5, 1], [0.1, 1]])
cen_o = np.array([[0., 0.]])
rect_o = np.array([[1., 1.]])
print(for_obstacle(cen_o, rect_o, pos, vel))

# print (for_shape(pos, tar))

# time_stamp = [x*dt for x in range(0, int((30-0)/dt))]
# for index_t in time_stamp:
#     # calculate of forces
#     R_f = 1
#     cen_o = np.array([-3., 0.])
#     rect_o = np.array([1.2, 0.8])
#     pos_o = pos
#     vel_o = vel
#     # here for virtual leaders dynamics
#     # [cen_vl, ~] = dynamics(np.zeros(np.shape(cen)), np.array([0., 0.]), cen_vl, dt)
#     F_r = for_repulsive(pos)
#     # f_s = for_spherical(cen_vl, R_f, pos)
