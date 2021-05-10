#!/usr/bin/env python
import numpy as np

# m_gridmap = [[4, 4], [2, 3], [3, 4], [6, 7], [6, 5], [9, 8], [5, 6], [1, 7]]


# car_final = pos_save[:, -1, :]


class testcode:
    def __init__(self):
        self.shape_in = np.array([[0, 0, 4, 0], [4, 4, 2, 2]])
        final_car = np.array([[3, 4],[0.1, 0.2],[1, 2],[3.5, 0],[2.8,3.2],[4, -5]])
        self.for_fix_target(final_car)

    def norm_row(self, input):
        row_re = np.sum(input**2, 1)**0.5
        return self.row_T(row_re)

    def row_T(self, input_row):  # input is one dim only.
        return input_row.reshape(input_row.shape[0], 1)

    def for_fix_target(self, car_final):
        shape_points = np.block([[self.shape_in[:, 2:4]],[self.shape_in[:, 0:2]]])
        dependent_points = np.array([shape_points[0][:]])
        for ind_ang in range(1, shape_points.shape[0]):
            check_point = shape_points[ind_ang][:]
            flag_same = False
            for ind_check in range(0,dependent_points.shape[0]):
                if np.linalg.norm(dependent_points[ind_check][:] - check_point, 2) > 0:
                    continue
                else:
                    flag_same = True
            if not flag_same:
                dependent_points = np.block([[dependent_points], [check_point]])
        print('dependent_points:\n', dependent_points)

        car_align = np.ones(car_final.shape)*np.nan
        # car_final match with dependent_points 

        # get closest ancle
        distance_matrix = np.zeros([car_final.shape[0], dependent_points.shape[0]])
        for ind_car in range(0, car_final.shape[0]):
            for ind_dp in range(0, dependent_points.shape[0]):
                distance_matrix[ind_car, ind_dp] = np.linalg.norm(dependent_points[ind_dp,:] - car_final[ind_car,:],2)
        
        distance_all = np.sum(np.sum(distance_matrix,1),0)
        for ind_car in range(0, car_final.shape[0]):
            if ind_car == dependent_points.shape[0]:
                break
            min_axis_0 = np.argmin(distance_matrix)//distance_matrix.shape[1] # car index
            min_axis_1 = np.argmin(distance_matrix)%distance_matrix.shape[1] # ancle index
            car_align[min_axis_0,:] = dependent_points[min_axis_1,:]
            distance_matrix[min_axis_0, :] = distance_matrix[min_axis_0, :] + distance_all
            distance_matrix[:, min_axis_1] = distance_matrix[:, min_axis_1] + distance_all
        if car_final.shape[0] > dependent_points.shape[0]:
            for ind_left in range(0, car_final.shape[0]):
                if np.isnan(car_align[ind_left,0]):
                    pos_i = car_final[ind_left, :]
                    tar_shape = self.shape_in
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
                    
                    bool_is_acute_1 = (np.tile(np.sum(diff_p_a1*diff_a2_a1, 1)
                                            > 0, (2, 1)).T).astype(int)
                    bool_isnot_acute = 1-bool_is_acute_1
                    result_p_a1 = (bool_is_acute_1*proj_p_pp +
                                bool_isnot_acute*(-diff_p_a1))

                    bool_is_acute_2 = (
                        np.tile(np.sum(diff_p_a2*(-diff_a2_a1), 1) > 0, (2, 1)).T).astype(int)
                    bool_isnot_acute = 1 - bool_is_acute_2
                    result_pp = (bool_is_acute_2*result_p_a1 +
                                bool_isnot_acute*(-diff_p_a2))
                    ind = np.argmin(self.norm_row(result_pp))

                    vec_i = result_pp[ind, :]
                    project_result_pi = pos_i + vec_i
                    car_align[ind_left][:] = project_result_pi

                    if bool_is_acute_1[ind][0] == 0 or bool_is_acute_2[ind][0] == 0:
                        car_align[ind_left][:] = np.array([np.nan,ind])
        print("orign:", car_final,"reassign to :", car_align)

ob = testcode()

# shape_points = np.array(m_gridmap)
# shape_points = np.block([[self.shape_in[:, 2:4]],[self.shape_in[:, 0:2]]])
# dependent_point = np.array([shape_points[0][:]])
# for ind_ang in range(1, shape_points.shape[0]):
#     check_point = shape_points[ind_ang][:]
#     flag_same = False
#     for ind_check in range(0,dependent_point.shape[0]):
#         if np.linalg.norm(dependent_point[ind_check][:] - check_point, 2) > 0:
#             continue
#         else:
#             flag_same = True
#     if not flag_same:
#         dependent_point = np.block([[dependent_point], [check_point]])
# print(dependent_point)
exit()

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
    print(arrange_s)
# extract rect params
cen_list = list()
rect_list = list()
for inter_ar in arrange_s:
    x_sort = sorted(inter_ar,key = lambda inter_ar:inter_ar[0])
    y_sort = sorted(inter_ar,key = lambda inter_ar:inter_ar[1])
    x_min = x_sort[0][0]
    x_max = x_sort[-1][0]
    y_min = y_sort[0][1]
    y_max = y_sort[-1][1]
    cen_list.append([(x_min+x_max)/2,(y_min+y_max)/2])
    rect_list.append([(-x_min+x_max)/2,(-y_min+y_max)/2])
cen_list = np.array(cen_list)
rect_list = np.array(rect_list)
rect_list[rect_list <= 0.1] = 0.1

print(cen_list)
print(rect_list)