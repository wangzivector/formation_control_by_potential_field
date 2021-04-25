#!/usr/bin/env python
import numpy as np

m_gridmap = [[4, 4], [2, 3], [3, 4], [6, 7], [6, 5], [9, 8], [5, 6], [1, 7], [11, 7]]

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