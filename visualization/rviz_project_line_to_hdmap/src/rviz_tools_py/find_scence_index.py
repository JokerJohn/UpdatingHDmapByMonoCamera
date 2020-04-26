#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rviz_tools_py.public_para import Public_para as _para

import numpy as np
def find_scence_index(scence_id):
    path = '/home/apple/catkin_ws/src/rviz_tools_py/src/rviz_tools_py/scence_id_all.txt'
    scences_list =list(np.loadtxt(path,dtype='str'))
    index = scences_list.index(scence_id)
    return index
