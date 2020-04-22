#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python includeswhich
import numpy as np
import random
import glob
from rviz_tools_py.public_para import Public_para as _para

# gps_test_path = '/home/apple/catkin_ws/src/rviz_tools_py/data/GPStrafficLine.txt'
# test_bug_txt = '/home/apple/catkin_ws/test_bug.txt'
# np_data_path = '/home/apple/catkin_ws/src/rviz_tools_py/data/gps_pre_frame2.npy'
# light_path = '/home/apple/catkin_ws/src/rviz_tools_py/new_data/traffic_hdmap.txt'


gps_test_path = _para.gps_test_path
np_data_path = _para.np_data_path
test_path = _para.test_path
plus_data_path = _para.plus_data_path

def read_points_per_frame(path=gps_test_path):

    tmp = np.loadtxt(gps_test_path, delimiter=' ')
    # print(np.shape(tmp))
    frame = []
    points_line = []
    points_line_i = []
    before = 0
    headings = []

    for ele in tmp:

        if type(before) == int:
            before = ele[:2]
            frame.append([ele[0], ele[1], 0])
            points_line_i.append([ele[2], ele[3], 0])
            headings.append(ele[4])
            continue

        if all(ele[:2] == before):
            # print('----')
            points_line_i.append([ele[2], ele[3], 0])
        else:
            frame.append([ele[0], ele[1], 0])
            headings.append(ele[4])
            before = ele[:2]
            points_line.append(points_line_i)
            points_line_i = []
            points_line_i.append([ele[2], ele[3], 0])

    points_line.append(points_line_i)

    return frame, points_line, headings

def save_plus_data(plus_data_path, file_name):
    with open(plus_data_path, 'r') as f:
        line = f.readline()
        data_list = []
        while line:
            num = map(float,line.split())
            # print(type(num))
            data_list.append(num)
            line = f.readline()
    data_array = np.array(data_list)
    np.save(file_name, data_array)


def load_and_save():
    frame, points_line, headings = read_points_per_frame()
    np.save(np_data_path, [frame, points_line, headings])

def print_test():
    trajactor, points_line, headings = np.load(np_data_path)
    with open(test_path, 'w') as fb:
        print >> fb, 'trajactor'
        print >> fb, np.shape(trajactor),type(trajactor)
        print >> fb, np.shape(trajactor[0]),type(trajactor[0])
        print >> fb, trajactor
        print >> fb, 'points_line'
        print >> fb, np.shape(points_line), type(points_line)
        print >> fb, np.shape(points_line[0]), type(points_line[0])
        print >> fb, points_line
        print >> fb, 'headings'
        print >> fb, np.shape(headings), type(headings)
        print >> fb, np.shape(headings[0]), type(headings[0])
        print >> fb, headings


if __name__ == "__main__":
    # load_and_save()
    # print_test()
    list = [16, 67]
    path = '/home/apple/catkin_ws/src/rviz_tools_py/new_data/file/'
    plus_data_path = '/home/apple/catkin_ws/src/rviz_tools_py/new_data/file/'# _para.plus_data_path
    list_num = 0
    for l in list:
        save_plus_data(path+str(l)+'A.txt', path+str(l)+'A.npy')
        save_plus_data(path+str(l)+'B.txt', path+str(l)+'B.npy')
        print(str(l)+'A.npy')
        # blue_point = np.load(plus_data_path+str(list[list_num])+'A.npy', allow_pickle=True)
        # red_point = np.load(plus_data_path+str(list[list_num])+'B.npy', allow_pickle=True)
        # print('blue_point', blue_point)
        # print('red_point', red_point)
    # save_plus_data(plus_data_path, 'divider12')
    # tmp = np.load(plus_data_path+'divider.npy', allow_pickle=True)
    # print(tmp)


    # tmp = np.loadtxt(gps_test_path, delimiter=' ')

    # frame = []
    # points_line = []
    # points_line_i = []
    # before = 0
    # line_list = []

    # for i, ele in enumerate(tmp):

    #     if all(ele[:2] == before):
    #         line_list.append(i)
    #     elif before !=0:
    #         frame.append(ele[:2])
    #         before  = ele[:2]
    #         points_line.append(points_line_i)
    #         points_line_i = []
    #         points_line_i.append(ele[2:4])
    #     elif before ==0:
    #         frame.append(ele[:2])
    #         points_line_i.append(ele[2:4])

    # temp_point =np.array([tmp[i:i + step] for line in range(0, len(tmp), step)], dtype=float)


# def convert2cloud(temp_point):
#     temp_point = np.array(temp_point)
#     x = temp_point[:, 0].reshape(-1)
#     y = temp_point[:, 1].reshape(-1)
#     z = temp_point[:, 2].reshape(-1)
#     cloud = np.stack((x, y, z))
#     header = Header()
#     header.stamp = rospy.Time().now()
#     header.frame_id = "divider"
#     pointclouds = create_cloud_xyz(header, cloud.T)





    # frame, points_line, headings = read_points_per_frame()
    # # aa = read_points_per_frame()
    # #print(type(aa))
    # np.save(np_data_path, [frame, points_line, headings])
    # print("saved")
    # frame, points_line, headings = np.load(np_data_path)
    # for i,ele in enumerate(points_line):
    #     print(np.shape(ele))
    # for ii,eele in  enumerate(ele):
    #     print(ii)
    #     convert2cloud(eele)
    # print(type(frame), type(points_line))
    # (frame, points_line) = l
    # trajactor, points_line, headings = np.load(np_data_path)
    # with open(test_path, 'w') as fb:
    #      print >> fb, 'trajactor'
    #      print >> fb, np.shape(trajactor),type(trajactor)
    #      print >> fb, np.shape(trajactor[0]),type(trajactor[0])
    #      print >> fb, trajactor
    #      print >> fb, 'points_line'
    #      print >> fb, np.shape(points_line), type(points_line)
    #      print >> fb, np.shape(points_line[0]), type(points_line[0])
    #      print >> fb, points_line
    #      print >> fb, 'headings'
    #      print >> fb, np.shape(headings), type(headings)
    #      print >> fb, np.shape(headings[0]), type(headings[0])
    #      print >> fb, headings

'''
def read_gps_all(path=gps_test_path):
    points = []
    headings = []
    count = 0

    for f in glob.glob(path):  # glob.glob(psth+"/gps_store/*.txt"):
        # if count ==2:
        #    break
        count += 1
        # print(count)
        # if count ==2:
        #    break
        tmp = np.loadtxt(f, delimiter=',')
        points_line = []
        headings_line = []
        for ele in tmp:
            points_line.append(ele[:3])
            headings_line.append(ele[3])
        #    print('points_line',points_line)
        #    print('headings_line', headings_line)
        # print('ele',np.shape(ele))
        # print('np.shape(headings_line)', np.shape(headings_line))
        points.append(np.array(points_line))
        headings.append(np.array(headings_line))

        # print('np.shape(headings)', np.shape(headings))

    return np.array(points), np.array(headings)

'''
'''


# ROS includes
import roslib
import rospy

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()


import rviz_tools_py.readpb as readpb
import rviz_tools_py.rviz_tools as rviz_tools

import sys
sys.path.append("..")
# import hd_map.readpb as readpb

# Initialize the ROS Node
rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()

# rospy.on_shutdown(cleanup_node)


markers_divider = rviz_tools.RvizMarkers('/map', 'markers_divider')
#message = readpb.read_from_pb_debug()
#points = readpb.read_divider(message)
points = readpb.read_divider_txt(file_path = "/home/lfg/my_work/deecamp/lfg/HDMapProject/rviz_ws/src/rviz_tools_py/data/divider_hdmap.txt") # read_hd_txt()
# print(points)
markers_gps = rviz_tools.RvizMarkers('/map', 'markers_gps')
local_point, headings = readpb.read_gps_all(path = '/home/lfg/my_work/deecamp/lfg/HDMapProject/rviz_ws/src/rviz_tools_py/data/GPS.txt')








def getRandomColor():
    # result = ColorRGBA()
    a = 1
    while True:
        r = random.random()  # random float from 0 to 1
        g = random.random()
        b = random.random()
        if ((r + g + b) > 1.5):  # 0=black, 3=white
            break
    return [r,g, b, a]

color = 'none'

# rviz_tools.genPublishArrow_simple(local_point,headings,markers, color)
# rviz_tools.genPublishPath_simple(points, markers)


# rviz_tools.genPublishPath_simple(points, markers)

rospy.AsyncSpinner spinner(4); # Use 4 threads
spinner.start();

while not rospy.is_shutdown():

    # Axis:

    # # Publish an axis using a numpy transform matrix
    # T = transformations.translation_matrix((1,0,0))
    # axis_length = 0.4
    # axis_radius = 0.05
    # markers.publishAxis(T, axis_length, axis_radius, 5.0) # pose, axis length, radius, lifetime

    # # Publish an axis using a ROS Pose Msg
    # P = Pose(Point(2,0,0),Quaternion(0,0,0,1))
    # axis_length = 0.4
    # axis_radius = 0.05
    # markers.publishAxis(P, axis_length, axis_radius, 5.0) # pose, axis length, radius, lifetime

    # 单个地画marker
    # 画车道线
    # rviz_tools.genPublishPath_simple(points, markers)
    # 画GPS
    rviz_tools.genPublishArrow_simple(local_point,headings,markers_gps, color)

    # 使用markerArray画一系列的marker
    # 画车道线
    rviz_tools.genPublishPathArray_simple(points, markers_divider, lifetime = None, toArray = True)
    # 画GPS
    # rviz_tools.genPublishArrowArray_simple(local_point,headings,markers_gps, color, lifetime = None, toArray = True)


    # Path:

    # Publish a path using a list of ROS Point Msgs
    # path = []
    # path.append( Point(0,-0.5,0) )
    # path.append( Point(1,-0.5,0) )
    # path.append( Point(1.5,-0.2,0) )
    # path.append( Point(2,-0.5,0) )
    # path.append( Point(2.5,-0.2,0) )
    # path.append( Point(3,-0.5,0) )
    # path.append( Point(4,-0.5,0) )
    # width = 0.02
    # markers.publishPath(path, 'orange', width, 5.0) # path, color, width, lifetime
    # rospy.spin()
    rospy.Rate(10).sleep() #1 Hz


'''
