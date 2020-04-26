#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python includeswhich
import numpy as np
import random

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


markers_divider = rviz_tools.RvizMarkers('/map', 'markers_divider')
#message = readpb.read_from_pb_debug()
#points = readpb.read_divider(message)
points = readpb.read_divider_txt(file_path = 
    '/home/apple/catkin_ws/src/rviz_tools_py/data/divider_hdmap.txt') # read_hd_txt()
# print(points)
markers_gps = rviz_tools.RvizMarkers('/map', 'markers_gps')
local_point, headings = readpb.read_gps_all(path = '/home/apple/catkin_ws/src/rviz_tools_py/data')
gps_test_path = '/home/apple/catkin_ws/src/rviz_tools_py/data/GPStrafficLine.txt'
np_data_path = '/home/apple/catkin_ws/src/rviz_tools_py/data/gps_pre_frame2.npy'


frame, points_line, headings = np.load(np_data_path)


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

markers = rviz_tools.RvizMarkers('/map', 'vv_marker')

count = 0
_len = len(points_line)

while not rospy.is_shutdown():

    T = transformations.translation_matrix((3,1,0))
    scale = Vector3(1.5,1.5,1.5)
    mesh_file1 = "package://rviz_tools_py/meshes/box_mesh.stl"
    markers.publishMesh(T, mesh_file1, 'lime_green', scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime


    # Display collada model with original texture (no coloring)
    P = Pose(Point(5,1,0),Quaternion(0,0,0,1))
    mesh_file3 = "package://rviz_tools_py/meshes/car_stl.stl"
    mesh_scale = 4.0
    markers.publishMesh(P, mesh_file3, None, mesh_scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime



    # rviz_tools.genPublishPathArray_simple(points, markers_divider, lifetime = 5.0, toArray = True)
    # rviz_tools.convert2cloud(points_line[count])
    # rviz_tools.genPublishArrowArray_simple(local_point,headings,markers_gps, color, lifetime = 5, toArray = True)
    # count+=1
    # count = count%_len
    rospy.Rate(1).sleep() #1 Hz

    # Display collada model with original texture (no coloring)
    # P = Pose(Point(5,1,0),Quaternion(0,0,0,1))
    # mesh_file3 = "package://rviz_tools_py/meshes/car.dae"
    # mesh_scale = 4.0
    # markers.publishMesh(P, mesh_file3, None, mesh_scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime
    

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
    #rviz_tools.genPublishArrow_simple(local_point,headings,markers_gps, color)

    # 使用markerArray画一系列的marker
    # 画车道线
    # rviz_tools.genPublishPathArray_simple(points, markers_divider, lifetime = None, toArray = True)
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