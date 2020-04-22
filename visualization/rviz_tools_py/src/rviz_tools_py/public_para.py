#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from geometry_msgs.msg import Vector3


class Public_para():

    path_prefix = '/home/apple/catkin_ws/src/rviz_tools_py/new_data/'

    # parameters for static node: map elemetents
    '''
    TODO : 
    1. ADD the traffic light and lane_markers(OK)
    2. discrimenate different type of lanes
    '''
    lane_markers = path_prefix + 'lane_markers.npy'
    static_lane_topic = 'markers_lanemarker'
    static_light_topic = 'marker_light'
    one_scence = '/home/apple/catkin_ws/src/rviz_tools_py/data/gps_pre_frame2.npy'
    divider_txt = '/home/apple/catkin_ws/src/rviz_tools_py/data/divider_hdmap.txt'
    package_name = 'rviz_tools_py'
    static_pub_rate = 1
    light_color = 'green'
    wait_time = None
    static_node_name = 'static_marker'
    static_frame = '/map'
    static_divider_topic = 'markers_divider'
    divider_color = 'white'
    lane_marker_color = 'blue'
    divider_ArrayTopic = 'path_Array'
    lanemarker_ArrayTopic = 'lanemarker_Array'
    lights_ArrayTopic = 'light_Array'
    static_refresh_time = 5.0
    color_point_time = 15.
    divider_width = 0.1
    light_sacale = 1.
    color_point_scale = 0.4
    lanemarker_width = 0.2
    light_gps = np.array([[590.6837504, 143.13797475,   0.],
                            [542.30352281, 149.59286593,   0.],
                            [590.68375040404, 143.13797475258, 0],
                            [589.12907013251, 145.13612234499, 0],
                            [592.23843195103, 141.13872094033, 0],
                            [542.30352280661, 149.59286592947, 0],
                            [540.18000715273, 152.23502500495, 0],
                            [544.42704070266, 146.94960141322, 0]])

    '''
    parameters for dynamic node, elements indlude:
    point cloud, car model, trajector GPS and TF transformation
    TODO :
    1. solve the synchronization the problem in ROS system
    2. add the WASD control to change the view
    3. set the default view window via code ()
    4. Add beautiful 3D model into Rviz and build the enviroments
    '''
    # download_prefix = '/media/psf/Home/Downloads'
    gps_test_path = path_prefix+'lanes2ENU/0.txt'
    np_data_path = path_prefix+'lanes2ENU/0.npy'
    next_data_path = path_prefix + 'lanes2ENU/'
    plus_data_path = path_prefix+'plus/'
    scence_id = '20190126102613_291d28cfb93e6d131ab4a8be7d1a1da6_4'
    color_point_topic = 'color_point'
    colorfulArray = 'colorfulArray'

    test_path = '/home/apple/catkin_ws/test_bug.txt'
    car_stl_model = "package://rviz_tools_py/meshes/car_stl.stl"
    dynamic_node_name = 'dynamic_marker'
    car_arrow_topic = 'car_and_heading'
    dynamic_base_frame = '/vehicle'
    dynamic_single_marker_topic = 'single_topic'
    point_cloud_topic = 'point_cloud'
    arrow_color = 'orange'
    arrow_lifetime = 2.0
    car_color = None
    dynamic_pub_rate = 30
    dynamic_refresh_time = 1./dynamic_pub_rate
    dynamic_refresh_time_slow = dynamic_refresh_time/10

    # scale is for arrow
    scale = Vector3(5.0, 0.2, 0.2)  # x=length, y=height, z=height
    # mesh_scale is for car model
    mesh_scale = 0.3

    # addtional path for plus data
    scence_id_txt_path = '/home/apple/catkin_ws/src/rviz_tools_py/src/rviz_tools_py/scence_id_all.txt'

