#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tf
import numpy as np
import rospy
import roslib
from rviz_tools_py.public_para import Public_para as _para
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import rviz_tools_py.readpb as readpb
import rviz_tools_py.rviz_tools as rviz_tools
roslib.load_manifest(_para.package_name)

# init the node for publishing
rospy.init_node(_para.dynamic_node_name, anonymous=False,
                log_level=rospy.INFO, disable_signals=False)

# init the broadcaster for TF transformation
br = tf.TransformBroadcaster()  # 初始化一次

# define the constant and parameters
rate = rospy.Rate(_para.dynamic_pub_rate)
refresh_time = _para.dynamic_refresh_time
arrow_color = _para.arrow_color
car_model = _para.car_stl_model

# define the basements
base_frame = _para.static_frame
maker_topic = _para.dynamic_single_marker_topic

# init the ArrayMarkers and RvzMarker class
car_and_heading = rviz_tools.ArrayMarkers(
    _para.car_arrow_topic, wait_time=_para.wait_time)
arrow_maker = rviz_tools.RvizMarkers(base_frame, maker_topic)
car_marker = rviz_tools.RvizMarkers(base_frame, maker_topic)

# path and load data
# type(points_line(count)) -> list, 从np.load的第一级都是ndarray
trajactor, points_line, headings = np.load(_para.np_data_path, allow_pickle=True)# np.load(path, allow_pickle=True)
plus_data_path = _para.plus_data_path
# blue_point = np.load(plus_data_path+'divider.npy')
# red_point = np.load(plus_data_path, 'divider2.npy')




# count 作为计数，转换不同的GPS轨迹点trajactor[count]
count = 0
_len = len(trajactor)

s_count = 0

file_count = 73

# define the auxiliary functions
def load_next_data():
    global trajactor, points_line, headings, file_count
    if file_count == 153:
        file_count = 0
    next_data_path = _para.next_data_path+str(file_count)+'.npy'
    trajactor, points_line, headings = np.load(next_data_path, allow_pickle=True)
    file_count+=1

while not rospy.is_shutdown():

    # 发布tf坐标变换关系
    # print(np.shape(trajactor))
    # print(count)
    print('file_count:  ',file_count)
    current_point = (trajactor[count][0],
                     trajactor[count][1], trajactor[count][2])
    current_heading = headings[count]
    car_P = rviz_tools.getTMatrix(current_point, current_heading)
    car_heading = rviz_tools.getHeading(current_heading)
    br.sendTransform(current_point, car_heading,
                     rospy.Time.now(), _para.dynamic_base_frame, _para.static_frame)

    # 可选，使用slow_down完成同步功能
    # slow_down(_para.dynamic_refresh_time_slow)

    # 发布当前的箭头
    _arrow_maker = arrow_maker.publishArrow(
        car_P, arrow_color, _para.scale, base_frame, in_id=count, lifetime=_para.arrow_lifetime, toArray=True)
    # 发布当前的car_model
    # pose, car_model_name, color, mesh_scale, lifetime
    _car_marker = car_marker.publishMesh(
        car_P, car_model, _para.car_color, _para.mesh_scale, base_frame, lifetime=refresh_time, toArray=True)

    # 合并为MarkerArray发布
    car_and_heading.clean()
    car_and_heading.addMarkers(_arrow_maker)
    car_and_heading.addMarkers(_car_marker)
    car_and_heading.pubulishMarkerArrow()

    # 发布当前的点云
    rviz_tools.convert2cloud2(
        points_line[count], base_frame, _para.point_cloud_topic)

    count += 1
    if count == _len:
        load_next_data()
        _len = len(trajactor)
        count = 0
    rate.sleep()


# import sys
# sys.path.append("..")
# if __name__ == '__main__':

# rviz_tools.genPublishPathArray_simple(points, markers_divider, lifetime = 5.0, toArray = True)
# rviz_tools.convert2cloud(points_line[count])
# rviz_tools.genPublishArrowArray_simple(local_point,headings,markers_gps, color, lifetime = 5, toArray = True)



# def slow_down(refresh_time):
#     global s_count
#     if s_count == 10:
#         # 发布当前的箭头
#         _arrow_maker = arrow_maker.publishArrow(
#             car_P, arrow_color, _para.scale, in_id=count, lifetime=refresh_time, toArray=True)
#         # 发布当前的car_model
#         # pose, car_model_name, color, mesh_scale, lifetime
#         _car_marker = car_marker.publishMesh(
#             car_P, car_model, _para.car_color, _para.mesh_scale, lifetime=refresh_time, toArray=True)

#         # 合并为MarkerArray发布
#         car_and_heading.clean()
#         car_and_heading.addMarkers(_arrow_maker)
#         car_and_heading.addMarkers(_car_marker)
#         car_and_heading.pubulishMarkerArrow()
#         print("-----------Publishing!")
#         s_count = 0
#     else:
#         print("Waiting--------------", s_count)
#         s_count+=1
