#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python includeswhich
import numpy
import random

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()

import hd_map.rviz_tools as rviz_tools

from hd_map.readpb import Divider
from hd_map.readpb import read_from_pb, read_divider

import sys
sys.path.append("..")
import hd_map.readpb as readpb
from hd_map.


# Initialize the ROS Node
rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()

# rospy.on_shutdown(cleanup_node)



markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')
message = readpb.read_from_pb_debug()

points = readpb.read_divider(message)
print(points)

while not rospy.is_shutdown():

    # Axis:

    # Publish an axis using a numpy transform matrix
    T = transformations.translation_matrix((1,0,0))
    axis_length = 0.4
    axis_radius = 0.05
    markers.publishAxis(T, axis_length, axis_radius, 5.0) # pose, axis length, radius, lifetime

    # Publish an axis using a ROS Pose Msg
    P = Pose(Point(2,0,0),Quaternion(0,0,0,1))
    axis_length = 0.4
    axis_radius = 0.05
    markers.publishAxis(P, axis_length, axis_radius, 5.0) # pose, axis length, radius, lifetime

    rviz_tools.genPublishPath_simple(points, markers)


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
    
    rospy.Rate(1).sleep() #1 Hz



if __name__ == '__main__':
    rospy.init_node('hdmap', anonymous=True)

    message = HDMap()
    with open('/home/lfg/my_work/deecamp/roshd/src/hd_map/hdmap_deecamp.pb', 'rb') as fb:
        pb_content = fb.read()  # 如果文件很大，则要分批读取
        # pb_content 是二进制的pb数据，如果是文件，则需要用 open 方法读取数据
        message.ParseFromString(pb_content)
    pub_rate = 1
    hdmap_pub(message, pub_rate)