#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: David Butterworth <dbworth@cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
This is a demo of Rviz Tools for python which tests all of the
available functions by publishing lots of Markers in Rviz.
"""

# Python includes
import numpy
import random

import math
import tf


# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()

import rviz_tools_py.rviz_tools as rviz_tools


# Initialize the ROS Node
rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)


markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')


def getTMatrix(point, heading):
    zaxis = (0,0,1)
    # print(heading)
    try:
        heading = math.radians(-heading+90)
    except TypeError:
        print('error!')
        # print('numpy.shape(heading)', numpy.shape(heading))

        return None

    Rx = tf.transformations.rotation_matrix(heading, zaxis)
    #print("heading:",heading)
    #print('Rx:', Rx)
    Tm = tf.transformations.translation_matrix(point)
    #print("Tm",Tm)
    T = tf.transformations.concatenate_matrices(Tm, Rx)
    #print(T)
    return T

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


    # Line:

    # Publish a line between two ROS Point Msgs
    point1 = Point(-2,1,0)
    point2 = Point(2,1,0) 
    width = 0.05
    markers.publishLine(point1, point2, 'green', width, 5.0) # point1, point2, color, width, lifetime

    # Publish a line between two ROS Poses
    P1 = Pose(Point(-2,1.1,0),Quaternion(0,0,0,1))
    P2 = Pose(Point(2,1.1,0),Quaternion(0,0,0,1))
    width = 0.02
    markers.publishLine(P1, P2, 'red', width, 5.0) # point1, point2, color, width, lifetime

    # Publish a line between two numpy transform matrices
    T1 = transformations.translation_matrix((-2,1.2,0))
    T2 = transformations.translation_matrix((2,1.2,0))
    width = 0.02
    markers.publishLine(T1, T2, 'blue', width, 5.0) # point1, point2, color, width, lifetime


    # Path:

    # Publish a path using a list of ROS Point Msgs
    path = []
    path.append( Point(0,-0.5,0) )
    path.append( Point(1,-0.5,0) )
    path.append( Point(1.5,-0.2,0) )
    path.append( Point(2,-0.5,0) )
    path.append( Point(2.5,-0.2,0) )
    path.append( Point(3,-0.5,0) )
    path.append( Point(4,-0.5,0) )
    width = 0.02
    markers.publishPath(path, 'orange', width, 5.0) # path, color, width, lifetime


    # Plane / Rectangle:

    # Publish a rectangle between two points (thin, planar surface)
    # If the z-values are different, this will produce a cuboid
    point1 = Point(-1,0,0)
    point2 = Point(-2,-1,0) 
    markers.publishRectangle(point1, point2, 'blue', 5.0)

    # Publish a rotated plane using a numpy transform matrix
    R_y = transformations.rotation_matrix(0.3, (0,1,0)) # Rotate around y-axis by 0.3 radians
    T0 = transformations.translation_matrix((-3,-1.5,0))
    T = transformations.concatenate_matrices(T0, R_y)
    depth = 1.1
    width = 1.5
    markers.publishPlane(T, depth, width, 'purple', 5.0) # pose, depth, width, color, lifetime

    # Publish a plane using a ROS Pose Msg
    P = Pose(Point(-3,0,0),Quaternion(0,0,0,1))
    depth = 1.3
    width = 1.3
    markers.publishPlane(P, depth, width, 'brown', 5.0) # pose, depth, width, color, lifetime


    # Polygon:

    # Publish a polygon using a ROS Polygon Msg
    polygon = Polygon()
    polygon.points.append( Point(0.0,-1.0,0.0) )
    polygon.points.append( Point(0.0,-2.0,0.0) )
    polygon.points.append( Point(-1.0,-2.0,0.0) )
    polygon.points.append( Point(-1.0,-1.0,0.0) )
    markers.publishPolygon(polygon, 'red', 0.02, 5.0) # path, color, width, lifetime


    # Text:

    # Publish some text using a ROS Pose Msg
    P = Pose(Point(3,-1,0),Quaternion(0,0,0,1))
    scale = Vector3(0.2,0.2,0.2)
    markers.publishText(P, 'This is some text', 'white', scale, 5.0) # pose, text, color, scale, lifetime


    # Arrow:
    __points = [1,-2,0]
    __headings = 45
    TT = getTMatrix(__points, __headings)
    scale = Vector3(1.0,0.2,0.2) # x=length, y=height, z=height
    # T2 = transformations.translation_matrix((3,-2,0))
    markers.publishArrow(TT, 'blue', scale, 5.0) # pose, color, scale, lifetime



    # Publish an arrow using a numpy transform matrix
    T = transformations.translation_matrix((1,-2,0))
    scale = Vector3(1.0,0.2,0.2) # x=length, y=height, z=height
    # T2 = transformations.translation_matrix((3,-2,0))
    markers.publishArrow(T, 'blue', scale, 5.0) # pose, color, scale, lifetime

    # Publish an arrow using a ROS Pose Msg
    P = Pose(Point(1,-3,0),Quaternion(0,0,0,1))
    arrow_length = 2.0 # single value for length (height is relative)
    markers.publishArrow(P, 'pink', arrow_length, 5.0) # pose, color, arrow_length, lifetime


    # Cube / Cuboid:

    # Publish a cube using a numpy transform matrix
    T = transformations.translation_matrix((-3,2.2,0))
    cube_width = 0.5 # cube is 0.5x0.5x0.5
    markers.publishCube(T, 'green', cube_width, 5.0) # pose, color, cube_width, lifetime

    # Publish a cube using a ROS Pose Msg
    P = Pose(Point(-2,2.2,0),Quaternion(0,0,0,1))
    cube_width = 0.6
    markers.publishCube(P, 'blue', cube_width, 5.0) # pose, color, cube_width, lifetime

    # Publish a cube using wrapper function publishBlock()
    P = Pose(Point(-1,2.2,0),Quaternion(0,0,0,1))
    cube_width = 0.7
    markers.publishBlock(P, 'orange', cube_width, 5.0) # pose, color, cube_width, lifetime

    # Publish a cuboid using a numpy transform matrix
    T = transformations.translation_matrix((0.6,2.2,0))
    scale = Vector3(1.5,0.2,0.2)
    markers.publishCube(T, 'yellow', scale, 5.0) # pose, color, scale, lifetime

    # Publish a cuboid using a ROS Pose Msg
    P = Pose(Point(2.2,2.2,0),Quaternion(0,0,0,1))
    scale = Vector3(1.1,0.2,0.8)
    markers.publishCube(P, 'brown', scale, 5.0) # pose, color, scale, lifetime


    # List of cubes:

    # Publish a set of cubes using a list of ROS Point Msgs
    points = []
    z_height = 0.1
    points.append(Point(3.5+0*0.2, 0.5, z_height)) # row 1
    points.append(Point(3.5+1*0.2, 0.5, z_height))
    points.append(Point(3.5+2*0.2, 0.5, z_height))
    points.append(Point(3.5+0*0.2, 0.5+1*0.2, z_height)) # row 2
    points.append(Point(3.5+1*0.2, 0.5+1*0.2, z_height))
    points.append(Point(3.5+2*0.2, 0.5+1*0.2, z_height))
    points.append(Point(3.5+0*0.2, 0.5+2*0.2, z_height)) # row 3
    points.append(Point(3.5+1*0.2, 0.5+2*0.2, z_height))
    points.append(Point(3.5+2*0.2, 0.5+2*0.2, z_height))
    points.append(Point(3.5+0*0.2, 0.5+2*0.2, z_height+0.2)) # 2nd layer
    diameter = 0.2-0.005
    markers.publishCubes(points, 'red', diameter, 5.0) # path, color, diameter, lifetime


    # Sphere:

    # Publish a sphere using a numpy transform matrix
    T = transformations.translation_matrix((-3,3.2,0))
    scale = Vector3(0.5,0.5,0.5) # diameter
    color = [0,1,0] # list of RGB values (green)
    markers.publishSphere(T, color, scale, 5.0) # pose, color, scale, lifetime

    # Publish a sphere using a ROS Pose
    P = Pose(Point(-2,3.2,0),Quaternion(0,0,0,1))
    scale = Vector3(0.6,0.6,0.6) # diameter
    color = (0,0,1) # tuple of RGB values (blue)
    markers.publishSphere(P, color, scale, 5.0) # pose, color, scale, lifetime

    # Publish a sphere using a ROS Point
    point = Point(-1,3.2,0)
    scale = Vector3(0.7,0.7,0.7) # diameter
    color = 'orange'
    markers.publishSphere(point, color, scale, 5.0) # pose, color, scale, lifetime

    # Publish a sphere by passing diameter as a float
    point = Point(0,3.2,0)
    diameter = 0.8
    markers.publishSphere(point, 'yellow', diameter, 5.0) # pose, color, diameter, lifetime

    # Publish a sphere with higher render quality (this is one sphere in a SPHERE_LIST)
    point = Point(1,3.2,0)
    diameter = 0.9
    markers.publishSphere2(point, 'brown', diameter, 5.0) # pose, color, scale, lifetime


    # List of spheres:

    # Publish a set of spheres using a list of ROS Point Msgs
    points = []
    points.append( Point(-3,4,0) )
    points.append( Point(-2,4,0) )
    points.append( Point(-1,4,0) )
    points.append( Point(0,4,0) )
    diameter = 0.3
    markers.publishSpheres(points, 'white', diameter, 5.0) # path, color, diameter, lifetime
    
    # Publish a set of spheres using a list of ROS Pose Msgs
    poses = []
    poses.append( Pose(Point(1,4,0),Quaternion(0,0,0,1)) )
    poses.append( Pose(Point(2,4,0),Quaternion(0,0,0,1)) )
    poses.append( Pose(Point(3,4,0),Quaternion(0,0,0,1)) )
    scale = Vector3(0.5,0.5,0.5) # diameter
    markers.publishSpheres(poses, 'blue', scale, 5.0) # path, color, scale, lifetime

    # Publish a set of spheres using a list of numpy transform matrices
    poses = []
    poses.append( Pose(Point(4,4,0),Quaternion(0,0,0,1)) )
    poses.append( Pose(Point(5,4,0),Quaternion(0,0,0,1)) )
    diameter = 0.6
    markers.publishSpheres(poses, 'green', diameter, 5.0) # path, color, scale, lifetime


    # Cylinder:

    # Publish a cylinder using a numpy transform matrix
    T = transformations.translation_matrix((-3,5,0))
    markers.publishCylinder(T, 'green', 1.0, 0.5, 5.0) # pose, color, height, radius, lifetime

    # Publish a cylinder using a ROS Pose
    P = Pose(Point(-2,5,0),Quaternion(0,0,0,1))
    markers.publishCylinder(P, 'blue', 1.0, 0.5, 5.0) # pose, color, height, radius, lifetime


    # Publish a cylinder of a random color (method #1)
    P = Pose(Point(-1,5,0),Quaternion(0,0,0,1))
    markers.publishCylinder(P, markers.getRandomColor(), 1.0, 0.5, 5.0) # pose, color, height, radius, lifetime

    # Publish a cylinder of a random color (method #2)
    P = Pose(Point(0,5,0),Quaternion(0,0,0,1))
    markers.publishCylinder(P, 'random', 1.0, 0.5, 5.0) # pose, color, height, radius, lifetime


    # Model mesh:

    # Publish STL mesh of box, colored green
    T = transformations.translation_matrix((3,1,0))
    scale = Vector3(1.5,1.5,1.5)
    mesh_file1 = "package://rviz_tools_py/meshes/box_mesh.stl"
    markers.publishMesh(T, mesh_file1, 'lime_green', scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime

    # Display STL mesh of bottle, re-scaled to smaller size
    P = Pose(Point(4,1,0),Quaternion(0,0,0,1))
    scale = Vector3(0.6,0.6,0.6)
    mesh_file2 = "package://rviz_tools_py/meshes/fuze_bottle_collision.stl"
    markers.publishMesh(P, mesh_file2, 'blue', scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime

    # Display collada model with original texture (no coloring)
    P = Pose(Point(5,1,0),Quaternion(0,0,0,1))
    mesh_file3 = "package://rviz_tools_py/meshes/fuze_bottle_visual.dae"
    mesh_scale = Vector3(0.6,0.6,0.6)
    markers.publishMesh(P, mesh_file3, None, mesh_scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime

    P = Pose(Point(10,10,0),Quaternion(0,0,0,1))
    # mesh_file3 = "/home/apple/catkin_ws/src/rviz_tools_py/meshes/new_car.dae"
    mesh_file3 = "file://home/apple/catkin_ws/src/rviz_tools_py/meshes/car_stl.stl"
    mesh_scale = Vector3(0.6,0.6,0.6)
    markers.publishMesh(P, mesh_file3, None, mesh_scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime


    rospy.Rate(1).sleep() #1 Hz
