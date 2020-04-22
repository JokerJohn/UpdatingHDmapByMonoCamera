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


# Python includes
import numpy as np
import random  # randint
import math

# ROS includes
import roslib
import rospy
import tf  # tf/transformations.py
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Point32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Polygon
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class ArrayMarkers(object):
    def __init__(self, marker_topic, wait_time=None):
        # self.base_frame = base_frame
        self.marker_topic = marker_topic
        self.pubulisherArray = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1)
        self.inerMarker = MarkerArray()
        self.in_id = 0

    def addMarkers(self, marker):

        self.inerMarker.markers.append(marker)
        self.in_id += 1

    def pubulishMarkerArrow(self):

        self.pubulisherArray.publish(self.inerMarker)
        # rospy.Publisher(self.base_frame, MarkerArray, queue_size=1)

        return True
    def clean(self):
        self.inerMarker.markers = []


class RvizMarkers(object):
    """
    A class for publishing markers in Rviz
    """

    def __init__(self, base_frame, marker_topic, wait_time=None):
        self.base_frame = base_frame
        self.marker_topic = marker_topic

        # Set the default Marker parameters
        self.setDefaultMarkerParams()

        # Create the Rviz Marker Publisher
        self.loadMarkerPublisher(wait_time)

    def setDefaultMarkerParams(self):
        """
        Set the default parameters for each type of Rviz Marker
        """

        self.marker_lifetime = rospy.Duration(0.0)  # 0 = Marker never expires
        self.muted = False
        self.alpha = 1.0

        # Set default parameters for Cylinder Marker
        self.cylinder_marker = Marker()
        self.cylinder_marker.header.frame_id = self.base_frame
        self.cylinder_marker.ns = "Cylinder"  # unique ID
        self.cylinder_marker.action = Marker().ADD
        self.cylinder_marker.type = Marker().CYLINDER
        self.cylinder_marker.lifetime = self.marker_lifetime

        # Reset Marker
        self.reset_marker = Marker()
        self.reset_marker.header.frame_id = self.base_frame
        self.reset_marker.header.stamp = rospy.Time()
        self.reset_marker.action = 3

        # Arrow Marker
        # self.arrow_marker = Marker()
        # self.arrow_marker.header.frame_id = self.base_frame
        # self.arrow_marker.ns = "Arrow"  # unique ID
        # self.arrow_marker.action = Marker().ADD
        # self.arrow_marker.type = Marker().ARROW
        # self.arrow_marker.lifetime = self.marker_lifetime

        # Rectangle Marker
        self.rectangle_marker = Marker()
        self.rectangle_marker.header.frame_id = self.base_frame
        self.rectangle_marker.ns = "Rectangle"  # unique ID
        self.rectangle_marker.action = Marker().ADD
        self.rectangle_marker.type = Marker().CUBE
        self.rectangle_marker.lifetime = self.marker_lifetime

        # Line Marker
        self.line_marker = Marker()
        self.line_marker.header.frame_id = self.base_frame
        self.line_marker.ns = "Line"  # unique ID
        self.line_marker.action = Marker().ADD
        self.line_marker.type = Marker().LINE_STRIP
        self.line_marker.lifetime = self.marker_lifetime

        # Path Marker (Line List)
        # self.path_marker = Marker()
        # self.path_marker.header.frame_id = self.base_frame
        # self.path_marker.ns = "Path"  # unique ID
        # self.path_marker.action = Marker().ADD
        # self.path_marker.type = Marker().LINE_LIST
        # self.path_marker.lifetime = self.marker_lifetime
        # self.path_marker.pose.position.x = 0.0
        # self.path_marker.pose.position.y = 0.0
        # self.path_marker.pose.position.z = 0.0
        # self.path_marker.pose.orientation.x = 0.0
        # self.path_marker.pose.orientation.y = 0.0
        # self.path_marker.pose.orientation.z = 0.0
        # self.path_marker.pose.orientation.w = 1.0

        # Sphere Marker (A single sphere)
        # This renders a low-quality sphere
        # self.sphere_marker = Marker()
        # self.sphere_marker.header.frame_id = self.base_frame
        # self.sphere_marker.ns = "Sphere"  # unique ID
        # self.sphere_marker.type = Marker().SPHERE
        # self.sphere_marker.action = Marker().ADD
        # self.sphere_marker.lifetime = self.marker_lifetime
        # self.sphere_marker.pose.position.x = 0
        # self.sphere_marker.pose.position.y = 0
        # self.sphere_marker.pose.position.z = 0
        # self.sphere_marker.pose.orientation.x = 0.0
        # self.sphere_marker.pose.orientation.y = 0.0
        # self.sphere_marker.pose.orientation.z = 0.0
        # self.sphere_marker.pose.orientation.w = 1.0

        # Sphere Marker #2 (A single sphere)
        # A Sphere List with one sphere, this renders a
        # higher-quality sphere than the method above
        self.sphere_marker2 = Marker()
        self.sphere_marker2.header.frame_id = self.base_frame
        self.sphere_marker2.ns = "Sphere"  # unique ID
        self.sphere_marker2.type = Marker().SPHERE_LIST
        self.sphere_marker2.action = Marker().ADD
        self.sphere_marker2.lifetime = self.marker_lifetime
        self.sphere_marker2.pose.position.x = 0
        self.sphere_marker2.pose.position.y = 0
        self.sphere_marker2.pose.position.z = 0
        self.sphere_marker2.pose.orientation.x = 0.0
        self.sphere_marker2.pose.orientation.y = 0.0
        self.sphere_marker2.pose.orientation.z = 0.0
        self.sphere_marker2.pose.orientation.w = 1.0
        point1 = Point()
        self.sphere_marker2.points.append(point1)
        self.sphere_marker2.colors.append(self.getColor('blue'))

        # Spheres List (Multiple spheres)
        self.spheres_marker = Marker()
        self.spheres_marker.header.frame_id = self.base_frame
        self.spheres_marker.ns = "Spheres"  # unique ID
        self.spheres_marker.type = Marker().SPHERE_LIST
        self.spheres_marker.action = Marker().ADD
        self.spheres_marker.lifetime = self.marker_lifetime
        self.spheres_marker.pose.position.x = 0.0
        self.spheres_marker.pose.position.y = 0.0
        self.spheres_marker.pose.position.z = 0.0
        self.spheres_marker.pose.orientation.x = 0.0
        self.spheres_marker.pose.orientation.y = 0.0
        self.spheres_marker.pose.orientation.z = 0.0
        self.spheres_marker.pose.orientation.w = 1.0

        # Cube Marker (Block or cuboid)
        self.cube_marker = Marker()
        self.cube_marker.header.frame_id = self.base_frame
        self.cube_marker.ns = "Block"  # unique ID
        self.cube_marker.action = Marker().ADD
        self.cube_marker.type = Marker().CUBE
        self.cube_marker.lifetime = self.marker_lifetime

        # Cubes List (Multiple cubes)
        self.cubes_marker = Marker()
        self.cubes_marker.header.frame_id = self.base_frame
        self.cubes_marker.ns = "Cubes"  # unique ID
        self.cubes_marker.type = Marker().CUBE_LIST
        self.cubes_marker.action = Marker().ADD
        self.cubes_marker.lifetime = self.marker_lifetime
        self.cubes_marker.pose.position.x = 0.0
        self.cubes_marker.pose.position.y = 0.0
        self.cubes_marker.pose.position.z = 0.0
        self.cubes_marker.pose.orientation.x = 0.0
        self.cubes_marker.pose.orientation.y = 0.0
        self.cubes_marker.pose.orientation.z = 0.0
        self.cubes_marker.pose.orientation.w = 1.0

        # Cylinder Marker
        self.cylinder_marker = Marker()
        self.cylinder_marker.header.frame_id = self.base_frame
        self.cylinder_marker.ns = "Cylinder"  # unique ID
        self.cylinder_marker.action = Marker().ADD
        self.cylinder_marker.type = Marker().CYLINDER
        self.cylinder_marker.lifetime = self.marker_lifetime

        # Mesh Marker
        self.mesh_marker = Marker()
        # self.mesh_marker.header.frame_id = self.base_frame
        self.mesh_marker.ns = "Mesh"  # unique ID
        self.mesh_marker.action = Marker().ADD
        self.mesh_marker.type = Marker().MESH_RESOURCE
        self.mesh_marker.lifetime = self.marker_lifetime

        # Text Marker
        self.text_marker = Marker()
        self.text_marker.header.frame_id = self.base_frame
        self.text_marker.ns = "Text"  # unique ID
        self.text_marker.action = Marker().ADD
        self.text_marker.type = Marker().TEXT_VIEW_FACING
        self.text_marker.lifetime = self.marker_lifetime

    def loadMarkerPublisher(self, wait_time=None):
        """
        Initialize the ROS Publisher.

        If wait_time != None, wait for specified number of
        seconds for a subscriber to connect.
        """

        # Check if the ROS Publisher has already been created
        if hasattr(self, 'pub_rviz_marker'):
            return

        # Create the Rviz Marker Publisher
        self.pub_rviz_marker = rospy.Publisher(
            self.marker_topic, Marker, queue_size=10)
        rospy.logdebug("Publishing Rviz markers on topic '%s'",
                       self.marker_topic)

        # Block for specified number of seconds,
        # or until there is 1 subscriber
        if wait_time != None:
            self.waitForSubscriber(self.pub_rviz_marker, wait_time)

    def waitForSubscriber(self, publisher, wait_time=1.0):
        """
        Wait until there is 1 subscriber to a ROS Publisher,
        or until some number of seconds have elapsed.
        """

        start_time = rospy.Time.now()
        max_time = start_time + rospy.Duration(wait_time)

        num_existing_subscribers = publisher.get_num_connections()

        while (num_existing_subscribers == 0):
            #print 'Number of subscribers: ', num_existing_subscribers
            rospy.Rate(100).sleep()

            if (rospy.Time.now() > max_time):
                rospy.logerr(
                    "No subscribers connected to the '%s' topic after %f seconds", self.marker_topic, wait_time)
                return False

            num_existing_subscribers = publisher.get_num_connections()

        return True

    def publishMarker(self, marker):
        """
        Publish a Marker Msg
        """

        if (self.muted == True):
            return True

        # Check ROS Publisher
        # self.loadMarkerPublisher()

        self.pub_rviz_marker.publish(marker)

        return True

    def deleteAllMarkers(self):
        """
        Publish a Msg to delete all Markers
        """

        return self.publishMarker(self.reset_marker)

    def getColor(self, color):
        """
        Convert a color name or RGB value to a ROS ColorRGBA type

        @param color name (string) or RGB color value (tuple or list)

        @return color (ColorRGBA)
        """

        result = ColorRGBA()
        result.a = self.alpha

        if (type(color) == tuple) or (type(color) == list):
            if len(color) == 3:
                result.r = color[0]
                result.g = color[1]
                result.b = color[2]
            elif len(color) == 4:
                result.r = color[0]
                result.g = color[1]
                result.b = color[2]
                result.a = color[3]
            else:
                raise ValueError(
                    'color must have 3 or 4 float values in getColor()')
        elif (color == 'red'):
            result.r = 0.8
            result.g = 0.1
            result.b = 0.1
        elif (color == 'green'):
            result.r = 0.1
            result.g = 0.8
            result.b = 0.1
        elif (color == 'blue'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.8
        elif (color == 'grey') or (color == 'gray'):
            result.r = 0.9
            result.g = 0.9
            result.b = 0.9
        elif (color == 'white'):
            result.r = 1.0
            result.g = 1.0
            result.b = 1.0
        elif (color == 'orange'):
            result.r = 1.0
            result.g = 0.5
            result.b = 0.0
        elif (color == 'translucent_light'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.1
            result.a = 0.1
        elif (color == 'translucent'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.1
            result.a = 0.25
        elif (color == 'translucent_dark'):
            result.r = 0.1
            result.g = 0.1
            result.b = 0.1
            result.a = 0.5
        elif (color == 'black'):
            result.r = 0.0
            result.g = 0.0
            result.b = 0.0
        elif (color == 'yellow'):
            result.r = 1.0
            result.g = 1.0
            result.b = 0.0
        elif (color == 'brown'):
            result.r = 0.597
            result.g = 0.296
            result.b = 0.0
        elif (color == 'pink'):
            result.r = 1.0
            result.g = 0.4
            result.b = 1
        elif (color == 'lime_green'):
            result.r = 0.6
            result.g = 1.0
            result.b = 0.2
        elif (color == 'clear'):
            result.r = 1.0
            result.g = 1.0
            result.b = 1.0
            result.a = 0.0
        elif (color == 'purple'):
            result.r = 0.597
            result.g = 0.0
            result.b = 0.597
        elif (color == 'random'):
            # Get a random color that is not too light
            while True:
                result.r = random.random()  # random float from 0 to 1
                result.g = random.random()
                result.b = random.random()
                if ((result.r + result.g + result.b) > 1.5):  # 0=black, 3=white
                    break
        else:
            rospy.logerr(
                "getColor() called with unknown color name '%s', defaulting to 'blue'", color)
            result.r = 0.1
            result.g = 0.1
            result.b = 0.8

        return result

    def getRandomColor(self):
        """
        Get a random color.

        @return color (ColorRGBA)
        """

        # Make a list of the color names to choose from
        all_colors = []
        all_colors.append('red')
        all_colors.append('green')
        all_colors.append('blue')
        all_colors.append('grey')
        all_colors.append('white')
        all_colors.append('orange')
        all_colors.append('yellow')
        all_colors.append('brown')
        all_colors.append('pink')
        all_colors.append('lime_green')
        all_colors.append('purple')

        # Chose a random color name
        rand_num = random.randint(0, len(all_colors) - 1)
        rand_color_name = all_colors[rand_num]

        return rand_color_name
    def getRandomColor2():
        # result = ColorRGBA()
        a = 1
        while True:
            r = random.random()  # random float from 0 to 1
            g = random.random()
            b = random.random()
            if ((r + g + b) > 1.5):  # 0=black, 3=white
                break
        return [r, g, b, a]

    def publishSphere(self, pose, color, scale, in_id = 0, lifetime=None, toArray = False):
        """
        Publish a sphere Marker. This renders 3D looking sphere.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """
        self.sphere_marker = Marker()
        self.sphere_marker.header.frame_id = self.base_frame
        self.sphere_marker.ns = "Sphere"  # unique ID
        self.sphere_marker.type = Marker().SPHERE
        self.sphere_marker.action = Marker().ADD
        self.sphere_marker.lifetime = self.marker_lifetime
        self.sphere_marker.pose.position.x = 0
        self.sphere_marker.pose.position.y = 0
        self.sphere_marker.pose.position.z = 0
        self.sphere_marker.pose.orientation.x = 0.0
        self.sphere_marker.pose.orientation.y = 0.0
        self.sphere_marker.pose.orientation.z = 0.0
        self.sphere_marker.pose.orientation.w = 1.0

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            sphere_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            sphere_pose = pose
        elif type(pose) == Point:
            pose_msg = Pose()
            pose_msg.position = pose
            sphere_pose = pose_msg
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishSphere()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            sphere_scale = scale
        elif type(scale) == float:
            sphere_scale = Vector3(scale, scale, scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishSphere()", type(scale).__name__)
            return False

        # Increment the ID number
        self.sphere_marker.id = in_id

        # Get the default parameters
        sphere_marker = self.sphere_marker

        if lifetime == None:
            sphere_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            sphere_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        sphere_marker.header.stamp = rospy.Time.now()

        # Set marker size
        sphere_marker.scale = sphere_scale

        # Set marker color
        sphere_marker.color = self.getColor(color)

        # Set the pose
        sphere_marker.pose = sphere_pose

        if toArray==True:
            return sphere_marker

        return self.publishMarker(sphere_marker)

    def publishSphere2(self, pose, color, scale, lifetime=None):
        """
        Publish a sphere Marker. This renders a smoother, flatter-looking sphere.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            sphere_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            sphere_pose = pose
        elif type(pose) == Point:
            pose_msg = Pose()
            pose_msg.position = pose
            sphere_pose = pose_msg
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishSphere()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            sphere_scale = scale
        elif type(scale) == float:
            sphere_scale = Vector3(scale, scale, scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishSphere()", type(scale).__name__)
            return False

        # Increment the ID number
        self.sphere_marker.id += 1

        # Get the default parameters
        sphere_marker = self.sphere_marker2  # sphere_marker2 = SPHERE_LIST

        if lifetime == None:
            sphere_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            sphere_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        sphere_marker.header.stamp = rospy.Time.now()

        # Set marker size
        sphere_marker.scale = sphere_scale

        # Set marker color
        sphere_marker.color = self.getColor(color)

        # Set the pose of one sphere in the list
        sphere_marker.points[0] = sphere_pose.position
        sphere_marker.colors[0] = self.getColor(color)

        return self.publishMarker(sphere_marker)

    def publishArrow(self, pose, color, scale, base_frame, in_id = 0, lifetime=None, toArray=False):
        """
        Publish an arrow Marker.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """
        self.arrow_marker = Marker()
        self.arrow_marker.header.frame_id = base_frame
        self.arrow_marker.ns = "Arrow"  # unique ID
        self.arrow_marker.action = Marker().ADD
        self.arrow_marker.type = Marker().ARROW
        self.arrow_marker.lifetime = self.marker_lifetime

        if (self.muted == True):
            return True

        # arrows list
        # _pose = pose
        # arrow_pose = []
        # for pose in _pose:
        #     if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
        #         arrow_pose.append(mat_to_pose(pose))
        #     elif type(pose) == Pose:
        #         arrow_pose.append(pose)
        #     else:
        #         rospy.logerr(
        #             "Pose is unsupported type '%s' in publishArrow()", type(pose).__name__)
        #         return False

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            arrow_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            arrow_pose = pose
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishArrow()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            arrow_scale = scale
        elif type(scale) == float:
            arrow_scale = Vector3(scale, 0.1*scale, 0.1*scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishArrow()", type(scale).__name__)
            return False

        # Increment the ID number
        self.arrow_marker.id = in_id

        # Get the default parameters
        arrow_marker = self.arrow_marker

        if lifetime == None:
            arrow_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            arrow_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        arrow_marker.header.stamp = rospy.Time.now()

        # Set the pose
        # arrow_marker.pose = pose
        arrow_marker.pose = arrow_pose

        # Set marker size
        arrow_marker.scale = arrow_scale

        # Set marker color
        arrow_marker.color = self.getColor(color)

        if toArray == True:
            return arrow_marker

        return self.publishMarker(arrow_marker)

    def publishCube(self, pose, color, scale, lifetime=None):
        """
        Publish a cube Marker.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            cube_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            cube_pose = pose
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishCube()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            cube_scale = scale
        elif type(scale) == float:
            cube_scale = Vector3(scale, scale, scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishCube()", type(scale).__name__)
            return False

        # Increment the ID number
        self.cube_marker.id += 1

        # Get the default parameters
        cube_marker = self.cube_marker

        if lifetime == None:
            cube_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            cube_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        cube_marker.header.stamp = rospy.Time.now()

        # Set the pose
        cube_marker.pose = cube_pose

        # Set marker size
        cube_marker.scale = cube_scale

        # Set marker color
        cube_marker.color = self.getColor(color)

        return self.publishMarker(cube_marker)

    def publishCubes(self, list_of_cubes, color, scale, lifetime=None):
        """
        Publish a list of cubes.

        @param list_of_cubes (list of np matrix, list of np ndarray, list of ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Check input
        if type(list_of_cubes) != list:
            rospy.logerr("list_of_cubes is unsupported type '%s' in publishCubes()", type(
                list_of_cubes).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            cubes_scale = scale
        elif type(scale) == float:
            cubes_scale = Vector3(scale, scale, scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishCubes()", type(scale).__name__)
            return False

        # Increment the ID number
        self.cubes_marker.id += 1

        # Get the default parameters
        cubes_marker = self.cubes_marker

        if lifetime == None:
            cubes_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            cubes_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        cubes_marker.header.stamp = rospy.Time.now()

        # Set marker size
        cubes_marker.scale = cubes_scale

        # Set marker color
        cubes_marker.color = self.getColor(color)

        cubes_color = self.getColor(color)

        # Set the cubes positions and color
        cubes_marker.points[:] = []  # clear
        cubes_marker.colors[:] = []
        for i in range(0, len(list_of_cubes)):

            # Each cube position needs to be a ROS Point Msg
            if type(list_of_cubes[i]) == Pose:
                cubes_marker.points.append(list_of_cubes[i].position)
                cubes_marker.colors.append(cubes_color)
            elif (type(list_of_cubes[i]) == np.matrix) or (type(list_of_cubes[i]) == np.ndarray):
                pose_i = mat_to_pose(list_of_cubes[i])
                cubes_marker.points.append(pose_i.position)
                cubes_marker.colors.append(cubes_color)
            elif type(list_of_cubes[i]) == Point:
                cubes_marker.points.append(list_of_cubes[i])
                cubes_marker.colors.append(cubes_color)
            else:
                rospy.logerr("list_of_cubes contains unsupported type '%s' in publishCubes()", type(
                    list_of_cubes[i]).__name__)
                return False

        return self.publishMarker(cubes_marker)

    def publishBlock(self, pose, color, scale, lifetime=None):
        """
        Publish a cube Marker.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        return self.publishCube(pose, color, scale)

    def publishCylinder(self, pose, color, height, radius, lifetime=None):
        """
        Publish a cylinder Marker.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param height (float)
        @param radius (float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            cylinder_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            cylinder_pose = pose
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishCylinder()", type(pose).__name__)
            return False

        # Increment the ID number
        self.cylinder_marker.id += 1

        # Get the default parameters
        cylinder_marker = self.cylinder_marker

        if lifetime == None:
            cylinder_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            cylinder_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        cylinder_marker.header.stamp = rospy.Time.now()

        # Set the pose
        cylinder_marker.pose = cylinder_pose

        # Set marker size
        cylinder_marker.scale.x = radius
        cylinder_marker.scale.y = radius
        cylinder_marker.scale.z = height

        # Set marker color
        cylinder_marker.color = self.getColor(color)

        return self.publishMarker(cylinder_marker)

    def publishAxis(self, pose, length, radius, lifetime=None):
        """
        Publish an axis Marker.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param length axis length (float)
        @param radius axis radius (float)
        @param lifetime (float, None = never expire)
        """

        # Convert input pose to a np matrix
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            axis_pose = pose
        elif type(pose) == Pose:
            axis_pose = pose_to_mat(pose)
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishAxis()", type(pose).__name__)
            return False

        t = tf.transformations.translation_matrix((length/2.0, 0.0, 0.0))
        r = tf.transformations.rotation_matrix(np.pi/2.0, (0, 1, 0))
        m = tf.transformations.concatenate_matrices(axis_pose, t, r)
        x_pose = mat_to_pose(m)
        self.publishCylinder(x_pose, 'red', length, radius, lifetime)

        t = tf.transformations.translation_matrix((0.0, length/2.0, 0.0))
        r = tf.transformations.rotation_matrix(np.pi/2.0, (1, 0, 0))
        m = tf.transformations.concatenate_matrices(axis_pose, t, r)
        y_pose = mat_to_pose(m)
        self.publishCylinder(y_pose, 'green', length, radius, lifetime)

        t = tf.transformations.translation_matrix((0.0, 0.0, length/2.0))
        r = tf.transformations.rotation_matrix(0.0, (0, 0, 1))
        m = tf.transformations.concatenate_matrices(axis_pose, t, r)
        z_pose = mat_to_pose(m)
        self.publishCylinder(z_pose, 'blue', length, radius, lifetime)

        return True

    def publishMesh(self, pose, file_name, color, scale, base_frame, lifetime=None, toArray = False):
        """
        Publish a mesh Marker. The mesh file can be a binary STL or collada DAE file.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param file_name (string)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """
        # self.mesh_marker = Marker()
        self.mesh_marker.header.frame_id = base_frame
        # self.mesh_marker.ns = "Mesh"  # unique ID
        # self.mesh_marker.action = Marker().ADD
        # self.mesh_marker.type = Marker().MESH_RESOURCE
        # self.mesh_marker.lifetime = self.marker_lifetime

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            mesh_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            mesh_pose = pose
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishMesh()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            mesh_scale = scale
        elif type(scale) == float:
            mesh_scale = Vector3(scale, scale, scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishMesh()", type(scale).__name__)
            return False

        # Increment the ID number
        self.mesh_marker.id += 1

        # Get the default parameters
        mesh_marker = self.mesh_marker

        if lifetime == None:
            mesh_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            mesh_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        mesh_marker.header.stamp = rospy.Time.now()

        # Set marker size
        mesh_marker.scale = mesh_scale

        # Set marker color
        if color == None:
            mesh_marker.color = ColorRGBA()  # no color
        else:
            mesh_marker.color = self.getColor(color)

        # Set the pose
        mesh_marker.pose = mesh_pose

        # Set the mesh
        mesh_marker.mesh_resource = file_name
        mesh_marker.mesh_use_embedded_materials = True

        if toArray == True:
            return mesh_marker

        return self.publishMarker(mesh_marker)

    def publishRectangle(self, point1, point2, color, lifetime=None):
        """
        Publish a rectangle Marker between two points. If the z-values are not the same then this will result in a cuboid.

        @param point1 (ROS Point)
        @param point2 (ROS Point)
        @param color name (string) or RGB color value (tuple or list)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input points to ROS Point Msgs
        if type(point1) == Point:
            rect_point1 = point1
        else:
            rospy.logerr(
                "Point1 is unsupported type '%s' in publishRectangle()", type(point1).__name__)
            return False
        if type(point2) == Point:
            rect_point2 = point2
        else:
            rospy.logerr(
                "Point2 is unsupported type '%s' in publishRectangle()", type(point2).__name__)
            return False

        # Increment the ID number
        self.rectangle_marker.id += 1

        # Get the default parameters
        rectangle_marker = self.rectangle_marker

        if lifetime == None:
            rectangle_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            rectangle_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        rectangle_marker.header.stamp = rospy.Time.now()

        # Set marker color
        rectangle_marker.color = self.getColor(color)

        # Calculate the center pose
        rect_pose = Pose()
        rect_pose.position.x = (
            rect_point1.x - rect_point2.x) / 2.0 + rect_point2.x
        rect_pose.position.y = (
            rect_point1.y - rect_point2.y) / 2.0 + rect_point2.y
        rect_pose.position.z = (
            rect_point1.z - rect_point2.z) / 2.0 + rect_point2.z
        rectangle_marker.pose = rect_pose

        # Calculate scale
        rectangle_marker.scale.x = np.fabs(rect_point1.x - rect_point2.x)
        rectangle_marker.scale.y = np.fabs(rect_point1.y - rect_point2.y)
        rectangle_marker.scale.z = np.fabs(rect_point1.z - rect_point2.z)

        return self.publishMarker(rectangle_marker)

    def publishPlane(self, pose, depth, width, color, lifetime=None):
        """
        Publish a plane Marker.

        @param pose (np matrix, np ndarray, ROS Pose)
        @param depth (float)
        @param width (float)
        @param color name (string) or RGB color value (tuple or list)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            rect_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            rect_pose = pose
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishRectangle()", type(pose).__name__)
            return False

        # Increment the ID number
        self.rectangle_marker.id += 1

        # Get the default parameters
        rectangle_marker = self.rectangle_marker

        if lifetime == None:
            rectangle_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            rectangle_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        rectangle_marker.header.stamp = rospy.Time.now()

        # Set marker color
        rectangle_marker.color = self.getColor(color)

        # Set the pose
        rectangle_marker.pose = rect_pose

        # Set the scale
        rectangle_marker.scale.x = depth
        rectangle_marker.scale.y = width
        rectangle_marker.scale.z = 0.0

        return self.publishMarker(rectangle_marker)

    def publishLine(self, point1, point2, color, width, lifetime=None):
        """
        Publish a line Marker between two points.

        @param point1 (ROS Point, ROS Pose, np matrix, np ndarray)
        @param point2 (ROS Point, ROS Pose, np matrix, np ndarray)
        @param color name (string) or RGB color value (tuple or list)
        @param width (float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input points to ROS Point Msgs
        if type(point1) == Point:
            line_point1 = point1
        elif type(point1) == Pose:
            position = point1.position
            line_point1 = Point(position.x, position.y, position.z)
        elif (type(point1) == np.matrix) or (type(point1) == np.ndarray):
            pose = mat_to_pose(point1)
            position = pose.position
            line_point1 = Point(position.x, position.y, position.z)
        else:
            rospy.logerr(
                "Point1 is unsupported type '%s' in publishLine()", type(point1).__name__)
            return False

        if type(point2) == Point:
            line_point2 = point2
        elif type(point2) == Pose:
            position = point2.position
            line_point2 = Point(position.x, position.y, position.z)
        elif (type(point2) == np.matrix) or (type(point2) == np.ndarray):
            pose = mat_to_pose(point2)
            position = pose.position
            line_point2 = Point(position.x, position.y, position.z)
        else:
            rospy.logerr(
                "Point2 is unsupported type '%s' in publishLine()", type(point2).__name__)
            return False

        # Increment the ID number
        self.line_marker.id += 1

        # Get the default parameters
        line_marker = self.line_marker

        if lifetime == None:
            line_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            line_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        line_marker.header.stamp = rospy.Time.now()

        # Set marker color
        line_marker.color = self.getColor(color)

        # Set the start and end points
        line_marker.points[:] = []  # clear
        line_marker.points.append(line_point1)
        line_marker.points.append(line_point2)

        # Set the line width
        line_marker.scale.x = width

        return self.publishMarker(line_marker)

    def publishPath(self, path, color, width, in_id = 0, lifetime=None, toArray=False):
        """
        Publish a path Marker using a set of waypoints.

        @param path (list of ROS Points)
        @param color name (string) or RGB color value (tuple or list)
        @param width (float)
        @param lifetime (float, None = never expire)
        """
        self.path_marker = Marker()
        self.path_marker.header.frame_id = self.base_frame
        self.path_marker.ns = "Path"  # unique ID
        self.path_marker.action = Marker().ADD
        self.path_marker.type = Marker().LINE_LIST
        self.path_marker.lifetime = self.marker_lifetime

        if (self.muted == True):
            return True

        # Check input
        if type(path) == list:
            path_path = path  # :-)
        else:
            rospy.logerr(
                "Path is unsupported type '%s' in publishPath()", type(path).__name__)
            return False

        # Increment the ID number
        self.path_marker.id = in_id

        # Get the default parameters
        path_marker = self.path_marker

        if lifetime == None:
            path_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            path_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        path_marker.header.stamp = rospy.Time.now()

        # Set the path width
        path_marker.scale.x = width

        path_color = self.getColor(color)

        # Set the path points and color
        path_marker.points[:] = []  # clear
        path_marker.colors[:] = []
        for i in range(1, len(path)):

            # Each path waypoint needs to be a ROS Point Msg
            if type(path[i]) == Point:
                # Start of segment is previous point
                path_marker.points.append(path[i-1])
                path_marker.colors.append(path_color)
                # End of segment is current point
                path_marker.points.append(path[i])
                path_marker.colors.append(path_color)
            elif type(path[i]) == Pose:
                # Start of segment is previous point
                position = path[i-1].position
                point = Point(position.x, position.y, position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
                # End of segment is current point
                position = path[i].position
                point = Point(position.x, position.y, position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
            elif (type(path[i]) == np.matrix) or (type(path[i]) == np.ndarray):
                # Start of segment is previous point
                pose = mat_to_pose(path[i-1])
                position = pose.position
                point = Point(position.x, position.y, position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
                # End of segment is current point
                pose = mat_to_pose(path[i])
                position = pose.position
                point = Point(position.x, position.y, position.z)
                path_marker.points.append(point)
                path_marker.colors.append(path_color)
            else:
                rospy.logerr("path list contains unsupported type '%s' in publishPath()", type(
                    path[i]).__name__)
                return False
        if toArray == True:
            return path_marker

        return self.publishMarker(path_marker)

    def publishPolygon(self, polygon, color, width, lifetime=None):
        """
        Publish a polygon Marker.

        @param polygon (ROS Polygon)
        @param color name (string) or RGB color value (tuple or list)
        @param width line width (float)
        @param lifetime (float, None = never expire)

        a path with the start and end points connected
        """

        if (self.muted == True):
            return True

        # Check input
        if type(polygon) == Polygon:
            polygon_msg = polygon
        else:
            rospy.logerr(
                "Path is unsupported type '%s' in publishPolygon()", type(polygon).__name__)
            return False

        # Copy points from ROS Polygon Msg into a list
        polygon_path = []
        for i in range(0, len(polygon_msg.points)):
            x = polygon_msg.points[i].x
            y = polygon_msg.points[i].y
            z = polygon_msg.points[i].z
            polygon_path.append(Point(x, y, z))

        # Add the first point again
        x = polygon_msg.points[0].x
        y = polygon_msg.points[0].y
        z = polygon_msg.points[0].z
        polygon_path.append(Point(x, y, z))

        return self.publishPath(polygon_path, color, width, lifetime)

    def publishSpheres(self, list_of_spheres, color, scale, lifetime=None):
        """
        Publish a list of spheres. This renders smoother, flatter-looking spheres.

        @param list_of_spheres (list of np matrix, list of np ndarray, list of ROS Pose)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Check input
        if type(list_of_spheres) != list:
            rospy.logerr("list_of_spheres is unsupported type '%s' in publishSpheres()", type(
                list_of_spheres).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            spheres_scale = scale
        elif type(scale) == float:
            spheres_scale = Vector3(scale, scale, scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishSpheres()", type(scale).__name__)
            return False

        # Increment the ID number
        self.spheres_marker.id += 1

        # Get the default parameters
        spheres_marker = self.spheres_marker

        if lifetime == None:
            spheres_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            spheres_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        spheres_marker.header.stamp = rospy.Time.now()

        # Set marker size
        spheres_marker.scale = spheres_scale

        # Set marker color
        spheres_marker.color = self.getColor(color)

        spheres_color = self.getColor(color)
        #spheres_marker.color = spheres_color

        # Set the sphere positions and color
        spheres_marker.points[:] = []  # clear
        spheres_marker.colors[:] = []
        for i in range(0, len(list_of_spheres)):

            # Each sphere position needs to be a ROS Point Msg
            if type(list_of_spheres[i]) == Pose:
                spheres_marker.points.append(list_of_spheres[i].position)
                spheres_marker.colors.append(spheres_color)
            elif (type(list_of_spheres[i]) == np.matrix) or (type(list_of_spheres[i]) == np.ndarray):
                pose_i = mat_to_pose(list_of_spheres[i])
                spheres_marker.points.append(pose_i.position)
                spheres_marker.colors.append(spheres_color)
            elif type(list_of_spheres[i]) == Point:
                spheres_marker.points.append(list_of_spheres[i])
                spheres_marker.colors.append(spheres_color)
            else:
                rospy.logerr("list_of_sphere contains unsupported type '%s' in publishSphere()", type(
                    list_of_spheres[i]).__name__)
                return False

        return self.publishMarker(spheres_marker)

    def publishText(self, pose, text, color, scale, lifetime=None):
        """
        Publish a text Marker

        @param pose (np matrix, np ndarray, ROS Pose)
        @param text (string)
        @param color name (string) or RGB color value (tuple or list)
        @param scale (ROS Vector3, float)
        @param lifetime (float, None = never expire)
        """

        if (self.muted == True):
            return True

        # Convert input pose to a ROS Pose Msg
        if (type(pose) == np.matrix) or (type(pose) == np.ndarray):
            text_pose = mat_to_pose(pose)
        elif type(pose) == Pose:
            text_pose = pose
        else:
            rospy.logerr(
                "Pose is unsupported type '%s' in publishText()", type(pose).__name__)
            return False

        # Convert input scale to a ROS Vector3 Msg
        if type(scale) == Vector3:
            text_scale = scale
        elif type(scale) == float:
            text_scale = Vector3(scale, scale, scale)
        else:
            rospy.logerr(
                "Scale is unsupported type '%s' in publishText()", type(scale).__name__)
            return False

        # Increment the ID number
        self.text_marker.id += 1

        # Get the default parameters
        text_marker = self.text_marker

        if lifetime == None:
            text_marker.lifetime = rospy.Duration(
                0.0)  # 0 = Marker never expires
        else:
            text_marker.lifetime = rospy.Duration(lifetime)  # in seconds

        # Set the timestamp
        text_marker.header.stamp = rospy.Time.now()

        # Set the pose
        text_marker.pose = text_pose

        # Set marker size
        text_marker.scale = text_scale

        # Set marker color
        text_marker.color = self.getColor(color)

        text_marker.text = text

        return self.publishMarker(text_marker)


#------------------------------------------------------------------------------#


def pose_to_mat(pose):
    """
    Convert a ROS Pose msg to a 4x4 matrix.

    @param pose (ROS geometry_msgs.msg.Pose)

    @return mat 4x4 matrix (np.matrix)
    """

    quat = [pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w]
    pos = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
    mat = np.matrix(tf.transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos

    return mat


def mat_to_pose(mat):
    """
    Convert a homogeneous transformation matrix to a ROS Pose msg.

    @param mat 4x4 homogenous transform (np.matrix or np.ndarray)

    @return pose (ROS geometry_msgs.msg.Pose)
    """

    pose = Pose()
    pose.position.x = mat[0, 3]
    pose.position.y = mat[1, 3]
    pose.position.z = mat[2, 3]

    quat = tf.transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

#--------------------------------------------------------------------------------------#
# def


def genPublishPath(point_list, base_frame, name_id, color, width, lifetime):
    markers = RvizMarkers(base_frame, name_id)
    path = []
    for i in point_list:
        path.append(Point(i))
    markers.publishPath(path, color, width, lifetime)


def getTMatrix(point, heading):
    zaxis = (0, 0, 1)
    # print(heading)
    try:
        heading = math.radians(-heading+90)
    except TypeError:
        print('error!')
        # print('np.shape(heading)', np.shape(heading))

        return None

    Rx = tf.transformations.rotation_matrix(heading, zaxis)
    # print("heading:",heading)
    #print('Rx:', Rx)
    Tm = tf.transformations.translation_matrix(point)
    # print("Tm",Tm)
    T = tf.transformations.concatenate_matrices(Tm, Rx)
    # print(T)
    return T

def getHeading(heading):
    zaxis = (0, 0, 1)
    _quaternion = tf.transformations.quaternion_about_axis(math.radians(-heading+90),zaxis)
    return _quaternion


def getRandomColor():
    # result = ColorRGBA()
    a = 1
    while True:
        r = random.random()  # random float from 0 to 1
        g = random.random()
        b = random.random()
        if ((r + g + b) > 1.5):  # 0=black, 3=white
            break
    return [r, g, b, a]


def genPublishPath_simple(points, markers, lifetime=5.0, toArray=False):
    color = getRandomColor()
    for count, lines in enumerate(points):
        paths = []
        # print(count)
        for i in lines:
            if i == []:
                continue
            # print(type(i[0]))
            # print('i[0]:',i[0])
            # print("--------")
            # print(i)
            paths.append(Point(i[0], i[1], i[2]))
            # print(i[0])
            width = 0.3
            # path, color, width, lifetime
        markers.publishPath(paths, 'white', width, lifetime, toArray)


def genPublishArrow_simple(point_list, headings, markers, color, lifetime=1.0, toArray=False):
    pathArray = ArrayMarkers('arrow_Array')
    for _points, _headings in zip(point_list, headings):
        # color = getRandomColor()
        for __points, __headings in zip(_points, _headings):
            # print("roger")
            # print('points, vector', points, vector)
            T = getTMatrix(__points, __headings)
            scale = Vector3(3.0, 0.4, 0.2)  # x=length, y=height, z=height
            # pose, color, scale, lifetime
            markers.publishArrow(T, 'pink', scale, lifetime, toArray)


def genPublishPathArray_simple(points, markers, colors, width, pathArray_name, lifetime=5.0, toArray=True, toPub = True):
    pathArray = ArrayMarkers(pathArray_name)
    # color = getRandomColor()
    for count, lines in enumerate(points):
        paths = []
        # print(count)
        for i in lines:
            if i == []:
                continue
            # print(type(i[0]))
            # print('i[0]:',i[0])
            # print("--------")
            # print(i)
            paths.append(Point(i[0], i[1], i[2]))
            # print(i[0])
            # width = 0.1
            # path, color, width, lifetime
        # markers.publishPath(paths, 'white', width, lifetime, toArray)

        tmp = markers.publishPath(
            paths, colors, width, pathArray.in_id, lifetime, toArray)
        pathArray.addMarkers(tmp)
    # with open('test_bug.txt', 'w') as fb:
    #     print >> fb, pathArray.inerMarker.markers
    # print >>  pathArray.inerMarker.markers
    # print(len(pathArray.inerMarker.markers))
    if toPub == True:
        return pathArray

    pathArray.pubulishMarkerArrow()

# def genPublishPathArray_simple2(points, markers, colors, width, pathArray, lifetime=5.0, toArray=True):
#     # pathArray = ArrayMarkers('lanemarker_Array')
#     # color = getRandomColor()
#     for count, lines in enumerate(points):
#         paths = []
#         # print(count)
#         for i in lines:
#             if i == []:
#                 continue
#             # print(type(i[0]))
#             # print('i[0]:',i[0])
#             # print("--------")
#             # print(i)
#             paths.append(Point(i[0], i[1], i[2]))
#             # print(i[0])
#             # width = 0.1
#             # path, color, width, lifetime
#         # markers.publishPath(paths, 'white', width, lifetime, toArray)

#         tmp = markers.publishPath(
#             paths, colors, width, pathArray.in_id, lifetime, toArray)
#         pathArray.addMarkers(tmp)
#     # with open('test_bug.txt', 'w') as fb:
#     #     print >> fb, pathArray.inerMarker.markers
#     # print >>  pathArray.inerMarker.markers
#     # print(len(pathArray.inerMarker.markers))
#     pathArray.pubulishMarkerArrow()



def genPublishArrowArray_simple(point_list, headings, markers, color, lifetime=10.0, toArray=True):
    pathArray = ArrayMarkers('arrow_Array')
    # print("roger that")
    # print(np.shape(point_list), np.shape(headings))
    count = 0
    for _points, _headings in zip(point_list, headings):

        # color = getRandomColor()
        for __points, __headings in zip(_points, _headings):
            count += 1
            # print("roger")
            # print('points, vector', points, vector)
            T = getTMatrix(__points, __headings)
            scale = Vector3(3.0, 0.4, 0.2)  # x=length, y=height, z=height
            # pose, color, scale, lifetime
            tmp = markers.publishArrow(
                T, 'pink', scale, pathArray.in_id, lifetime, toArray)
            pathArray.addMarkers(tmp)
            if count == 10000:
                break
        # __points,__headings -> [1,2,3], 20
        # points_list, heading ->  [[[1, 2, 3]]] [[20]]
        # if count== 3:
        #   break
    # print(len(pathArray.inerMarker.markers))
    pathArray.pubulishMarkerArrow()

def genPublishSphereArray_simple(point_list, markers, color, arrayName, scale, lifetime = None, toArray = True, toPub = True):

    sphereArray = ArrayMarkers(arrayName)
    count = 0
    for ele in point_list:
        
        p = Point(ele[0], ele[1], ele[2])
        # print(p)
        tmp = markers.publishSphere(p, color, scale, count, lifetime, toArray)
        sphereArray.addMarkers(tmp)
        count+=1
    if toPub == True:
        return sphereArray
    sphereArray.pubulishMarkerArrow()

def genPublishSphereArray_simple_plus(point_list, markers, marker_array, color, scale, count, lifetime = None, toArray = True, toPub = True):

    # sphereArray = ArrayMarkers(arrayName)
    # count = 0
    for ele in point_list:
        p = Point(ele[0], ele[1], ele[2])
        # print(p)
        tmp = markers.publishSphere(p, color, scale, count, lifetime, toArray)
        marker_array.addMarkers(tmp)
        count+=1
    if toPub == True:
        return marker_array, count
    else:
        print("Wrong Doer!")
    # marker_array.pubulishMarkerArrow()


def create_cloud_xyz(header, points):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),]
                # PointField('intensity', 12, PointField.FLOAT32, 1)],
                # PointField('label', 16, PointField.FLOAT32, 1)
    return pc2.create_cloud(header, fields, points)

def convert2cloud(temp_point):
    temp_point = np.array(temp_point)
    x = temp_point[:, 0].reshape(-1)
    y = temp_point[:, 1].reshape(-1)
    z = temp_point[:, 2].reshape(-1)
    # intensity = 0.5*np.ones_like(x)
    cloud = np.stack((x, y, z))
    # print('np,shape(temp_point)',np.shape(temp_point))
    # print(np.shape(x), np.shape(cloud))
    header = Header()
    header.stamp = rospy.Time().now()
    header.frame_id = "/vehicle"
    pointclouds = create_cloud_xyz(header, cloud.T)

    divider_pub = rospy.Publisher("/dividers_topic", PointCloud2, queue_size=10)
    divider_pub.publish(pointclouds)
    # print("pub once")
    return True

def convert2cloud2(temp_point, _header = "/vehicle", _topic = 'point_cloud'):
    temp_point = np.array(temp_point)
    x = temp_point[:, 0].reshape(-1)
    y = temp_point[:, 1].reshape(-1)
    z = temp_point[:, 2].reshape(-1)
    # intensity = 0.5*np.ones_like(x)
    cloud = np.stack((x, y, z))
    # print('np,shape(temp_point)',np.shape(temp_point))
    # print(np.shape(x), np.shape(cloud))
    header = Header()
    header.stamp = rospy.Time().now()
    header.frame_id = _header
    pointclouds = create_cloud_xyz(header, cloud.T)

    divider_pub = rospy.Publisher(_topic, PointCloud2, queue_size=10)
    divider_pub.publish(pointclouds)
    # print("pub once")
    return True


def read_points_per_frame(path):

    tmp = np.loadtxt(path, delimiter=' ')
    frame = []
    points_line = []
    points_line_i = []
    before = 0
    for ele in tmp:
        
        if all(ele[:2] == before):
            points_line_i.append([ele[2], ele[3], 0])
        elif before !=0:
            frame.append([ele[0], ele[1], 0])
            before  = ele[:2]
            points_line.append([ele[2], ele[3], 0])
            points_line_i = []
            points_line_i.append([ele[2], ele[3], 0])
        elif before ==0:
            frame.append([ele[0], ele[1], 0])
            points_line_i.append([ele[2], ele[3], 0])
            
    points_line.append(points_line_i)
    
    return frame, np.array(points_line, dtype=float)
