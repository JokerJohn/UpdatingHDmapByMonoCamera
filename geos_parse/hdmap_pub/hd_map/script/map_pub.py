#!/usr/bin/python2
# -*- coding: utf-8 -*-
import rospy
from hd_map.msg import element
from hd_map.msg import elements
from HDMap_pb2 import HDMap
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import re
import numpy as np

class hdmap_pub():
    def __init__(self, message, pub_rate):
        self.divider_pub = rospy.Publisher("/dividers_topic", elements, queue_size=10)
        self.trafficlight_pub = rospy.Publisher("/trafficlight_topic", elements, queue_size=10)
        self.landmark_pub = rospy.Publisher("/landmark_topic", elements, queue_size=10)
        self.message = message
        self.pub_rate = pub_rate
        while not rospy.is_shutdown():
            self.collect_divider()
            self.collect_trafficlight()
            self.collect_landmark()
        rospy.spin()
    def collect_divider(self):
        dividers = elements()
        rate = rospy.Rate(self.pub_rate)

        for ele in self.message.dividers:
            divider = element()
            # record sequence
            divider.id = ele.id
            # 颜色，0：未知；1：白色；2：黄色
            divider.color = ele.color
            # 类型 0: 未知； 1：单虚线；2：单实线；3：双实线；4：左实右虚；5：左虚右实,；6: 不可通行减速线；7：可通行减速线
            divider.element_type = ele.type
            # read point
            elestr = ele.geometry
            tmp = re.findall(r"\d+\.?\d*", elestr)
            #=============1
            # cloud_arr = []
            cloud_arr = list(map(float, tmp))
            # cloud_str= [str(i) for i in cloud_arr]
            # cloud = ",".join(cloud_str)
            # #============data给不进去
            # divider.pointclouds.data = cloud
            #==============2
            # divider.pointclouds.data = temp_point.tostring()
            #=====3
            # new_pcd = pcl2.PointCloud2(temp_point)

            # p = pcl.PointCloud(np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32))
            step = 3
            temp_point =np.array([tmp[i:i + step] for i in range(0, len(tmp), step)], dtype=float)
            x = temp_point[:, 0].reshape(-1)
            y = temp_point[:, 1].reshape(-1)
            z = temp_point[:, 2].reshape(-1)
            cloud = np.stack((x, y, z))
            header = Header()
            header.stamp = rospy.Time().now()
            header.frame_id = "divider"

            # point cloud segments
            divider.pointclouds = self.create_cloud_xyz(header, cloud.T)

            # append
            dividers.elements.append(divider)
        # 0 divider; 1 trafficlight; 2 landmark
        dividers.type = 0
        self.divider_pub.publish(dividers)
        # clear container
        dividers.elements=[]
        rate.sleep()  # 按rate设置的发布频率休眠
    def collect_trafficlight(self):
        trafficlights = elements()
        rate = rospy.Rate(self.pub_rate)
        for ele in self.message.tafficlights:
            trafficlight = element()
            # record sequence
            trafficlight.id = ele.id
            # mwiyou
            trafficlight.color = 0

            geomtry_record = []
            elestr = ele.geometry
            tmp = re.findall(r"\d+\.?\d*", elestr)
            #geomtry_record   qiansandain  shi
            geomtry_record.extend(tmp)
            for dirs in ele.lights:
                #  方向，1：竖排；2：横排; 进入ros后1变为49,2变为50
                trafficlight.direction += str(dirs.direction)
                # read point
                elestr = dirs.geometry
                tmp = re.findall(r"\d+\.?\d*", elestr)
                geomtry_record.extend(tmp)
            step = 3
            temp_point = np.array([geomtry_record[i:i + step] for i in range(0, len(geomtry_record), step)], dtype='float32')
            x = temp_point[:, 0].reshape(-1)
            y = temp_point[:, 1].reshape(-1)
            z = temp_point[:, 2].reshape(-1)
            cloud = np.stack((x, y, z))
            header = Header()
            header.stamp = rospy.Time().now()
            header.frame_id = "trafficlights"
            # point
            trafficlight.pointclouds = self.create_cloud_xyz(header, cloud.T)
            # append
            trafficlights.elements.append(trafficlight)

        # 1 divider; 2 trafficlight; 3 landmark
        trafficlights.type = 1
        #类型，1：车辆；2：行人
        trafficlights.motor_type = ele.motor_type
        self.trafficlight_pub.publish(trafficlights)
        # clear container
        trafficlights.elements = []
        rate.sleep()  # 按rate设置的发布频率休眠
    def collect_landmark(self):
        landmarks = elements()
        rate = rospy.Rate(self.pub_rate)
        for ele in self.message.lanemarkings:
            landmark = element()
            # record sequence
            landmark.id = ele.id
            # 没有color
            landmark.color = 0
            # 类型，1：人行道；2：停止线；3：地面标识
            landmark.element_type = ele.type
            # read point
            elestr = ele.geometry
            tmp = re.findall(r"\d+\.?\d*", elestr)
            step = 3
            temp_point = np.array([tmp[i:i + step] for i in range(0, len(tmp), step)], dtype=float)
            x = temp_point[:, 0].reshape(-1)
            y = temp_point[:, 1].reshape(-1)
            z = temp_point[:, 2].reshape(-1)
            cloud = np.stack((x, y, z))
            header = Header()
            header.stamp = rospy.Time().now()
            header.frame_id = "lanemarking"
            # point cloud segments
            landmark.pointclouds = self.create_cloud_xyz(header, cloud.T)

            # append
            landmarks.elements.append(landmark)


        # 0 divider; 1 trafficlight; 2 landmark
        landmarks.type = 2
        #meiyou
        landmarks.motor_type = 0

        self.landmark_pub.publish(landmarks)
        # clear container
        landmarks.elements=[]
        rate.sleep()  # 按rate设置的发布频率休眠
    def create_cloud_xyz(self, header, points):
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),]
                  # PointField('intensity', 12, PointField.FLOAT32, 1),
                  # PointField('label', 16, PointField.FLOAT32, 1)
        return pc2.create_cloud(header, fields, points)

if __name__ == '__main__':
    rospy.init_node('hdmap', anonymous=True)

    message = HDMap()
    with open('/home/lfg/my_work/deecamp/roshd/src/hd_map/hdmap_deecamp.pb', 'rb') as fb:
        pb_content = fb.read()  # 如果文件很大，则要分批读取
        # pb_content 是二进制的pb数据，如果是文件，则需要用 open 方法读取数据
        message.ParseFromString(pb_content)
    pub_rate = 1
    hdmap_pub(message, pub_rate)


