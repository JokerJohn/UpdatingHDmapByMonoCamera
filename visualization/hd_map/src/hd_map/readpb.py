#!/usr/local/bin/python
# -*- coding: utf-8 -*-
import re
import numpy as np

from HDMap_pb2 import HDMap

class element(object):
    # uint8 id
    # uint8 color
    # uint8 element_type
    # sensor_msgs/PointCloud2 pointclouds
    # uint8[] direction
    def __init__(self):
        self.id = 0
        self.color = 0
        self.element_type = 0
        self.direction = []
    def copy_ele(self, ele):
        self.id = ele.id
        self.color = ele.color
        self.element_type = ele.element_type
        self.direction = ele.direction

class elements(object):
    # uint8 type
    # uint8 motor_type
    # element[] elements
    def __init__(self):
        self.type = 0
        self.motor_type = 0
        self.elements = element
    
    def copy_ele(self, eles):
        self.type = eles.type
        self.motor_type = eles.motor_type
        self.elements = eles.element

    
class Divider(object):
    def __init__(self, id, color, type, geometry):
        self.id = id
        # 颜色，0：未知；1：白色；2：黄色
        self.color = color
        # 类型 0: 未知； 1：单虚线；2：单实线；3：双实线；4：左实右虚；5：左虚右实,；6: 不可通行减速线；7：可通行减速线
        self.type = type
        self.geometry = geometry
        
    def getPoints(self, normal = True, stacked=False):
        elestr = self.geometry
        tmp = re.findall(r"\d+\.?\d*", elestr)
        point_lenth = 3
        # [lat, lon, 0]
        temp_point =np.array([tmp[i:i + point_lenth] for i in range(0, len(tmp), point_lenth)], dtype=float)
        # print("temp_point", temp_point)
        if normal == True:
            temp_point = normalize_for_rviz(temp_point)
        if stacked == False:
            return temp_point # shape = (n, 3)
        x = temp_point[:, 0].reshape(-1)
        y = temp_point[:, 1].reshape(-1)
        z = temp_point[:, 2].reshape(-1)
        
        points = np.stack((x, y, z))
        
        return points #shape = (3, n)

class Trafficlights(element):
    def __init__(self, ele):
        super.__init__()
        super.copy_ele(ele)
        self.geomtry_record = []
        self.lights = ele.lights
        self.geometry = ele.geometry

    def getPoints(self, normal = True):
        
        elestr = self.geometry
        tmp = re.findall(r"\d+\.?\d*", elestr)
        self.geomtry_record.extend(tmp)
        for dirs in self.lights:
            #  方向，1：竖排；2：横排; 进入ros后1变为49,2变为50
            self.direction += str(dirs.direction)
            # read point
            elestr = dirs.geometry
            tmp = re.findall(r"\d+\.?\d*", elestr)
            self.geomtry_record.extend(tmp)
            
        if normal == True:
            self.geomtry_record = normalize_for_rviz(self.geomtry_record)

        return self.geomtry_record
class Landmarks(element):
    def __init__(self, ele):
        super.__init__()
        super.copy_ele(ele)
        self.geometry = ele.geometry

    def getPoints(self, normal = True):
        elestr = self.geometry
        tmp = re.findall(r"\d+\.?\d*", elestr)
        step = 3
        temp_point = np.array([tmp[i:i + step] for i in range(0, len(tmp), step)], dtype=float)
        
        if normal == True:
            temp_point = normalize_for_rviz(temp_point)
    
        # point cloud segments
        # landmark.pointclouds = self.create_cloud_xyz(header, cloud.T)

        # append
        # landmarks.elements.append(landmark)

        return temp_point



def read_from_pb(path = "package://hd_map/data", write2txt = False):
    message = HDMap()
    with open(path+'/hdmap_deecamp.pb', 'rb') as fb:
        pb_content = fb.read() # 如果文件很大，则要分批读取
        # pb_content 是二进制的pb数据，如果是文件，则需要用 open 方法读取数据
        message.ParseFromString(pb_content)
    # print(message)
    
    if write2txt==True:
        with open(path+'/hdmap_message.txt', 'w') as fb:
            print >> fb, message
    return message

def read_from_pb_debug(path = "/home/apple/catkin_ws/src/hd_map/data", write2txt = False):
    message = HDMap()
    with open(path+'/hdmap_deecamp.pb', 'rb') as fb:
        pb_content = fb.read() # 如果文件很大，则要分批读取
        # pb_content 是二进制的pb数据，如果是文件，则需要用 open 方法读取数据
        message.ParseFromString(pb_content)
    # print(message)
    
    if write2txt==True:
        with open(path+'/hdmap_message.txt', 'w') as fb:
            print >> fb, message
    return message

def normalize_for_rviz(temp_point):
    # lat_max = np.max(temp_point[:, 0])
    # lon_max = np.max(temp_point[:, 1])
    # lat_min = np.min(temp_point[:, 0])
    # lon_min = np.min(temp_point[:, 1])

    # print("lat_min, lon_min", lat_min, lon_min)
    # print("temp_point[:, 0][3]", temp_point[:, 0][0])
    # print("temp_point[:, 1][3]", temp_point[:, 1][0])
    lat_min = 22.68037672
    lon_min = 114.36243453
    temp_point[:, 0] = 100000*(temp_point[:, 0]-lat_min)
    temp_point[:, 1] = 100000*(temp_point[:, 1]-lon_min)
    # print("After temp_point[:, 0][3]", temp_point[:, 0][:])
    # print("After temp_point[:, 1][3]", temp_point[:, 1])

    return temp_point

def read_divider(message):
    points_list = []
    for ele in message.dividers:
        this_ele = Divider(ele.id, ele.color, ele.type, ele.geometry)
        
        points= this_ele.getPoints(normal = True, stacked = False)

        # print(points.shape)
        points_list.append(points)
        
    return points_list    
    
def read_trafficlight(message):
    points_list = []
    for ele in message.tafficlights:
        trafficlight = Trafficlights(ele)

        points = trafficlight.getPoints()

        points_list.append(points)
    
    return points_list

if __name__ == "__main__":
    message = read_from_pb_debug()
    res = read_divider(message)
    # print("result:   ", res[0])