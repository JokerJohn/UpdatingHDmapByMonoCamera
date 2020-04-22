#!/usr/local/bin/python
# -*- coding: utf-8 -*-
import re
import numpy as np
import os
import glob

# from HDMap_pb2 import HDMap
# from proto.SourceInfo_pb2 import GPSInfo


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
        self.id = ele.id
        self.geometry = ele.geometry
        self.motor_type = ele.motor_type
        self.lights = ele.lights
        self.geomtry_record = []
        self.location = []

    def getPoints(self, normal = True):
        elestr = self.geometry
        tmp = re.findall(r"\d+\.?\d*", elestr)
        self.location.append(tmp)
        for dirs in self.lights:
            #  方向，1：竖排；2：横排; 进入ros后1变为49,2变为50
            # self.direction += str(dirs.direction)
            # read point
            _elestr = dirs.geometry
            tmp = re.findall(r"\d+\.?\d*", _elestr)
            _id = dirs.id
            self.geomtry_record.append((_id, dirs.direction, tmp))

        if normal == True:
            # print(np.shape(self.location))
            self.location = normalize_for_rviz(self.location)

        return self.location
class Landmarks(element):
    def __init__(self, ele):
        self.id = ele.id
        self.type = ele.type
        self.geometry = ele.geometry

    def getPoints(self, normal = True):
        elestr = self.geometry
        tmp = re.findall(r"\d+\.?\d*", elestr)
        step = 3
        temp_point = np.array([tmp[i:i + step] for i in range(0, len(tmp), step)], dtype=float)
        
        if normal == True:
            temp_point = normalize_for_rviz(temp_point)

        return temp_point

def read_gps_all(path):
    points = []
    headings = []
    count = 0
    
    for f in glob.glob(path+"/gps_store/*.txt"): # glob.glob(path): # 
        #if count ==2:
        #    break
        count+=1
        # print(count)
        #if count ==2:
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




def read_from_pb(path = "package://rviz_tools_py/data", write2txt = False):
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

class Divider2(object):
    def __init__(self, id, color, type, geometry):
        self.id = id
        # 颜色，0：未知；1：白色；2：黄色
        self.color = color
        # 类型 0: 未知； 1：单虚线；2：单实线；3：双实线；4：左实右虚；5：左虚右实,；6: 不可通行减速线；7：可通行减速线
        self.type = type
        self.geometry = geometry
        
    # def getPoints(self, normal = True, stacked=False):
    #     elestr = self.geometry
    #     tmp = re.findall(r"\d+\.?\d*", elestr)
    #     point_lenth = 3
    #     # [lat, lon, 0]
    #     temp_point =np.array([tmp[i:i + point_lenth] for i in range(0, len(tmp), point_lenth)], dtype=float)
    #     # print("temp_point", temp_point)
    #     if normal == True:
    #         temp_point = normalize_for_rviz(temp_point)
    #     if stacked == False:
    #         return temp_point # shape = (n, 3)
    #     x = temp_point[:, 0].reshape(-1)
    #     y = temp_point[:, 1].reshape(-1)
    #     z = temp_point[:, 2].reshape(-1)
        
    #     points = np.stack((x, y, z))
        
    #     return points #shape = (3, n)




def read_divider_txt(file_path = "/home/lfg/my_work/deecamp/lfg/HDMapProject/rviz_ws/src/rviz_tools_py/data/divider_hdmap.txt"):
    id_list = []
    type_list = []
    color_list = []
    geo_list = []

    with open(file_path, 'rt') as f:
        lines = [line.strip('\n').strip(',') for line in f]
        # print(np.shape(lines))
        line_length = 0
        for line in lines:
            line = re.split(',', line)
            # print(float(_line[0]))

            
            try:
                f_line = [float(ele) for ele in line]
            except ValueError:
                print(line)
                print("error!")
                break
            #print(line)
            #print(f_line)
            id_list.append(f_line[0])
            type_list.append(f_line[1])
            color_list.append(f_line[2])

            step = 3
            _geo = np.array([f_line[i:i+step] for i in range(line_length+3,len(f_line), step)], dtype = float)
            
            # fix bug of txt
            line_length = len(f_line)-3
            # print('len(f_line)', len(_geo))
            # print("lenth", line_length)
            
            
            # print(_geo)
            geo_list.append(_geo)

    divider_list = []
    for i in range(len(id_list)):
        divider_list.append(Divider2(id_list[i], color_list[i], type_list[i], np.array(geo_list[i])))
    

    return np.array(geo_list)
    # return divider_list


class Gps(object):
    def __init__(self, scene_id, device_id, pts):
        self.scene_id = scene_id
        self.device_id = device_id
        self.pts = Pts(pts.id, pts.gpstime, pts.heading, pts.speed, pts.geometry)

class Pts(object):
    def __init__(self, id, gpstime, heading, speed, geometry):
        self.id = float(id)
        self.gpstime = float(gpstime)
        self.heading = float(heading)
        self.speed = float(speed)
        self.geometry = self.getGeometry(geometry)

    def getGeometry(self, geometry):
        
        tmp = re.findall(r"\d+\.?\d*", geometry)
        t_geo = [float(i) for i in tmp]

        return t_geo

# from proto.SourceInfo_pb2 import GPSInfo
def read_gps_txt(file_path='"package://rviz_tools_py/data/20190123112752_7ac6ab9d61d94314188426910d324c39_4deecamp_gps.pb'):
    message = GPSInfo()
    with open(file_path, 'rt') as f:
        pb_content = f.read() # 如果文件很大，则要分批读取
        # pb_content 是二进制的pb数据，如果是文件，则需要用 open 方法读取数据
        message.ParseFromString(pb_content)
    
    gps = Gps(message.scene_id, message.device_id, message.pts)
    return gps




def read_from_pb_debug(path = '/home/lfg/my_work/deecamp/lfg/HDMapProject/rviz_ws/src/rviz_tools_py/data/hdmap_deecamp.pb', write2txt = False):
    message = HDMap()
    with open(path, 'rb') as fb:
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
    if type(temp_point) == list:
        temp_point = np.array(temp_point, dtype = float)
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

def read_lanemarkings(message):
    points_list = []
    for ele in message.lanemarkings:
        lanemarkings = Landmarks(ele)

        points = lanemarkings.getPoints()
        points_list.append(points)
    
    return points_list

gps_test_path = '/home/apple/catkin_ws/src/rviz_tools_py/data/GPStrafficLine.txt'
test_bug_txt = '/home/apple/catkin_ws/test_bug.txt'
np_data_path = '/home/apple/catkin_ws/src/rviz_tools_py/data/gps_pre_frame.npy'

def read_points_per_frame(path='/home/apple/catkin_ws/src/rviz_tools_py/data/GPStrafficLine.txt'):

    tmp = np.loadtxt(gps_test_path, delimiter=' ')
    frame = []
    points_line = []
    points_line_i = []
    before = 0
    for ele in tmp:
        
        if all(ele[:2] == before):
            points_line_i.append(ele[2:4])
        elif before !=0:
            frame.append(ele[:2])
            before  = ele[:2]
            points_line.append(points_line_i)
            points_line_i = []
            points_line_i.append(ele[2:4])
        elif before ==0:
            frame.append(ele[:2])
            points_line_i.append(ele[2:4])
            
    points_line.append(points_line_i)
    
    return frame, points_line




# if __name__ == "__main__":
#     message = read_from_pb_debug()
#     # res = read_divider(message)
#     res  = read_lanemarkings(message)
#     print(res)



    # print("result:   ", res[0])
