import re
import numpy as np

from HDMap_pb2 import HDMap


def read_from_pb(path = './hdmap_message.txt', write2txt = False):
    message = HDMap()
    with open('./hdmap_deecamp.pb', 'rb') as fb:
        pb_content = fb.read() # 如果文件很大，则要分批读取
        # pb_content 是二进制的pb数据，如果是文件，则需要用 open 方法读取数据
        message.ParseFromString(pb_content)
    # print(message)
    
    if write2txt==Ture:
        with open('./hdmap_message.txt', 'w') as fb:
            print >> fb, message
    return message
    
class Divider(object):
    def __init__(self, id, color, type, geometry):
        self.id = id
        # 颜色，0：未知；1：白色；2：黄色
        self.color = color
        # 类型 0: 未知； 1：单虚线；2：单实线；3：双实线；4：左实右虚；5：左虚右实,；6: 不可通行减速线；7：可通行减速线
        self.type = type
        self.geometry = geometry
        
    def getPoints(self, stacked=False):
        elestr = self.geometry
        tmp = re.findall(r"\d+\.?\d*", elestr)
        point_lenth = 3
        temp_point =np.array([tmp[i:i + point_lenth] for i in range(0, len(tmp), point_lenth)], dtype=float)
        
        if stacked == False:
            return temp_point # shape = (n, 3)
        x = temp_point[:, 0].reshape(-1)
        y = temp_point[:, 1].reshape(-1)
        z = temp_point[:, 2].reshape(-1)
        
        points = np.stack((x, y, z))
        
        return points #shape = (3, n)
    
def read_divider(message):
    points_list = []
    for ele in message.dividers:
        this_ele = Divider(ele.id, ele.color, ele.type, ele.geometry)
        
        points= this_ele.getPoints(stacked = False)

        # print(points.shape)
        points_list.append(points)
        
    return points_list    
    