#include<iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

#include "read_detection.h"

typedef struct{
    double x;
    double y;
    int label;
}PointDouble;

typedef struct{
    double x;
    double y;
    double z;
    int label;
}PointDepth;

bool cam2enu(const PointDepth& coors_cam, PointDepth& coors_enu,
            const cv:: Mat& R, const cv:: Mat& t)
{
    cv::Mat pt_trans = R*( Mat_<double>(3,1) << coors_cam.x, coors_cam.y, coors_cam.z) + t;
    coors_enu.x = pt_trans.at<double>(0, 0);
    coors_enu.y = pt_trans.at<double>(1, 0);
    coors_enu.z = pt_trans.at<double>(2, 0);
    coors_enu.label = coors_cam.label;

    return true;
}

bool pixel2cam ( const PointInt& p, PointDouble& cam, const cv::Mat& K )
{
    // K 为内参矩阵，包含的数据类型 double！！！
    cam.x = (p.x - K.at<double>(0,2)) / K.at<double>(0,0);
    cam.y = (p.y - K.at<double>(1,2)) / K.at<double>(1,1);
    cam.label = p.label;
    
    return true;
}


