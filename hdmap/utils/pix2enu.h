//
// Created by catalina on 2019/8/7.
//

#ifndef HDMAP_PIX2ENU_H
#define HDMAP_PIX2ENU_H
#include <iostream>
#include "utils/Utils.h"
#include <math.h>
#include <iomanip>
#include "utils/read_hdmap.h"

// 相机参数加载和相关参数初始化
/*const std::string strCameraPath = "../config/param.yml";
cv::FileStorage fSettings(strCameraPath, cv::FileStorage::READ);
double fx = fSettings["Camera.fx"];
double fy = fSettings["Camera.fy"];
double cx = fSettings["Camera.cx"];
double cy = fSettings["Camera.cy"];*/
// 这里的K和distCoeffs都是全局变量

Utils::new3s_PointXYZ  scene_point_start;
Utils trans;
double theta_p = 4*3.14/180;
double theta_y = -5*3.14/180;
double hh=1.32;
double dx=3.6e-6;
cv::Mat R1 = (cv::Mat_<double>(3, 3) <<
                                     1, 0.0, 0,
        0.0, cos(theta_p),-sin(theta_p),
        0.0, sin(theta_p), cos(theta_p));
cv::Mat R2 = (cv::Mat_<double>(3, 3) <<
                                     cos(theta_y), -sin(theta_y), 0,
        sin(theta_y), cos(theta_y),0,
        0.0, 0, 1);
cv::Mat K = (cv::Mat_<double>(3, 3) <<
                            fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
PointT pixel2Cam(const PointT& p, const cv::Mat K) {
    PointT cameraCoord;
//    std::cout << "K.at<double>(1,1)" << K.at<double>(1, 1) << std::endl;
//    std::cout << "p.y:" << p.y << std::endl;
//    double depth = 1.32 * K.at<double>(1, 1) / (p.y - K.at<double>(1, 2));
    double depth =( pow(K.at<double>(1, 1),2)*dx*tan(theta_p) + K.at<double>(1, 1)*hh/cos(theta_p) ) /(p.y - K.at<double>(1, 2)- K.at<double>(1, 1)*tan(theta_p));
//    std::cout << "depth:" << depth << std::endl;
    cameraCoord.y = depth;
    cameraCoord.x =(p.x - K.at<double>(0, 2)) * abs(depth) / K.at<double>(0, 0);
    cameraCoord.z = 0;//depth*(p.y - K.at<double>(1, 2))/K.at<double>(1, 1);
    return cameraCoord;
}

PointT car2ENU(const PointT& gps, double heading, const PointT& car_p){
    PointT ret;
    heading = (heading )*3.15/180.0 ;
    cv::Mat Rtem = (cv::Mat_<double>(3, 1) << car_p.x, car_p.y,car_p.z );
    cv::Mat res = R1*R2*Rtem;
    ret.x = gps.x + res.at<double>(1,0)*sin(heading) + res.at<double>(0,0)*cos(heading);
    ret.y = gps.y + res.at<double>(1,0)*cos(heading) - res.at<double>(0,0)*sin(heading);
    return ret;
}

bool pix2enu2(vector<DividerEach> &divider_vec ,string scene_id, int index) {
    GPSInfoEach gpsInfoEach;
    bool flasg = ReadHDMap::getGPSInfoBySceneId(scene_id, gpsInfoEach);
    vector<GPSPointEach> gps_points = gpsInfoEach.gpsPoints;

    DetchBatch detchBatch;
    bool flag8 = ReadHDMap::getAllDetectionBatchByIndex(scene_id, index, detchBatch);
    vector<DividerEach> d_vec = detchBatch.dividerPerFrame.dividerEach_vec;
    cout << " divider_size:" << d_vec.size() << endl;

//    vector<DividerEach> pix2enu_vec;
    for (int i = 0; i < d_vec.size(); i++) {
        vector<PointT> divider_points = d_vec[i].points_vec;
        DividerEach enu_points_each;
        for (int j = 0; j < divider_points.size(); j++) {
            //current body coordination
//            std::cout << "================divider_points[j]=============" << divider_points[j].x << ","
//                      << divider_points[j].y << std::endl;
            PointT cameraPoint = pixel2Cam(divider_points[j], K);
            //to ENU
            if (cameraPoint.y > 25) {
                continue;
            }
            PointT enu_point = car2ENU(gps_points[index].points, gps_points[index].heading, cameraPoint);
            enu_points_each.points_vec.push_back(enu_point);
        }
        divider_vec.push_back(enu_points_each);
    }
    return true;
}

#endif //HDMAP_PIX2ENU_H
