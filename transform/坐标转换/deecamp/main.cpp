#include <iostream>
#include "Utils.h"
#include <math.h>
#include <iomanip>
#include "utils/read_hdmap.h"

cv::Mat K;
cv::Mat distCoeffs;
cv::Mat Rcb;
cv::Mat mTcb = cv::Mat::eye(4, 4, CV_64F);
double header_former;
cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);

// 归一化像素坐标系
PointT pixel2Cam(const PointT& p, const cv::Mat K)
{
    PointT cameraCoord;
    cameraCoord.x = (p.x - K.at<double>(0,2)) / K.at<double>(0,0);
    cameraCoord.y = (p.y - K.at<double>(1,2)) / K.at<double>(1,1);

//    std::cout<<"Point: "<<p.y<<std::endl;
    cameraCoord.z = 1;
    return cameraCoord;

}

int main() {
    // 确定东北天坐标系原点和header的入口

    Utils::new3s_PointXYZ original;
//    vector<GPSInfoEach> gpsInfo_vec;
//    bool flag = ReadHDMap::getGPSInfo(gpsInfo_vec);
//    if (flag)
//    {
//        GPSPointEach gpsPointEach = gpsInfo_vec[0].gpsPoints[0];
//        gpsPointEach.heading;
//        original.set_x(gpsPointEach.points.x);
//        original.set_y(gpsPointEach.points.y);
//        original.set_z(0);
//        header_former = gpsPointEach.heading;
//    }

    double header_start_former = 77.0464;
    original.set_x(22.68085991);
    original.set_y(114.36478212);
    original.set_z(0);


    Utils::new3s_PointXYZ enu_coord_1, enu_coord_2;
    Utils transform;

    // 相机参数加载和相关参数初始化
    const std::string strCameraPath = "../Config/param.yml";
    cv::FileStorage fSettings(strCameraPath, cv::FileStorage::READ);
    double fx = fSettings["Camera.fx"];
    double fy = fSettings["Camera.fy"];
    double cx = fSettings["Camera.cx"];
    double cy = fSettings["Camera.cy"];

    double k1 = fSettings["Camera.k1"];
    double k2 = fSettings["Camera.k2"];
    double p1 = fSettings["Camera.p1"];
    double p2 = fSettings["Camera.p2"];

    double yaw = fSettings["Cam.yaw"];
    double pitch = fSettings["Cam.pitch"];
    double roll = fSettings["Cam.roll"];

    pitch = pitch - M_PI/2.0;

    // +号 -值在上有较好的效果
    // 这里的K和distCoeffs都是全局变量
    K = (cv::Mat_<double>(3, 3) <<
                               fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);
    distCoeffs = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);

//    std::cout<<"K: "<<K<<std::endl;

    // 正常Tcb的值的初始化和计算只需进行一次，因此这里代码应该放在主函数里
    cv::Mat mTbw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat mTcam = (cv::Mat_<double>(4, 1)<<0, 0, 0, 1);
    cv::Mat mRbw;
    Utils poseCompute;

    Eigen::Vector3d ea0(yaw,pitch,roll);
    Eigen::Matrix3d Rcb;
    cv::Mat mRcb;
    Rcb = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
//    Rcb.transpose();
    cv::eigen2cv(Rcb, mRcb);

    mRcb.copyTo(mTcb.rowRange(0, 3).colRange(0, 3));
    mTcb.row(2).col(3) = 1.32;
//    std::cout<<"mTcb: "<<mTcb<<std::endl;
    //////////////////////////////////////////////////////////
    cv::Mat Rall = (cv::Mat_<double>(3, 3) <<
            cos(yaw), -sin(yaw)*cos(pitch), sin(yaw)*sin(pitch),
            sin(yaw), cos(yaw)*cos(pitch), -cos(yaw)*sin(pitch),
            0.0, sin(pitch), cos(pitch));
    std::cout<<"Rall: "<<Rall<<std::endl;


    //////////////////////////////////////////////////////////

    std::ofstream f;
    f.open("../GPStrafficLine.txt");

    //////////////////////////////////////////////////////////

    string scene_id = "20190123112838_3faf30bde99e0f126cda2432ec90a621_4";

//  20190123112838_3faf30bde99e0f126cda2432ec90a621_4
    // 根据scene_id显示此帧gps数据点

    Utils::new3s_PointXYZ  scene_point_start;
    GPSInfoEach gpsInfoEach = ReadHDMap::getGPSInfoBySceneId(scene_id);
    vector<GPSPointEach> points_GPS = gpsInfoEach.gpsPoints;
    vector<ImageBatch> imgs;
    cv::Mat driverlineENU = (cv::Mat_<double>(3, 1)<<0, 0, 0);
    Utils::new3s_PointXYZ driverline_ENU;
    //bool flag = ReadHDMap::getImageBatchBySceneId(scene_id, imgs);
    for (int k = 0; k < points_GPS.size(); ++k) {
        double header_angle = points_GPS[k].heading;
        DetchBatch detchBatch;
        bool flag8 = ReadHDMap::getAllDetectionBatchByIndex(scene_id,k, detchBatch);
        vector<DividerEach> divider_vec = detchBatch.dividerPerFrame.dividerEach_vec;
//        std::cout<<" GPS 1:"<<detchBatch.point.points.x<<" "<<detchBatch.point.points.y<<" "<<detchBatch.point.points.z<<std::endl;

        // 这个是返回的NEU坐标系的点的差
        scene_point_start.set_x(points_GPS[k].points.x);
        scene_point_start.set_y(points_GPS[k].points.y);
        scene_point_start.set_z(points_GPS[k].points.z);
        transform.convertCJC02ToENU(scene_point_start, enu_coord_1, original);
//        std::cout<<"GPS 2:"<<points[k].points.x<<" "<<points[k].points.y<<" "<<points[k].points.z<<std::endl;
//        std::cout<<"enu_coord: "<<enu_coord_1.get_x()<<" "<<enu_coord_1.get_y()<<" "<<enu_coord_1.get_z()<<std::endl;


        //std::cout<<"delta_angle: "<<header_angle<<std::endl;
        cv::Mat tempRstart = poseCompute.convertAngleToR(header_angle);
        tempRstart.copyTo(pose.rowRange(0, 3).colRange(0, 3));
//        std::cout<<"tempRstart: "<<tempRstart<<std::endl;
        pose.row(0).col(3) = enu_coord_1.get_x();
        pose.row(1).col(3) = enu_coord_1.get_y();
        pose.row(2).col(3) = 0;
        std::cout<<"pose: "<<pose<<std::endl;
        // body ------> camera
        for (int i = 0; i < divider_vec.size(); i++) {
            vector<PointT> points_uv =  divider_vec[i].divider_vec;
            for (int j = 0; j <points_uv.size() ; j++) {
                PointT cameraPoint = pixel2Cam(points_uv[j], K);
                cv::Mat Point3dCam = (cv::Mat_<double>(3, 1) << cameraPoint.x, cameraPoint.y, cameraPoint.z);
//                std::cout<<"cameraPoint: "<<cameraPoint.x<<" "<<cameraPoint.y<<" "<<cameraPoint.z<< std::endl;
//                cv::Mat tempCam = (mTcb.rowRange(0, 3).colRange(0, 3)).inv()*Point3dCam;
                cv::Mat tempCam = Rall.inv()*Point3dCam;
//                cv::Mat tempAdd = (mTcb.rowRange(0, 3).colRange(0, 3)).t()*mTcb.col(3).rowRange(0, 3);
                std::cout<<"temp 1: "<<tempCam<<std::endl;
//                tempCam = tempCam * (tempAdd.row(2)/tempCam.row(2));
//                tempCam = tempCam - tempAdd;
                tempCam = tempCam * (-1.32/tempCam.row(2));
//                std::cout<<"tempCam 1: "<<tempCam<<std::endl;
                tempCam.row(2) = 0;
//                tempCam.copyTo(mTcam.col(0).rowRange(0, 3));
//                std::cout<<"tempCam 2: "<<tempCam<<std::endl;
//                std::cout<<"mTcam: "<<mTcam<<std::endl;
//                driverlineENU = (pose.rowRange(0, 3).colRange(0, 3)).t() * (tempCam - pose.col(3).rowRange(0, 3));
                driverlineENU = (pose.rowRange(0, 3).colRange(0, 3)).inv() * tempCam + pose.col(3).rowRange(0, 3);
//                driverlineENU = pose*mTcam;
//                std::cout<<"driverlineENU: "<<driverlineENU<<std::endl;
                driverline_ENU.set_x(driverlineENU.at<double>(0));
                driverline_ENU.set_y(driverlineENU.at<double>(1));
//                std::cout<<" driver_ENU: "<<driverline_ENU.get_x()<<" "<<driverline_ENU.get_y()<<std::endl;
//                f << std::setprecision(11) <<enu_coord_1.get_x() << " "<< enu_coord_1.get_y()<<" ";
//                f << std::setprecision(11) <<driverline_ENU.get_x()<< " "<< driverline_ENU.get_y()<<std::endl;
//                std::cout<<"temp 2: "<<temp<<std::endl;

            }

//            std::cout<<"11111"<<std::endl;

        }
//        cv::Mat camera_pose =  pose * mTcb;
//        for (int j = 0; j < ; ++j) {
//
//        }
//        std::cout<<"camera pose: "<<camera_pose<<std::endl;

    }
//    vector<DividerEach> diver_vec = detchBatch.dividerPerFrame.dividerEach_vec;
//
//
//    std::cout<<"divider point: "<<detchBatch.dividerPerFrame.dividerEach_vec[0].divider_vec[0].x<<" "<<detchBatch.dividerPerFrame.dividerEach_vec[0].divider_vec[0].y<<std::endl;
//    PointT cameraPoint = pixel2Cam(detchBatch.dividerPerFrame.dividerEach_vec[0].divider_vec[0], K);
//    std::cout<<"cameraPoint: "<<cameraPoint.x<<" "<<cameraPoint.y<<" "<<cameraPoint.z<< std::endl;
//    cv::Mat Point3dCam = (cv::Mat_<double>(3, 1) << cameraPoint.x, cameraPoint.y, cameraPoint.z);
//    cv::Mat temp = (mTcb.rowRange(0, 3).colRange(0, 3)).t()*Point3dCam;
////    std::cout<<"temp: "<<temp<<std::endl;
////    std::cout<<"temp 3: "<<temp.row(2)<<std::endl;
//    temp = temp * (1.32/temp.row(2));
//    std::cout<<"temp: "<<temp<<std::endl;
//    cv::Mat Point3dBod = (mTcb.rowRange(0, 3).colRange(0, 3)).t()*(Point3dCam - mTcb.col(3).rowRange(0, 3));

//    std::cout<<"Point3dBod: "<<Point3dBod<<std::endl;




    /************************************************************************************************************
  *   根据image_name/scene_id+index获取指定帧的gps 及检测中心结果
  ************************************************************************************************************/



   /*  DetectionDividerPerCapture detectionDividerPerCapture;
     bool flag7= ReadHDMap::getDetectionBatchBySceneId("20190123112752_7ac6ab9d61d94314188426910d324c39_4", detectionDividerPerCapture);

//     一个pb里面所有的车道线
     vector <DividerEach> divider_vec;
//     一个frame
    for (int i = 0; i < detectionDividerPerCapture.dividerPerFrame_vec.size(); ++i) {
        DetectionDividerPerFrame dividerPerFrame = detectionDividerPerCapture.dividerPerFrame_vec[i];

//        一个devider
        for (int j = 0; j < dividerPerFrame.dividerEach_vec.size(); ++j) {
            DividerEach dividerEach = dividerPerFrame.dividerEach_vec[j];
            vector<PointT> points = dividerEach.divider_vec;
            cout<< "points:"<< points[1].x << " " << points[1].y << " " << points[1].z << endl;
            divider_vec.push_back(dividerEach);
        }

    }
    cout << "divider size:--" << divider_vec.size() << endl;
    cout << "point size:--" << divider_vec[0].divider_vec.size() << endl;
*/
/*
    for (int i = 0; i < ; ++i) {

    }*/

    /************************************************************************************************************
     *    根据scene_id获取GPS数据
     ************************************************************************************************************/
    /*    GPSInfoEach gpsInfoEach = ReadHDMap::getGPSInfoBySceneId(scene_id);
        vector<GPSPointEach> points = gpsInfoEach.gpsPoints;
        for (int k = 0; k < points.size(); ++k) {
            cout << " points:" << points[k].points.x<<"," << points[k].points.y << "," << points[k].points.z << endl;
            cout << " heading:" << points[k].heading << endl;
        }*/


    /************************************************************************************************************
    *    读取整张hdmap高精地图元素
    ************************************************************************************************************/
    /*HDMAP readMap = ReadHDMap::getHDMAP();
    vector<DividerEach> dividerEach = readMap.dividers;
    for (int j = 0; j < dividerEach.size(); ++j) {
        std::cout << "test hd map:" << readMap.dividers[19].divider_vec[5].x << endl;
    }*/

    /************************************************************************************************************
     *    获取每一帧GPS及其对应的图片名称集合,按顺序存储
     ************************************************************************************************************/
    /*    vector<ImageBatch> imageBatch_vec;
        bool flag3 = ReadHDMap::getAllImageBatch(imageBatch_vec);
    //    cout << "dfgfb:"<< dataFiles.size()<<" dfgfd:"<< dataFiles[100].images_vec.size()<< endl;

        if(flag3)
        {
            cout << "image batch ok" << endl;
        }*/

    /************************************************************************************************************
     *    // 根据指定scene_id查询此帧gps数据点,按顺序输出
     ************************************************************************************************************/
    /*    const string  gps_file_folder = "../data/gps";
        vector<string> gps_vec;
        bool flag = calulate::getAllFiles(gps_file_folder,gps_vec);
    //  需要自行判断fileName是否存在 flag 0正确查询  -1 错误查询
        calulate::sortedVector(gps_vec);
        if (flag)
        {
            for (int i = 0; i < gps_vec.size(); ++i) {
    //            cout << "gps files:" << gps_files[i]<<endl;
            }
        }
        int num = gps_vec[10].find_last_of(".");
        string id = gps_vec[10].substr(0, num-14);
        ImageBatch imageBatch;
        bool flag4 = ReadHDMap::getImageBatchBySceneId(id, imageBatch);
        if (flag4)
        {
            for (int i = 0; i < imageBatch.images_vec.size(); ++i) {
    //        cout << "image batch " << imageBatch.images_vec[i] << endl;
            }
        }*/
    /************************************************************************************************************
     *    获取每一个gps点+对应一张图片
     ************************************************************************************************************/
    //    vector<GpsImageBatch> gpsImageBatch_vec;
    //    bool flag5= ReadHDMap::getAllGpsImageBatch(gpsImageBatch_vec);
    //    GpsImageBatch gpsImageBatch;
    //    bool flag6= ReadHDMap::getGpsImageBatchByImageId(scene_id, 2, gpsImageBatch);
    //    cout << "gps heading"<<gpsImageBatch.gpsPoint.heading<<endl;
/************************************************************************************************************
 *    根据scene_id获取车道线检测的中间结果,中间结果是像素坐标(仅x,y有效),需要转换
 ************************************************************************************************************/
    /* DetectionDividerPerCapture detectionDividerPerCapture;
     bool flag7= ReadHDMap::getDetectionBatchBySceneId(scene_id, detectionDividerPerCapture);
 */
/************************************************************************************************************
  *    根据scene_id获取车道线检测的中间结果,中间结果是像素坐标(仅x,y有效),需要转换
  *    1帧scene_id 对应一个TrafficPerCapture,多个TrafficPerFrame,一个Frame对应多个TrafficLight ,一个Tflight对应多个light
  ************************************************************************************************************/
    /*  DetectionTrafficPerCapture detectionTrafficPerCapture;
      bool flag8 = ReadHDMap::getDetctionTrafficlights("20190130161123_c6a0dc163825d772bed42152c9e9b9f0_4", detectionTrafficPerCapture);
      TrafficLightEachShow &trafficLightEachShow = detectionTrafficPerCapture.trafficPerFrame_vec[0].trafficLight_vec[0];
      cout<<"traffic_light geomery:" <<trafficLightEachShow.point_vec.size() << endl;*/


    return 0;
}