//
// Created by catalina on 2019/8/1.
//
#include "test.h"
#include "utils/Utils.h"
//#include "utils/read_detection.h"
//#include "utils/build_depth.h"
#include "utils/container.h"
#include "utils/pix2enu.h"
using namespace std;

//cv::Mat K;
cv::Mat distCoeffs;
cv::Mat Rcb;
//cv::Mat mTcb = cv::Mat::eye(4, 4, CV_64F);
double header_former;
//cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
string scene_id = "20190123112838_3faf30bde99e0f126cda2432ec90a621_4";

int main() {

/*    // 确定东北天坐标系原点和header的入口
    Utils::new3s_PointXYZ original;
    double header_start_former = 77.0464;
    original.set_x(22.68085991);
    original.set_y(114.36478212);
    original.set_z(0);

    // 这里的K和distCoeffs都是全局变量
    K = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    distCoeffs = (cv::Mat_<float>(4, 1) << k1, k2, p1, p2);

    // 正常Tcb的值的初始化和计算只需进行一次，因此这里代码应该放在主函数里
    cv::Mat mTbw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat mRbw;
    Eigen::Vector3d ea0(yaw, pitch, roll);
    Eigen::Matrix3d Rcb;
    cv::Mat mRcb;
    Rcb = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ea0[1],
                                                                                  Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
    cv::eigen2cv(Rcb, mRcb);
    mRcb.copyTo(mTcb.rowRange(0, 3).colRange(0, 3));
    mTcb.row(2).col(3) = 1.32;

    vector<ImageBatch> imageBatch_vec;
    bool flag = ReadHDMap::getAllImageBatch(imageBatch_vec);
    cout << "imageBatch_vec size: " << imageBatch_vec.size() << endl;

//    *********** TEST on single image: get depth map by gps_index & ref_img_index ********
    int gps_index = 23, ref_img_index = 3;
    cv::Mat depth(image_height, image_width, CV_64F, init_depth);             // 初始化深度图
    cv::Mat depth_cov(image_height, image_width, CV_64F, init_cov2);          // 初始化深度方差图
    Image3D image3D;
    getDepthMapOfSingleImage(gps_index, ref_img_index, imageBatch_vec, original, depth, depth_cov, image3D);
//}*/
//    *********** TEST on single scene: get all depth maps by gps_index *******
    /* vector<Image3D> images_gps;
     getDepthMapOfSingleScene(23, imageBatch_vec, original, images_gps); //23代表测试的gps_index
     cout << "images_gps : " << imageBatch_vec[23].scene_id << " size: " << images_gps.size() << endl;
     cout << "peek images_gps - image_id: " << images_gps[3].image_id << endl;
     cout << "peek images_gps - pose: " << images_gps[3].pose << endl;
     for (int i = 0; i < images_gps.size(); i++) {
         cout << "peek points size: " << images_gps[i].points.size() << endl;*/
//    }

    // ------------------ test cam2enu --------------
    /* cout << "Welcome to Summoner's Rift" << endl;
     PointInt p;
     p.x = 1, p.y = 2, p.label = 3;
     PointDouble cam;

     cv::Mat K;
     double fx = 481.2, fy = -480.0, cx = 319.5, cy = 239.5;
     K = (cv::Mat_<double>(3, 3) <<
                                 fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0);
     pixel2cam(p, cam, K);


     cout << " First blood." << endl;
     cout << " x: " << cam.x << " y: " << cam.y << " label: " << cam.label << endl;

     // cam2enu
     PointDepth coors_cam, coors_enu;
     coors_cam.x = cam.x;
     coors_cam.y = cam.y;
     coors_cam.z = 1.3;
     coors_cam.label = cam.label;
     cv:: Mat R, t;
     R = (cv::Mat_<double>(3, 3) <<
                                 1.0, 2.0, 3.0,
             4.0, 3.0, 1.0,
             2.0, 5.0, 1.2);
     t = (cv::Mat_<double>(3, 1) << 1.0, 2.0, 3.0);
     cam2enu(coors_cam, coors_enu, R, t);
     cout << "Double kill." << endl;
     cout << " x: " << coors_enu.x << " y: " << coors_enu.y << " depth: " << coors_enu.z << " label: " << coors_enu.label << endl;

     cout << "Victory" << endl;*/
    // ------------------ test cam2enu --------------


//----------------------------------------------------------------------------------------------------------------------
    // 确定东北天坐标系原点和header的入口
/*    Utils::new3s_PointXYZ original;
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
    Utils transform;*/

/*    // 相机参数加载和相关参数初始化
    const std::string strCameraPath = "../config/param.yml";
    cv::FileStorage fSettings(strCameraPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    float k1 = fSettings["Camera.k1"];
    float k2 = fSettings["Camera.k2"];
    float p1 = fSettings["Camera.p1"];
    float p2 = fSettings["Camera.p2"];

    float yaw = fSettings["Cam.yaw"];
    float pitch = fSettings["Cam.pitch"];
    float roll = fSettings["Cam.roll"];

    // 这里的K和distCoeffs都是全局变量
    K = (cv::Mat_<float>(3, 3) <<
                               fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);
    distCoeffs = (cv::Mat_<float>(4, 1) << k1, k2, p1, p2);

    // 正常Tcb的值的初始化和计算只需进行一次，因此这里代码应该放在主函数里
    cv::Mat mTbw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat mRbw;
    Utils poseCompute;

    Eigen::Vector3d ea0(yaw,pitch,roll);
    Eigen::Matrix3d Rcb;
    cv::Mat mRcb;
    Rcb = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(ea0[1],
            Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
    cv::eigen2cv(Rcb, mRcb);

    mRcb.copyTo(mTcb.rowRange(0, 3).colRange(0, 3));
    mTcb.row(2).col(3) = 1.32;

    //////////////////////////////////////////////////////////
    string scene_id = "20190123112838_3faf30bde99e0f126cda2432ec90a621_4";

    //    根据scene_id显示此帧gps数据点
    Utils::new3s_PointXYZ  scene_point_start;
    GPSInfoEach gpsInfoEach = ReadHDMap::getGPSInfoBySceneId(scene_id);
    vector<GPSPointEach> points = gpsInfoEach.gpsPoints;
    vector<ImageBatch> imgs;
    //bool flag = ReadHDMap::getImageBatchBySceneId(scene_id, imgs);
    for (int k = 0; k < points.size(); ++k) {
        double header_angle = points[k].heading;

        // 这个是返回的NEU坐标系的点的差
        scene_point_start.set_x(points[k].points.x);
        scene_point_start.set_y(points[k].points.y);
        scene_point_start.set_z(points[k].points.z);
        transform.convertCJC02ToENU(scene_point_start, enu_coord_1, original);
        std::cout << "enu_coord1: " << enu_coord_1.get_x() << " " << enu_coord_1.get_y() << " " << enu_coord_1.get_z()
                  << std::endl;

        std::cout << "delta_angle: " << header_angle << std::endl;
        cv::Mat tempRstart = poseCompute.convertAngleToR(header_angle);
        tempRstart.copyTo(pose.rowRange(0, 3).colRange(0, 3));
        pose.row(0).col(3) = enu_coord_1.get_x();
        pose.row(1).col(3) = enu_coord_1.get_y();
        pose.row(2).col(3) = 0;
        std::cout << "pose: " << pose << std::endl;
        cv::Mat camera_pose = mTcb * pose;
        //std::cout<<"camera_pose: "<<camera_pose<<std::endl;
//    }
    }*/
//--------------------------------------------------------------------------------------------------------------------
//坐标转换
/*    PointT point;
    point.x =  22.68214216 ;
    point.y = 114.37055086;
    point.z =   6.00000000;

    PointT pointT = ReadHDMap::transform2ENU(point);
    cout << pointT.x << "," << pointT.y << "," << pointT.z;*/

/************************************************************************************************************
 *    根据scene_id获取GPS数据
 ************************************************************************************************************/
//    取GPS第一个点 (起始位置)
/*    vector<GPSInfoEach> gpsInfo_vec;
    bool flag = ReadHDMap::getGPSInfo(gpsInfo_vec);

//    string txt_name = "/home/catalina/gps.txt";
//    ofstream out(txt_name);
    ofstream file("../GPS.txt");
    for (int i = 0; i < gpsInfo_vec.size(); i++) {
//        cout << "gps points number:" << gpsInfo_vec[i].gpsPoints.size() << endl;

        for (int j = 0; j < gpsInfo_vec[i].gpsPoints.size(); j++) {
            GPSPointEach gpsPointEach = gpsInfo_vec[i].gpsPoints[j];
            file <<  setprecision(12) << gpsPointEach.points.x << "," << gpsPointEach.points.y << ","
                 << gpsPointEach.points.z << ","<< gpsPointEach.heading<<endl;

//            string line_to_write = gpsPointEach.points.x + "," + double2Str(gpsPointEach.points.y) + "," +
//                                   double2Str(gpsPointEach.points.z);
//            out << line_to_write << endl;
        }

    }
    file.close();*/
//}
//    cout << "finished writing " << img_3d.points.size() << " points into " << txt_name << endl;


    /* *//***** transform recovered point with depth into enu coord and write into txt for smartFish *****//*
    for (Image3D& img_3d : images_gps) {
        if (img_3d.points.size() == 0) continue;
        string image_id = img_3d.image_id;
        string txt_name = "/home/joey/work/zhangjing/deecamp/depth_results/23/" + image_id + ".txt";
        ofstream out(txt_name);
        Mat pose = img_3d.pose;
        cv:: Mat R, t;
        R = (cv::Mat_<double>(3, 3) << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2),
                pose.at<double>(1, 0), pose.at<double>(1,1), pose.at<double>(1, 2),
                pose.at<double>(2, 0), pose.at<double>(2,1), pose.at<double>(2, 2));
        t = (cv::Mat_<double>(3, 1) << pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

        cout << "R: " << R << endl << "t: " << t << endl;

        for (Point3D& point_with_depth : img_3d.points) {
            Point3DEnu point3DEnu;
            pixel2enu(point_with_depth, R, t, point3DEnu);

            string line_to_write = double2Str(point3DEnu.x) + " " + double2Str(point3DEnu.y) + " " + double2Str(point3DEnu.z);
            out << line_to_write << endl;

            cout << "enu coord: " << line_to_write << endl;
            cout << "--------" << endl;
        }
        cout << "finished writing " << img_3d.points.size() << " points into " << txt_name << endl;*/
//    }

    /*  if (flag)
      {
          GPSPointEach gpsPointEach = gpsInfo_vec[0].gpsPoints[0];
          cout<< "gpsPointEach: "<<gpsPointEach.points.x << " y, " << gpsPointEach.points.y<< " z, " << gpsPointEach.points.z << endl;
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
  /*  HDMAP readMap = ReadHDMap::getHDMAP();
    vector<DividerEach> dividerEach = readMap.dividers;
    for (int j = 0; j < dividerEach.size(); ++j) {
        vector <PointT> divider_vec = dividerEach[j].divider_vec;
        std::cout << setprecision(14)<< "" << dividerEach[j].id << "," << dividerEach[j].type << ","<< dividerEach[j].color << ",";
        for (int i = 0; i < divider_vec.size() ; ++i) {
            std::cout<<setprecision(14)<< divider_vec[i].x << "," << divider_vec[i].y << ","<< divider_vec[i].z << ",";
        }
        cout << endl;
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
//        }
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
   /* DetectionTrafficPerCapture detectionTrafficPerCapture;
    bool flag8 = ReadHDMap::getDetctionTrafficlights("20190130161123_c6a0dc163825d772bed42152c9e9b9f0_4", detectionTrafficPerCapture);
    TrafficLightEachShow &trafficLightEachShow = detectionTrafficPerCapture.trafficPerFrame_vec[0].trafficLight_vec[0];
    cout<<"traffic_light geomery:" <<trafficLightEachShow.point_vec.size() << endl;*/
/************************************************************************************************************
  *   根据image_name/scene_id+index获取指定帧的gps 及检测中心结果
  ************************************************************************************************************/

/*    int index;
    bool flsg = ReadHDMap::getIndexByImageId("20190123112752_7ac6ab9d61d94314188426910d324c39_4_021",index);
    cout << "index " << index;*/

/************************************************************************************************************
  *   根据image_name/scene_id+index获取指定帧的divider检测中心结果
  ************************************************************************************************************/
    vector<DividerEach> divider_vec;
    pix2enu2(divider_vec, scene_id, 3);
    for (int i = 0; i < divider_vec.size() ; ++i) {
        vector<PointT> points_vec = divider_vec[i].points_vec;
        cout << " divider points size:" << divider_vec[i].points_vec.size() << endl;
    }
/*
    DetchBatch detchBatch;
    bool flag9 = ReadHDMap::getAllDetectionBatchByIndex(scene_id, 2, detchBatch);

    vector<DividerEach> divider_vec = detchBatch.dividerPerFrame.dividerEach_vec;
    for (int i = 0; i < divider_vec.size(); ++i) {
        vector<PointT> points =divider_vec[i].points_vec;
        for (int j = 0; j < points.size(); ++j) {
            points[j].x;
        }
    }
    cout<<"batch point:" <<detchBatch.point.points.x <<" " <<detchBatch.point.points.y<<" " <<detchBatch.point.points.z << endl;
    cout<<"batch image_name:" <<detchBatch.image_name <<endl;
    cout <<"trafficPerFrame " << detchBatch.trafficPerFrame.trafficLight_vec.size() << endl;
    cout <<"dividerPerFrame " << detchBatch.dividerPerFrame.dividerEach_vec.size() << endl;
*/

    return 0;
}
