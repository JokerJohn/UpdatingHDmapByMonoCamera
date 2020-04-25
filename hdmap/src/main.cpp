
#include "../utils/Utils.h"

#include "../utils/consts.h"  // 所有的相机参数矩阵
#include "../utils/vo.h"
#include "../utils/container.h"


int main() {
    // 确定东北天坐标系原点和header的入口
    Utils::new3s_PointXYZ original;
    original.set_x(22.68085991);
    original.set_y(114.36478212);
    original.set_z(0);

    // 正常Tcb的值的初始化和计算只需进行一次，因此这里代码应该放在主函数里
    cv::Mat mTbw = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat mTwb = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat mTcb = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat mTbc = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat mRbw;
    Eigen::Vector3d ea0(camera_yaw, camera_pitch, camera_roll);
    Eigen::Matrix3d Rcb;
    cv::Mat mRcb, mRbc;
    Rcb = Eigen::AngleAxisd(-ea0[1], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(+M_PI / 2, Eigen::Vector3d::UnitX());
    cv::eigen2cv(Rcb, mRcb);
    mRbc = mRcb.t();

//    Eigen::Vector3d mt = Eigen::Vector3d(0, 1, 0);
//    Eigen::Vector3d value;
//    value = Rcb*mt;
//    std::cout<<"value: "<<value<<std::endl;


//    cv::Mat mRbc;
//    mRbc = (cv::Mat_<double>(3, 3) <<
//            cos(camera_yaw),sin(camera_yaw), 0,
//            -cos(camera_pitch)*sin(camera_yaw), cos(camera_pitch)*cos(camera_yaw), sin(camera_pitch),
//            sin(camera_pitch) *sin(camera_yaw), -sin(camera_pitch)*cos(camera_yaw), cos(camera_pitch));
//    mRbc = (cv::Mat_<double>(3, 3) <<
//            cos(camera_yaw),-sin(camera_yaw), 0,
//            cos(camera_pitch)*sin(camera_yaw), cos(camera_pitch)*cos(camera_yaw), -sin(camera_pitch),
//            sin(camera_pitch) *sin(camera_yaw), sin(camera_pitch)*cos(camera_yaw), cos(camera_pitch));

//    cv::Mat mRcb;
//    mRcb = mRbc.inv();
    mRbc.copyTo(mTbc.rowRange(0, 3).colRange(0, 3));
    mTbc.row(2).col(3) = 1.32;
    mTcb = mTbc.inv();
    std::cout << "mTcb" << mTcb << std::endl;

    /************ Test VO  between img1 and img2 **************/
    string basedir = "/home/jianghc/testproject/hdmap/data/vo_test/undistort/";
    string scene_id = "20190123112924_d5096201da81bc8443f1aabccef099c0_4";
    Mat img_1 = imread(basedir + scene_id + "_324.jpeg", CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(basedir + scene_id + "_329.jpeg", CV_LOAD_IMAGE_COLOR);
    /// get boxes of img_1 and img_2
    vector <BBox> boxes;
    string image_id = scene_id + "_119";
//    readBoxes(image_id, "traffic light", boxes);
    readBoxes(image_id, "traffic sign", boxes);
    cout << "boxes size: " << boxes.size() << endl;
    for (BBox &box : boxes) {
        rectangle(img_1, cvPoint(box.xmin, box.ymin), cvPoint(box.xmax, box.ymax), Scalar(0, 0, 255), 1, 1, 0);
    }
    /// get poses from gps
    int image_index_1, image_index_2;
    ReadHDMap::getIndexByImageId(scene_id + "_324", image_index_1);
    ReadHDMap::getIndexByImageId(scene_id + "_329", image_index_2);
    cout << "image_index_1: " << image_index_1 << endl;
    cout << "image_index_2: " << image_index_2 << endl;
    vector <cv::Mat> poses = getPosesBySceneId(scene_id, original, mTcb);
    std::cout << "pose size： " << poses.size() << std::endl;
    cv::Mat pose1 = poses[image_index_1];
    cv::Mat pose2 = poses[image_index_2];

    ///-- 提取图像中的特征点且初步筛选
    vector <KeyPoint> keypoints_1, keypoints_2;
    vector <DMatch> good_matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, good_matches); //matches在该函数内部定义的
    ///-- 估计两张图像间运动
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, good_matches, R, t, K);
    ///-- 只关心目标区域内的点
    vector <DMatch> target_matches;
//    for ( DMatch m : good_matches ) {
//        int x = keypoints_1[m.queryIdx].pt.x;
//        int y = keypoints_1[m.queryIdx].pt.y;
//        if (x > boxes[0].xmin && x < boxes[0].xmax && y > boxes[0].ymin && y < boxes[0].ymax) {
//            target_matches.push_back(m);
//        }
//    }
    for (DMatch m : good_matches) {
//        int x = keypoints_1[m.queryIdx].pt.x;
//        int y = keypoints_1[m.queryIdx].pt.y;
//        if (x > boxes[1].xmin && x < boxes[1].xmax && y > boxes[1].ymin && y < boxes[1].ymax) {
//            target_matches.push_back(m);
//        }
        target_matches.push_back(m);
    }
    cout << "good matches: " << good_matches.size() << endl;
    cout << "target matches: " << target_matches.size() << endl;
    ///-- 三角化 - 通过gps算的t作为绝对尺度
    Mat T1, T2;
    // 第1种思路：
    T1 = (Mat_<double>(3, 4) <<
                             pose1.at<double>(0, 0), pose1.at<double>(0, 1), pose1.at<double>(0, 2), pose1.at<double>(0,
                                                                                                                      3),
            pose1.at<double>(1, 0), pose1.at<double>(1, 1), pose1.at<double>(1, 2), pose1.at<double>(1, 3),
            pose1.at<double>(2, 0), pose1.at<double>(2, 1), pose1.at<double>(2, 2), pose1.at<double>(2, 3));
    cv::Mat RT1 = T1.colRange(0, 3).rowRange(0, 3);
//    T1 = K * T1;
    T2 = (Mat_<double>(3, 4) <<
                             pose2.at<double>(0, 0), pose2.at<double>(0, 1), pose2.at<double>(0, 2), pose2.at<double>(0,
                                                                                                                      3),
            pose2.at<double>(1, 0), pose2.at<double>(1, 1), pose2.at<double>(1, 2), pose2.at<double>(1, 3),
            pose2.at<double>(2, 0), pose2.at<double>(2, 1), pose2.at<double>(2, 2), pose2.at<double>(2, 3));
//    T2 = K * T2;
    cv::Mat RT2 = T2.colRange(0, 3).rowRange(0, 3);
    cv::Mat convertR;
    convertR = RT1.t() * RT2;
    cout << "convertR: " << convertR << endl;
    cout << "T1: " << T1 << endl << "T2: " << T2 << endl;
//    cout << "Pose1: " << pose1 << endl << "Pose2: " << pose2 << endl;
    //这里第四列减完之后要转到T1的相机坐标系中。
    vector <Point3d> points;

    std::cout << "Pose1: " << pose1.inv() << std::endl;// Twc
    triangulation(keypoints_1, keypoints_2, K, target_matches, T1, T2, points);
    std::cout << "Points: " << points[0] << std::endl;
//    cv::Mat pointout = (cv::Mat_<double>(4, 1) << 0.097*3.77, -0.236*3.77, -0.96*3.77, 1);
//    std::cout<<"enu Points"<<pose1.inv() * pointout;

    //-- 验证三角化点与特征点的重投影关系
    double depth_total = 0;
    for (int i = 0; i < target_matches.size(); i++) {
        Point2d pt1_cam = pixel2cam(keypoints_1[target_matches[i].queryIdx].pt, K);
        Point2d pt1_cam_3d(points[i].x / points[i].z, points[i].y / points[i].z); //坐标归一化

        cout << "point in the first camera frame: " << pt1_cam << endl;
        cout << "point projected from 3D " << pt1_cam_3d << ", depth = " << points[i].z << endl;
        depth_total += points[i].z;
        putText(img_1, to_string(points[i].z), keypoints_1[target_matches[i].queryIdx].pt, cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(0, 0, 255), 1, 8, 0);

//        // 第二个图
////        Point2f pt2_cam = pixel2cam( keypoints_2[ target_matches[i].trainIdx ].pt, K );
////        Mat pt2_trans = R*( Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z ) + t;
////        pt2_trans /= pt2_trans.at<double>(2,0);
////        cout << "point in the second camera frame: " << pt2_cam <<endl;
////        cout << "point reprojected from second frame: " << pt2_trans.t() <<endl;
////        cout << endl;
    }
//    cout << "depth average: " << depth_total / target_matches.size() << endl;
    imshow("img_1", img_1);
    waitKey(0);
    return 0;
}


/************ For single image (specified by gps_index & ref_img_index): GET DEPTH MAP *********/
//    int gps_index, ref_img_index;
//    vector<ImageBatch> imageBatch_vec;
//    bool flag = ReadHDMap::getAllImageBatch(imageBatch_vec);
//    cout << "imageBatch_vec size: " << imageBatch_vec.size() << endl;
//    cout << imageBatch_vec[23].scene_id << endl;
//    for (int i = 0; i < imageBatch_vec.size(); i++) {
//        if (imageBatch_vec[i].scene_id == "20190129104232_e58b8b3d6ac89c45c2a50d84ba822803_4") {
//            gps_index = i;
//            cout << "gps_index: " << i << endl;
//            break;
//        }
//    }
//    string image_id = "20190129104232_e58b8b3d6ac89c45c2a50d84ba822803_4_344";
//    ReadHDMap::getIndexByImageId(image_id, ref_img_index);
//    cout << "ref_img_index: " << ref_img_index << endl;
//
//    cv::Mat depth( image_height, image_width, CV_64F, init_depth );             // 初始化深度图
//    cv::Mat depth_cov( image_height, image_width, CV_64F, init_cov2 );          // 初始化深度方差图
//    Image3D image3D;
//    string mode = "full";
//    getDepthMapOfSingleImage(gps_index, ref_img_index, imageBatch_vec, original, depth, depth_cov, image3D, mode);

/************ For single scene (specified by gps_index): Calculate Divider Points Depth *********/
//    for (int i = 0; i < imageBatch_vec.size(); i++) {
//        if (imageBatch_vec[i].scene_id == "20190123112838_3faf30bde99e0f126cda2432ec90a621_4") {
//            cout << "gps index is : " << i << endl;
//        }
//    }
//这里查出来的gps_index是1.

//    int gps_index = 2;
//    string scene_id = imageBatch_vec[gps_index].scene_id;
//    string txt_name = "/home/joey/work/zhangjing/deecamp/depth_results/divider/2/divider_all.txt";
//    ofstream out(txt_name);
//    for (int ref_img_index = 0; ref_img_index < imageBatch_vec[gps_index].images_vec.size(); ref_img_index++) {
//        string image_id = imageBatch_vec[gps_index].images_vec[ref_img_index];
//        cout << "image_id: " << image_id << endl;
//
//        // get pose of ref_img
//        vector<cv::Mat> poses = getPosesBySceneId(scene_id, original);
//        cv::Mat pose = poses[ref_img_index];
//        cv::Mat R, t;
//        R = (cv::Mat_<double>(3, 3) << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2),
//                pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2),
//                pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2));
//        t = (cv::Mat_<double>(3, 1) << pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
//        //cout << "camera R: " << R << endl << "camera t: " << t << endl;
//        cv::Mat carPose = pose * mTcb.inv();
//        //cout << "car pose: " << carPose << endl;
//
//        // get divider points of ref_img, estimate depth and transform back to enu
//        DetchBatch detchBatch;
//        bool flag_detection = ReadHDMap::getAllDetectionBatchByIndex(scene_id, ref_img_index, detchBatch);
//        vector<DividerEach> divider_vec = detchBatch.dividerPerFrame.dividerEach_vec;
//
//        if (divider_vec.size() == 0) continue;
//
//        int counter = 0;
//        //cout << "divider_vec.size : " << divider_vec.size() << endl;
//        for (int i = 0; i < divider_vec.size(); i++) {
//            vector<PointT> points = divider_vec[i].points_vec;
//            for (int j = 0; j < points.size(); ++j) {
//                // calculate depth for divider point specially based on triangle
//                Point3D point3D;
//                calDepthForDivider(points[j], point3D);
//                cout << "divider point (pixel): " << "x: " << point3D.x << " y: " << point3D.y << " z(depth): "
//                     << point3D.d << endl;
//
//                // transform the pixel divider point into enu
//                Point3DEnu point3DEnu;
//                pixel3D2enu(point3D, R, t, point3DEnu);
//                cout << "divider point (enu): " << " x: " << point3DEnu.x << " y: " << point3DEnu.y << " z(height): "
//                     << point3DEnu.z << endl;
//
//                string line_to_write =
//                        double2Str(point3DEnu.x) + " " + double2Str(point3DEnu.y) + " " + double2Str(point3DEnu.z);
//                out << line_to_write << endl;
//                counter++;
//            }
//        }
//        cout << "--------" << endl;
//    }



/************ For single scene(specified by gps_index): GET ALL DEPTH MAPS ********/
//    vector<Image3D> images_gps;
//    getDepthMapOfSingleScene(23, imageBatch_vec, original, images_gps); //23代表测试的gps_index
//    cout << "images_gps : " << imageBatch_vec[23].scene_id << " size: " << images_gps.size() << endl;
//    cout << "peek images_gps - image_id: " << images_gps[3].image_id << endl;
//    cout << "peek images_gps - pose: " << images_gps[3].pose << endl;

//    //    getDepthMapOfSingleScene(23, imageBatch_vec, original, images_gps); //23代表测试的gps_index
//    cout << "images_gps : " << imageBatch_vec[23].scene_id << " size: " << images_gps.size() << endl;
//    cout << "peek images_gps - image_id: " << images_gps[3].image_id << endl;
//    cout << "peek images_gps - pose: " << images_gps[3].pose << endl;

//    /***** transform recovered point with depth into enu coord and write into txt for smartFish *****/
//    for (Image3D& img_3d : images_gps) {
//        if (img_3d.points.size() == 0) continue;
//        string image_id = img_3d.image_id;
//        string txt_name = "/home/joey/work/zhangjing/deecamp/depth_results/23/" + image_id + ".txt";
//        ofstream out(txt_name);
//        Mat pose = img_3d.pose;
//        cv:: Mat R, t;
//        R = (cv::Mat_<double>(3, 3) << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2),
//                                       pose.at<double>(1, 0), pose.at<double>(1,1), pose.at<double>(1, 2),
//                                       pose.at<double>(2, 0), pose.at<double>(2,1), pose.at<double>(2, 2));
//        t = (cv::Mat_<double>(3, 1) << pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
//
//        cout << "R: " << R << endl << "t: " << t << endl;
//
//        for (Point3D& point_with_depth : img_3d.points) {
//            Point3DEnu point3DEnu;
//            pixel2enu(point_with_depth, R, t, point3DEnu);
//
//            string line_to_write = double2Str(point3DEnu.x) + " " + double2Str(point3DEnu.y) + " " + double2Str(point3DEnu.z);
//            out << line_to_write << endl;
//
//            cout << "enu coord: " << line_to_write << endl;
//            cout << "--------" << endl;
//       }
//       cout << "finished writing " << img_3d.points.size() << " points into " << txt_name << endl;
//    }


/////之前的主函数

//int main() {
//    // 确定东北天坐标系原点和header的入口
//    Utils::new3s_PointXYZ original;
//    double header_start_former = 77.0464;
//    original.set_x(22.68085991);
//    original.set_y(114.36478212);
//    original.set_z(0);
//
//    // 这里的K和distCoeffs都是全局变量
//    K = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
//    distCoeffs = (cv::Mat_<float>(4, 1) << k1, k2, p1, p2);
//
//    // 正常Tcb的值的初始化和计算只需进行一次，因此这里代码应该放在主函数里
//    cv::Mat mTbw = cv::Mat::eye(4, 4, CV_64F);
//    cv::Mat mRbw;
//    Eigen::Vector3d ea0(yaw, pitch, roll);
//    Eigen::Matrix3d Rcb;
//    cv::Mat mRcb;
//    Rcb = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
//    cv::eigen2cv(Rcb, mRcb);
//    mRcb.copyTo(mTcb.rowRange(0, 3).colRange(0, 3));
//    mTcb.row(2).col(3) = 1.32;
//
//    vector<ImageBatch> imageBatch_vec;
//    bool flag = ReadHDMap::getAllImageBatch(imageBatch_vec);
//    cout << "imageBatch_vec size: " << imageBatch_vec.size() << endl;
//
//    /************ TEST on single image: get depth map by gps_index & ref_img_index *********/
//    int gps_index = 23, ref_img_index = 3;
//    cv::Mat depth( image_height, image_width, CV_64F, init_depth );             // 初始化深度图
//    cv::Mat depth_cov( image_height, image_width, CV_64F, init_cov2 );          // 初始化深度方差图
//    Image3D image3D;
//    getDepthMapOfSingleImage(gps_index, ref_img_index, imageBatch_vec, original, depth, depth_cov, image3D);
//
//    /************ TEST on single scene: get all depth maps by gps_index ********/
//    vector<Image3D> images_gps;
//    getDepthMapOfSingleScene(23, imageBatch_vec, original, images_gps); //23代表测试的gps_index
//    cout << "images_gps : " << imageBatch_vec[23].scene_id << " size: " << images_gps.size() << endl;
//    cout << "peek images_gps - image_id: " << images_gps[3].image_id << endl;
//    cout << "peek images_gps - pose: " << images_gps[3].pose << endl;
//    for (int i = 0; i < images_gps.size(); i++) {
//        cout << "peek points size: " << images_gps[i].points.size() << endl;
//        if (images_gps[i].points.size() != 0) {
//            cout << "peek depth: " << images_gps[i].points[0].d << endl;
//        }
//    }
//
//    return 0;
//}




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
/*  vector<ImageBatch> imageBatch_vec;
  bool flag3 = ReadHDMap::getAllImageBatch(imageBatch_vec);
//    cout << "dfgfb:"<< dataFiles.size()<<" dfgfd:"<< dataFiles[100].images_vec.size()<< endl;

  if(flag3)
  {
      cout << "image batch ok" << endl;
  }*/

/************************************************************************************************************
 *    // 根据指定scene_id查询此帧gps数据点,按顺序输出
 ************************************************************************************************************/
//        const string  gps_file_folder = "../data/gps";
//        vector<string> gps_vec;
//        bool flag = calulate::getAllFiles(gps_file_folder,gps_vec);
//    //  需要自行判断fileName是否存在 flag 0正确查询  -1 错误查询
//        calulate::sortedVector(gps_vec);
//        if (flag)
//        {
//            for (int i = 0; i < gps_vec.size(); ++i) {
//    //            cout << "gps files:" << gps_files[i]<<endl;
//            }
//        }
//        int num = gps_vec[10].find_last_of(".");
//        string id = gps_vec[10].substr(0, num-14);
//        ImageBatch imageBatch;
//        bool flag4 = ReadHDMap::getImageBatchBySceneId(id, imageBatch);
//        if (flag4)
//        {
//            for (int i = 0; i < imageBatch.images_vec.size(); ++i) {
//    //        cout << "image batch " << imageBatch.images_vec[i] << endl;
//            }
//        }
/************************************************************************************************************
 *    获取每一个gps点+对应一张图片
 ************************************************************************************************************/
//    vector<GpsImageBatch> gpsImageBatch_vec;
//    bool flag5= ReadHDMap::getAllGpsImageBatch(gpsImageBatch_vec);
//    GpsImageBatch gpsImageBatch;
//    bool flag6= ReadHDMap::getGpsImageBatchByImageId(scene_id, 2, gpsImageBatch);
//    cout << "gps heading"<<gpsImageBatch.gpsPoint.heading<<endl;