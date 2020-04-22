//
// Created by joey on 19-8-4.
//

#ifndef DEECAMP_CONTAINER_H
#define DEECAMP_CONTAINER_H

#include <sophus/se3.h>
using Sophus::SE3;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <iostream>
#include "Utils.h"
#include <math.h>
#include <iomanip>

#include "read_hdmap.h"
//#include "build_depth.h"
#include "consts.h"

#define PI 3.14159265

cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);

string double2Str(double& value) {
    ostringstream oss;
    oss << value;
    return oss.str();
}

bool is_file_empty(std::ifstream& pFile)
{
    return pFile.peek() == std::ifstream::traits_type::eof();
}

bool readBoxes(const string& image_id, const string& type, vector<BBox>& boxes) {
    const string basedir = "../data/detection_result_1/vo_test/undistort/";
    string file_name;
    if (type == "traffic light") {
        file_name = basedir + image_id + ".txt";
    }
    if (type == "lane") {
        file_name = basedir + "lane/" + image_id + "@lane.txt";
    }
    if (type == "traffic sign"){
        file_name = basedir  + image_id + "@traffic_sign.txt";
    }
    cout << "filename: " << file_name << endl;
    string line;
    ifstream fin(file_name);
    if ( !fin ) return false; //是否open
    if ( is_file_empty(fin) ) return false; //判断文件是否为空
    while (getline(fin, line)) {
        istringstream iss(line);
        BBox box;
        cout << "line: " << line << endl;
        iss >> box.xmin >> box.ymin >> box.xmax >> box.ymax >> box.label >> box.score;
        boxes.push_back(box);
    }
    return true;
}

bool readDetectionFiles(const string& image_id, vector<PointInt>& img_coors)
{
    // file_name: 检测得到txt 文件的文件名
    // img_coors 检测到的像素对应的像素坐标和label
    const string file_path = "../data/detection_result_1/vo_test";
    string lane_file_name = file_path + "lane/" + image_id + "@divider.txt";
    cout << "lane_file_name: " << lane_file_name << endl;
//    string light_file_name = file_path + "traffic_light_bbox/" + image_id + "@traffic_light.txt";
    string line;

    // 读取车道线
    ifstream lane_fin( lane_file_name);
    if ( !lane_fin ) return false;
    if ( is_file_empty(lane_fin)) return false; //判断文件是否为空

    while (std::getline(lane_fin, line))
    {
        std::istringstream iss(line);
        // 数据格式：x, y, label
        int x, y, label;
        if (!(iss >> x >> y >> label)) break;
        iss >> x >> y >> label;

        PointInt coor_label;
        coor_label.x = x;
        coor_label.y = y;
        coor_label.label = label;

        img_coors.push_back(coor_label);

        if ( !lane_fin.good() ) break;
    }

//    // 读取红绿灯
//    ifstream light_fin( light_file_name);
//    if ( !light_fin ) return false;
//    if ( is_file_empty(light_fin) ) return false; //判断文件是否为空
//
//    while (std::getline(light_fin, line))
//    {
//        std::istringstream iss(line);
//        // 数据格式：x, y, label
//        int xmin, ymin, xmax, ymax, label;
//        double score;
//        if (!(iss >> xmin >> ymin >> xmax >> ymax >> label >> score)) break;
//
//        for(int i=xmin; i<=xmax; i++){
//            for (int j=ymin; j<=ymax; j++){
//                PointInt coor_label;
//                coor_label.x = i;
//                coor_label.y = j;
//                coor_label.label = label;
//                img_coors.push_back(coor_label);
//            }
//        }
//
//        if ( !light_fin.good() ) break;
//    }
    return true;
}

void pixel3D2enu (const Point3D& point3D, const cv:: Mat& R, const cv:: Mat& t, Point3DEnu& point3DEnu) {
    // pixel2cam - 这里很可能有错误
    double x = (point3D.x - K.at<double>(0,2)) / K.at<double>(0,0) * focal_length;
    double y = (point3D.y - K.at<double>(1,2)) / K.at<double>(1,1) * focal_length;
    double d = point3D.d - focal_length; //如果要刻意追求精度, 我觉得应该是要减去focal length的.

    cout << "pixel x: " << x << " pixel y: " << y << " pixel depth: " << d << endl;

    // cam2enu
    cv::Mat pt_trans = R.t()*( Mat_<double>(3,1) << x, d, -y) + t; //考虑到cam坐标系和当地enu坐标系不平行。
//    cv::Mat pt_trans = R.t()*( Mat_<double>(3,1) << x, y, d) + t;
    point3DEnu.x = pt_trans.at<double>(0, 0);
    point3DEnu.y = pt_trans.at<double>(1, 0);
    point3DEnu.z = pt_trans.at<double>(2, 0);
    point3DEnu.label = point3D.label;
}

//// specially calculate depth of points in divider
void calDepthForDivider(PointT& divider_point_2d, Point3D& point_with_depth) {
    int u = (int) divider_point_2d.x;
    int v = (int) divider_point_2d.y;

    double x_2d = (u - cx) / alpha;
    double y_2d = (v - cy) / beta;
//    cout << "x_2d: " << x_2d << " y_2d: " << y_2d << endl;
    double offset = sqrt(x_2d * x_2d + y_2d * y_2d);

//    cout << "offset / focal_length: " << offset / focal_length << endl;
//    cout << "atan(): " << atan(offset / focal_length) << endl;
    double theta = (atan(offset / focal_length) - camera_pitch) * 180 / PI;
    cout << "the angle between optical axis and projection ray: " << theta << endl;
    double depth = camera_height / sin(theta * PI / 180);

    point_with_depth.x = u;
    point_with_depth.y = v;
    point_with_depth.d = depth;
}

// get poses by scene_id corresponding to gps
vector<cv::Mat> getPosesBySceneId(string& scene_id, Utils::new3s_PointXYZ& original, cv::Mat mTcb) {
    Utils transform;
    Utils::new3s_PointXYZ enu_coord_1;
    Utils poseCompute;
    vector<cv::Mat> camera_poses;

    //根据scene_id显示此帧gps数据点
    Utils::new3s_PointXYZ  scene_point_start;
    GPSInfoEach gpsInfoEach;
    ReadHDMap::getGPSInfoBySceneId(scene_id, gpsInfoEach);
    vector<GPSPointEach> points = gpsInfoEach.gpsPoints;
    std::cout<<"GPS size: "<<points.size()<<std::endl;

    for (int k = 0; k < points.size(); ++k) {
        double header_angle = points[k].heading;

        // 这个是返回的NEU坐标系的点的差
        scene_point_start.set_x(points[k].points.x);
        scene_point_start.set_y(points[k].points.y);
        scene_point_start.set_z(points[k].points.z);
        transform.convertCJC02ToENU(scene_point_start, enu_coord_1, original);
        std::cout<<"enu_coord1: "<<enu_coord_1.get_x()<<" "<<enu_coord_1.get_y()<<" "<<enu_coord_1.get_z()<<std::endl;
        std::cout<<"delta_angle: "<<header_angle<<std::endl;
        // 这里对于header_angle的正负号可能还要确认一下
        cv::Mat tempRbw = poseCompute.convertAngleToR(-header_angle);
        cv::Mat tempTwb = cv::Mat::eye(4, 4, CV_64F);
        tempTwb.row(0).col(3) = enu_coord_1.get_x();
        tempTwb.row(1).col(3) = enu_coord_1.get_y();
        tempTwb.row(2).col(3) = 0;
        cv::Mat tempRwb = tempRbw.inv();
        tempRwb.copyTo(tempTwb.rowRange(0, 3).colRange(0, 3));
        pose = tempTwb.inv();
//       std::cout<<"tempTwb: "<<tempTwb<<std::endl;
//        std::cout<<"pose: "<<pose<<std::endl;
        cv::Mat camera_pose =   mTcb * pose  ; // Tcw
//        std::cout<<"mTcb1: "<<mTcb<<std::endl;
//        std::cout << "camera_pose: "<< camera_pose << std::endl;
        // Twc
        cv::Mat Twc = camera_pose.inv();
        std::cout << "Twc: "<< Twc << std::endl;
        camera_poses.push_back(camera_pose);
    }
    return camera_poses;
}


//convert RT of cv::Mat into R of Eigen::Matrix3d and t of Eigen::Vector3d
void cvMat2RT (cv::Mat& mat, Eigen::Matrix3d& R, Eigen::Vector3d& t) {
    R.row(0)(0) = mat.at<double>(0, 0);
    R.row(0)(1) = mat.at<double>(0, 1);
    R.row(0)(2) = mat.at<double>(0, 2);
    t.row(0)(0) = mat.at<double>(0, 3);

    R.row(1)(0) = mat.at<double>(1, 0);
    R.row(1)(1) = mat.at<double>(1, 1);
    R.row(1)(2) = mat.at<double>(1, 2);
    t.row(1)(0) = mat.at<double>(1, 3);

    R.row(2)(0) = mat.at<double>(2, 0);
    R.row(2)(1) = mat.at<double>(2, 1);
    R.row(2)(2) = mat.at<double>(2, 2);
    t.row(2)(0) = mat.at<double>(2, 3);
}


//// get image paths and corresponding poses (ref & currs) by gps_index and ref_img_index
//void getImagePathsAndPoses(int gps_index, int ref_img_index, vector<ImageBatch>& imageBatch_vec, Utils::new3s_PointXYZ& original,
//                           string& ref_img_path, vector<string>& curr_img_paths, cv::Mat& ref_pose, vector<cv::Mat>& curr_poses) {
//
//    // 当前gps对应的image paths信息
//    ImageBatch img_batch_curr = imageBatch_vec[gps_index];
//    vector<string> img_list_curr = img_batch_curr.images_vec;
//    int num_images_curr = img_list_curr.size();
//    string scene_id_curr = img_batch_curr.scene_id;
//    vector<cv::Mat> camera_poses_curr = getPosesBySceneId(scene_id_curr, original);
//
//    /**************get ref image path and corresponding pose************/
//    ref_img_path = img_list_curr[ref_img_index];
//    ref_pose = camera_poses_curr[ref_img_index];
//
//    /********************get neighbor image paths and corresponding poses************/
//    //如果邻近往后的frames数量比要求的neighbor frames小 (只考虑当前帧gps范围内的)
//    if (ref_img_index >= num_images_curr - neighbor_frames) {
//        for (int i = num_images_curr - neighbor_frames - 1; i < num_images_curr; i++) {
//            if (i == ref_img_index) continue;
//            curr_img_paths.push_back(img_list_curr[i]);
//            curr_poses.push_back(camera_poses_curr[i]);
//        }
//    }
//    else {
//        for (int i = ref_img_index + 1; i < ref_img_index + 1 + neighbor_frames; i++) {
//            curr_img_paths.push_back(img_list_curr[i]);
//            curr_poses.push_back(camera_poses_curr[i]);
//        }
//    }
//
////    //需要用到上一个scene
////    if (ref_img_index < neighbor_frames / 2) {
////        //当前gps是第一个
////        if (gps_index == 0) {
////            for (int i = 0; i < neighbor_frames + 1; i++){
////                if (i == ref_img_index) continue; //避开ref_img_path
////                curr_img_paths.push_back(imageBatch_vec[gps_index].images_vec[i]);
////                curr_poses.push_back(camera_poses_curr[i]);
////            }
////        }
////        //当前gps不是第一个
////        else {
////            ImageBatch img_batch_prev = imageBatch_vec[gps_index - 1]; //上一个scene的image paths
////            vector<string> img_list_prev = img_batch_prev.images_vec;
////            int num_images_prev = (int)img_list_prev.size();
////            string scene_id_prev = img_batch_prev.scene_id;
////            vector<cv::Mat> camera_poses_prev = getPosesBySceneId(scene_id_prev, original);
////
////            int end = ref_img_index + neighbor_frames / 2;            // end in curr_list
////            int start = num_images_prev - (neighbor_frames - end);    // start in before_list
////            for (int i = start; i < img_list_prev.size(); i++) {      //extract frames from before_list
////                curr_img_paths.push_back(img_list_prev[i]);
////                curr_poses.push_back(camera_poses_prev[i]);
////            }
////            for (int i = 0; i < end; i++) {                           //extract remained frames from curr_list
////                if (i == ref_img_index) continue;
////                curr_img_paths.push_back(img_list_curr[i]);
////                curr_poses.push_back(camera_poses_curr[i]);
////            }
////        }
////    }
////    //需要用到下一个scene
////    else if (ref_img_index > num_images_curr - neighbor_frames / 2) {
////        //当前gps是最后一个
////        if (gps_index == imageBatch_vec.size() - 1) {
////            for (int i = num_images_curr - neighbor_frames - 1; i < num_images_curr; i++) {
////                if (i == ref_img_index) continue;                           //避开ref_path
////                curr_img_paths.push_back(img_list_curr[i]);
////                curr_poses.push_back(camera_poses_curr[i]);
////            }
////        }
////        //当前gps不是最后一个
////        else {
////            ImageBatch img_batch_next = imageBatch_vec[gps_index + 1];
////            vector<string> img_list_next = img_batch_next.images_vec;
////            int num_images_next = (int)img_list_next.size();
////            string scene_id_next = img_batch_next.scene_id;
////            vector<cv::Mat> camera_poses_next = getPosesBySceneId(scene_id_next, original);
////
////            int start = ref_img_index - neighbor_frames / 2;
////            int end = neighbor_frames - (num_images_curr - start);
////            for (int i = start; i < num_images_curr; i++) {            // extract frames from curr
////                if (i == ref_img_index) continue;
////                curr_img_paths.push_back(img_list_curr[i]);
////                curr_poses.push_back(camera_poses_curr[i]);
////            }
////            for (int i = 0; i < end; i++) {                            // extract frames from next
////                curr_img_paths.push_back(img_list_next[i]);
////                curr_poses.push_back(camera_poses_next[i]);
////            }
////        }
////    }
////    //只在当前scene抽取neighbor frames
////    else {
////        for (int i = ref_img_index - neighbor_frames / 2; i <= ref_img_index + neighbor_frames / 2; i++) {
////            if (i == ref_img_index) continue;                              //避开ref_path
////            curr_img_paths.push_back(img_list_curr[i]);
////            curr_poses.push_back(camera_poses_curr[i]);
////        }
////    }
//}
//
//void getDepthMapOfSingleImage(int gps_index, int ref_img_index, vector<ImageBatch>& imageBatch_vec, Utils::new3s_PointXYZ& original,
//                              cv::Mat& depth, cv::Mat& depth_cov, Image3D& image3D, string& mode){
//
//    //inputs for building depth map
//    string ref_img_path;
//    cv::Mat ref_pose;
//    vector<string> curr_img_paths;
//    vector<cv::Mat> curr_poses;
//
//    getImagePathsAndPoses(gps_index, ref_img_index, imageBatch_vec, original, ref_img_path, curr_img_paths, ref_pose, curr_poses);
//
//    std::cout << "ref_img_path: " << ref_img_path << endl;
//    std::cout << "ref_pose: " << ref_pose << endl;
//    std::cout << "curr_img_paths size: " << curr_img_paths.size() << endl;
//    std::cout << "curr_poses size: " << curr_poses.size() << endl;
//
//    /*********** get "ref & currs" from "ref_img_path & curr_img_paths" ************/
//    string ref_img_name = "../data/images/" + ref_img_path + ".jpeg";
//    cv::Mat ref = imread(ref_img_name, 0);
//    vector<cv::Mat> currs;                                                              // 前后相邻frames
//    for (string curr_img_path : curr_img_paths) {
//        string curr_img_name = "../data/images/" + curr_img_path + ".jpeg";
//        cv::Mat curr = imread(curr_img_name, 0);
//        currs.push_back(curr);
//    }
//
//    /******* convert "ref_pose & curr_poses" into SE3 format: "ref_pose_SE3 & curr_poses_SE3" ********/
//    Eigen::Matrix3d R;
//    Eigen::Vector3d t;
//    cvMat2RT(ref_pose, R, t); //convert
//    Quaterniond q(R);
//    SE3 ref_pose_SE3(q, t);
//
//    vector<SE3> curr_poses_SE3;
//    for (cv::Mat curr_pose : curr_poses) {
//        Eigen::Matrix3d R;
//        Eigen::Vector3d t;
//        cvMat2RT(curr_pose, R, t);
//        Quaterniond q(R);
//        SE3 curr_pose_SE3(q, t);
//        curr_poses_SE3.push_back(curr_pose_SE3);
//    }
//
//    /********* actually build depth map from initialized depth map: Two modes ********/
//    if (mode == "region") {
//        vector<PointInt> points_target;
//        bool flag_detection = readDetectionFiles(ref_img_path, points_target);
//        if (flag_detection) {
//            cout << "Successfully detected traffic light or road lane!" << endl;
//        }
//
//        cout << "points_target size: " << points_target.size() << endl;
//        DepthMappingRegion DM(ref, ref_pose_SE3, currs, curr_poses_SE3, points_target);
//        bool ret = DM.build_depthMap(depth, depth_cov);
//        if (ret == true) {
//            cout << "successfully built region depth map!" << endl;
//        }
//
//        image3D.image_id = ref_img_path;
//        image3D.pose = ref_pose;
//        vector<Point3D> points_with_depth;
//        for (PointInt p : points_target) {
//            double curr_depth = depth.at<double> (image_height - p.y, p.x);
//            double curr_cov = depth_cov.at<double> (image_height - p.y, p.x);
//            if (curr_depth != init_depth && curr_cov < min_cov) {
//                // if (curr_depth > init_depth + 3 * curr_cov || curr_depth < init_depth - 3 * curr_cov) continue;
//                //包装信息
//                Point3D pwd;
//                pwd.x = p.x;
//                pwd.y = image_height - p.y; //和像素坐标系统一
//                pwd.label = p.label;
//                pwd.d = curr_depth;
//                points_with_depth.push_back(pwd);
//            }
//        }
//        image3D.points = points_with_depth;
//    }
//    if (mode == "full") {
//        DepthMappingFull DM(ref, ref_pose_SE3, currs, curr_poses_SE3);
//        bool ret = DM.build_depthMap(depth, depth_cov);
//        if (ret == true) {
//            cout << "successfully built full depth map!" << endl;
//        }
//
//        image3D.image_id = ref_img_path;
//        image3D.pose = ref_pose;
//        vector<Point3D> points_with_depth;
//        for (int x = border_x; x < image_width - border_x; x++) {
//            for (int y = border_y; y < image_height - border_y; y++) {
//                double curr_depth = depth.at<double> (image_height - y, x);
//                double curr_cov = depth_cov.at<double> (image_height - y, x);
//                if (curr_depth != init_depth && curr_cov < min_cov) {
//                    // if (curr_depth > init_depth + 3 * curr_cov || curr_depth < init_depth - 3 * curr_cov) continue;
//                    //包装信息
//                    Point3D point3D;
//                    point3D.x = x;
//                    point3D.y = image_height - y; //和像素坐标系统一
//                    //point3D.label = label;
//                    point3D.d = curr_depth;
//                    points_with_depth.push_back(point3D);
//                }
//            }
//        }
//        image3D.points = points_with_depth;
//    }
//
//    // save depth as image
//    Mat depth_out;
//    normalize(depth, depth_out, 0, 255, cv::NORM_MINMAX);
//    imwrite("/home/joey/work/zhangjing/deecamp/depth_results/depth_test.png", depth_out);
//    cout << "save /home/joey/work/zhangjing/deecamp/depth_results/depth_test.png" << endl;
//}
//
////get depth maps(all detected points with estimated depth and label + image_id & pose) of one scene
//void getDepthMapOfSingleScene(int gps_index, vector<ImageBatch>& imageBatch_vec, Utils::new3s_PointXYZ& original,
//                              vector<Image3D>& images_gps){
//
//    ImageBatch img_batch = imageBatch_vec[gps_index];
//    cout << "Number of image batch of current gps: " << img_batch.images_vec.size() << endl;
//    int counter = 0;
//    for (int i = 0; i < img_batch.images_vec.size() - 1; i++) {
//        counter++;
//        cv::Mat depth( image_height, image_width, CV_64F, init_depth );             // 初始化深度图
//        cv::Mat depth_cov( image_height, image_width, CV_64F, init_cov2 );          // 初始化深度方差图
//        Image3D image3D;
//        string mode = "region";
//        getDepthMapOfSingleImage(gps_index, i, imageBatch_vec, original, depth, depth_cov, image3D, mode);
//        images_gps.push_back(image3D);
//
////        // 将image3D中的所有点转换到enu坐标系下, 且写到txt。
////        string image_id = image3D.image_id;
////        string txt_name = "/home/joey/work/zhangjing/deecamp/depth_results/23/" + image_id + ".txt";
////        ofstream out(txt_name);
////        Mat pose = image3D.pose;
////        cv:: Mat R, t;
////        R = (cv::Mat_<double>(3, 3) << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2),
////                pose.at<double>(1, 0), pose.at<double>(1,1), pose.at<double>(1, 2),
////                pose.at<double>(2, 0), pose.at<double>(2,1), pose.at<double>(2, 2));
////        t = (cv::Mat_<double>(3, 1) << pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
////
////        cout << "R: " << R << "t: " << t << endl;
////
////        for (Point3D& point_with_depth : image3D.points) {
////            //pixel2cam
////            PointInt pixel_with_label;
////            pixel_with_label.x = point_with_depth.x;
////            pixel_with_label.y = point_with_depth.y;
////            pixel_with_label.label = point_with_depth.label;
////            PointDouble cam;
////            pixel2cam(pixel_with_label, cam, K);
////
////            // cam2enu
////            PointDepth coors_cam, coors_enu;
////            coors_cam.x = cam.x;
////            coors_cam.y = cam.y;
////            coors_cam.z = point_with_depth.d;
////            coors_cam.label = cam.label;
////            cam2enu(coors_cam, coors_enu, R, t);
////
////            // write into txt
////            string line_to_write = double2Str(coors_enu.x) + " " + double2Str(coors_enu.y) + " " + double2Str(coors_enu.z);
////            cout << line_to_write << endl;
////            out << line_to_write << endl;
////        }
////        cout << "finished writing into " << txt_name << endl;
//        cout << "-------------------------------------------" << endl;
//        if (counter == 4) break;
//    }
//}




#endif //DEECAMP_CONTAINER_H
