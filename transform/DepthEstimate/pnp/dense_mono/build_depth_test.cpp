#include <iostream>
#include <vector>
#include "build_depth.h"
#include "consts.h"
#include "ReadHDMap.h"

int main (int argc, char** argv) {
    if (argc != 8) {
        cout << "Usage: build_depth_test curr_index path_to_test_dataset" << endl;
        return -1;
    }

    // inputs
    const int ref_gps_index = atoi(argv[1]);                    // 当前gps的index     
    const int ref_img_index = atoi(argv[2]);                    // 在当前gps里的image index
    const string gps_path(argv[3]);                             // 当前gps的file path
    
    // get ref and currs
    string ref_path;                                                                // 当前要进行深度估计的image path
    vector<string> currs_path;                                                      // 前后相邻frames的image paths
    getImgPaths(gps_path, ref_gps_index, ref_img_index, ref_path, currs_path);
    Mat ref = imread(ref_path, 0);                                                  // 当前要进行深度估计的image
    vector<Mat> currs;                                                              // 前后相邻frames
    for (string curr_path : currs_path) {
        Mat curr = imread(curr_path);
        currs.push_back(curr);
    }

    // depth initialization
    Mat depth( height, width, CV_64F, init_depth );             // 深度图
    Mat depth_cov( height, width, CV_64F, init_cov2 );          // 深度方差图
   
    SE3 pose_ref;                                                                     //　当前要进行深度估计的image的pose
    Vector<Pose> poses_curr;                                                        //  前后相邻frames对应的pose    
    
    vector<Point> points;                                       //  目标区域的点集（带label）
    
    DepthMapping DM(ref, pose_ref, currs, poses_curr, points);
    bool ret = DM.build_depthMap(depth, depth_cov);
    if (ret == true) {
        cout << "successfully built depth map!" << endl;
    }

    // cout << "depth: " << depth << endl;
    Mat depth_out;
    normalize(depth, depth_out, 0, 255, cv::NORM_MINMAX); //normalize into 0 - 255
    imwrite("depth.png", depth_out);
    cout<<"done."<< endl;

    return 0;
}

bool getFilesFromDir(const string& gps_path, vector<string>& filenames);

// inputs:  gps_path, ref_gps_index, ref_img_index
// outputs: ref_path and currs_path
bool getImgPaths(const string& gps_path, const int& ref_gps_index, const int& ref_img_index, 
                 const string& ref_path, const vector<string>& currs_path) {                 
    // get ref_path
    vector<string> gps_files;                                               // gps_path内部的所有gps文件名称
    getFilesFromDir(gps_path, gps_files);
    int total_gps = gps_files.size();
    string scene_id = gps_files[ref_gps_index];
    GPSInfoEach gpsInfoEach = getGPSInfoEachBySceneId(sence_id);            // API
    vector<string> curr_img_list = gpsInfoEach.img_list;                    // 得到ref_gps_index所指引的gps对应的image文件名称
    ref_path = curr_img_list[ref_img_index];                             

    // get currs_path
    // 需要用到上一段gps的frames
    if (ref_img_index < (neighbor_frames / 2)){
        // 当前gps是第一个
        if (ref_gps_index == 0){
            for (int i = 0; i <= neighbor_frames; i++){
                if (i == ref_img_index) continue;                           //避开ref_path
                currs_path.push_back(curr_img_list[i]);
            }
        }
        // 当前gps不是第一个
        else{
            // API
            GPSInfoEach before_gpsInfoEach = getGPSInfoEachBySceneId(gps_files[ref_gps_index - 1]);
            vector<string> before_img_list = before_gpsInfoEach.img_list;   // ref_gps_index的before所指引的gps对应的image文件名称
            int end = ref_img_index + neighbor_frames / 2;                  // end in curr_list
            int start = before_img_list.size() - (neighbor_frames - end);   // start in before_list

            for (int i = start; i < before_img_list.size(); i++) {          //extract frames from before_list
                currs_path.push_back(before_img_list[i]);
            }
            for (int i = 0; i < end; i++) {                                 //extract remained frames from curr_list
                if (i == ref_img_index) continue;
                currs_path.push_back(curr_img_list[i]);
            }
        }
    }
    // 需要用到下一段gps的frames
    else if (ref_img_index > curr_img_list.size() - neighbor_frames / 2) {
        // 当前gps是最后一个
        if (ref_gps_index == total_gps - 1) {
            for (int i = curr_img_list.size() - neighbor_frames - 1; i < curr_img_list; i++) {
                if (i == ref_img_index) continue;                           //避开ref_path
                currs_path.push_back(curr_img_list[i]);   
            }
        }
        // 当前gps不是最后一个
        else{
            // API
            GPSInfoEach next_gpsInfoEach = getGPSInfoEachBySceneId(gps_files[ref_gps_index + 1]);
            vector<string> next_img_list = next_gpsInfoEach.img_list;       // ref_gps_index的next所指引的gps对应的image文件名称 
            int start = ref_img_index - neighbor_frames / 2;
            int end = neighbor_frames - (curr_img_list.size() - start);

            for (int i = start; i < curr_img_list.size(); i++) {            // extract frames from curr
                if (i == ref_img_index) continue;
                currs_path.push_back(curr_img_list[i]);
            }
            for (int i = 0; i < end; i++) {                                 // extract frames from next
                currs_path.push_back(next_img_list[i]);
            }
        }
    }
    else{
        for (int i = ref_img_index - neighbor_frames / 2; i <= ref_img_index + neighbor_frames / 2; i++){
            if (i == ref_img_index)　continue;                              //避开ref_path
            currs_path.push_back(curr_img_list[i]);
        }
    }

    return true;
}

