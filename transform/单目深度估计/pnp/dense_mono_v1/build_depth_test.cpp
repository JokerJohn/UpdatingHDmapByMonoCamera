#include <iostream>
#include <vector>
#include "build_depth.h"
#include "consts.h"

int main (int argc, char** argv) {
    if (argc != 4) {
        cout << "Usage: build_depth_test curr_index path_to_test_dataset" << endl;
        return -1;
    }

    const int curr_index = atoi(argv[1]);
    const int neighbor_frames = atoi(argv[2]);
    const string path(argv[3]);
    
    Mat depth( height, width, CV_64F, init_depth );             // 深度图
    Mat depth_cov( height, width, CV_64F, init_cov2 );

    DepthMapping DM(curr_index, neighbor_frames, path);
    bool ret = DM.build_depthMap(depth, depth_cov);
    if (ret == true) {
        cout << "successfully built depth map!" << endl;
    }

    // cout << "depth: " << depth << endl;
    Mat depth_out;
    normalize(depth, depth_out, 0, 255, cv::NORM_MINMAX);
    imwrite("depth.png", depth_out);
    cout<<"done."<< endl;

    return 0;
}