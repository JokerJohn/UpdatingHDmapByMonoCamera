//
//  consts.h
//  Monocular_dense
//
//  Created by doctor_lin on 7/30/19.
//  Copyright © 2019 doctor_lin. All rights reserved.
//

#ifndef consts_h
#define consts_h

// ------------------------------------------------------------------
// parameters
const int boarder = 20;     // 边缘宽度
const int width = 640;      // 宽度
const int height = 480;      // 高度
const double fx = 481.2f;    // 相机内参
const double fy = -480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
const int ncc_window_size = 2;    // NCC 取的窗口半宽度
const int ncc_area = (2*ncc_window_size+1)*(2*ncc_window_size+1); // NCC窗口面积
const double min_cov = 0.1;    // 收敛判定：最小方差
const double max_cov = 10;    // 发散判定：最大方差

const int max_frames = 200;
const int neighbor_frames = 20;

const double init_depth   = 3.0;    // 深度初始值
const double init_cov2    = 3.0;    // 方差初始值

//region point with label
typedef struct Point_tag {
    int x;
    int y;
    string label;
} Point_label;

//camera pose with image id
typedef struct Pose_tag {
    SE3 tf;
    string image_id;
} Pose_id;

#endif /* consts_h */
