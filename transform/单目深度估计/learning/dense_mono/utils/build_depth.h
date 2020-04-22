#ifndef __DepthMapping_H__
#define __DepthMapping_H__

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
using namespace std; 
#include <boost/timer.hpp>

// for sophus 
#include <sophus/se3.h>
using Sophus::SE3;

// for eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
#include "utils/consts.h"
#include "utils/read_hdmap.h"

class DepthMapping {
private:
    vector<Point_label> points_target;               //target region points
    Mat ref;                                         //ref image
    SE3 pose_ref_TWC;                                //ref_pose
    vector<Mat> currs;                               //curr images
    vector<Pose_id> poses_curr_TWC;                  //curr poses

public:
    // CONSTRUCTOR
    DepthMapping(Mat& ref, SE3& pose_ref, vector<Mat>& currs, vector<Pose_id>& poses_curr, vector<Point_label> points) {
        this->ref = ref;
        this->pose_ref_TWC = pose_ref;

        this->currs = currs;
        this->poses_curr_TWC = poses_curr;

        this->points_target = points;
        cout << "neighbors size: " << currs.size() << endl;
    }

    // main method
    bool build_depthMap(Mat& depth, Mat& depth_cov) {
        for (int ind = 0; ind < currs.size(); ind++) {
            cout << " *** loop " << ind << " *** " << endl;

            Mat& curr = currs[ind];
            if (curr.data == nullptr) continue;
            SE3 pose_curr_TWC = poses_curr_TWC[ind].tf;
            SE3 pose_T_C_R = pose_curr_TWC.inverse() * pose_ref_TWC;
            
            update( ref, curr, pose_T_C_R, depth, depth_cov );
            
            plotDepth( depth ); //updata depth picture
            imshow("image", curr);
            waitKey(1);
        }
        return true;
    }

    // 对整个深度图进行更新
    bool update(const Mat& ref, const Mat& curr, const SE3& T_C_R, Mat& depth, Mat& depth_cov ){
        int match_count = 0, converge_count = 0, diverge_count = 0;
        int total = (width - boarder) * (height - boarder);

        for (Point_label p : points_target) {
            int x = p.x;
            int y = p.y;
            // 遍历每个像素
            if ( depth_cov.ptr<double>(y)[x] < min_cov) { // 深度已收敛
                converge_count++;
                continue;
            }
            if ( depth_cov.ptr<double>(y)[x] > max_cov) { // 深度已发散
                diverge_count++;
                continue;
            }

            // 在极线上搜索 (x,y) 的匹配 
            Vector2d pt_curr; 
            bool ret = epipolarSearch ( ref, curr, T_C_R, Vector2d(x,y), depth.ptr<double>(y)[x], sqrt(depth_cov.ptr<double>(y)[x]),pt_curr);
            if ( ret == false ) { // 匹配失败
                continue;
            }
            
            // 取消该注释以显示匹配
            match_count++;
            // showEpipolarMatch( ref, curr, Vector2d(x,y), pt_curr );
            
            // 匹配成功，更新深度图 
            updateDepthFilter( Vector2d(x,y), pt_curr, T_C_R, depth, depth_cov );
        }
        cout << "match_count: " << match_count << endl;
        printf("found %.2f%% matches! ", (float)match_count / total * 100);
        printf("%.2f%% already converged! ", (float)converge_count / total * 100);
        printf("%.2f%% already diverged! \n", (float)diverge_count / total * 100);
    }

    // 极线搜索
    bool epipolarSearch(const Mat& ref, const Mat& curr, const SE3& T_C_R, const Vector2d& pt_ref, 
        const double& depth_mu, const double& depth_cov, Vector2d& pt_curr )
    {
        Vector3d f_ref = px2cam( pt_ref );
        f_ref.normalize(); //归一化到深度为1的平面
        Vector3d P_ref = f_ref*depth_mu;	// 参考帧的 P 向量
        
        Vector2d px_mean_curr = cam2px( T_C_R*P_ref ); // 按深度均值投影的像素
        double d_min = depth_mu-3*depth_cov, d_max = depth_mu+3*depth_cov;
        if ( d_min<0.1 ) d_min = 0.1;
        Vector2d px_min_curr = cam2px( T_C_R*(f_ref*d_min) );	// 按最小深度投影的像素
        Vector2d px_max_curr = cam2px( T_C_R*(f_ref*d_max) );	// 按最大深度投影的像素
        
        Vector2d epipolar_line = px_max_curr - px_min_curr;	// 极线（线段形式）
        Vector2d epipolar_direction = epipolar_line;		// 极线方向 
        epipolar_direction.normalize();
        double half_length = 0.5*epipolar_line.norm();	// 极线线段的半长度
        if ( half_length>100 ) half_length = 100;   // 我们不希望搜索太多东西 
        
        // 取消此句注释以显示极线（线段）
        // showEpipolarLine( ref, curr, pt_ref, px_min_curr, px_max_curr );
        
        // 在极线上搜索，以深度均值点为中心，左右各取半长度
        double best_ncc = -1.0;
        Vector2d best_px_curr; 
        for ( double l=-half_length; l<=half_length; l+=0.7 )  // l+=sqrt(2) 
        {
            Vector2d px_curr = px_mean_curr + l*epipolar_direction;  // 待匹配点
            if ( !inside(px_curr) )
                continue; 
            // 计算待匹配点与参考帧的 NCC
            double ncc = NCC( ref, curr, pt_ref, px_curr );
            if ( ncc>best_ncc )
            {
                best_ncc = ncc; 
                best_px_curr = px_curr;
            }
        }
        if ( best_ncc < 0.85f )      // 只相信 NCC 很高的匹配
            return false; 
        pt_curr = best_px_curr; //got the matched px_curr
        return true;
    }

    //helper function for epipolar search: computing ncc
    double NCC (const Mat& ref, const Mat& curr, const Vector2d& pt_ref, const Vector2d& pt_curr)
    {
        // 零均值-归一化互相关
        // 先算均值
        double mean_ref = 0, mean_curr = 0;
        vector<double> values_ref, values_curr; // 参考帧和当前帧的均值
        for ( int x=-ncc_window_size; x<=ncc_window_size; x++ )
            for ( int y=-ncc_window_size; y<=ncc_window_size; y++ )
            {
                double value_ref = double(ref.ptr<uchar>( int(y+pt_ref(1,0)) )[ int(x+pt_ref(0,0)) ])/255.0;
                mean_ref += value_ref;
                
                double value_curr = getBilinearInterpolatedValue( curr, pt_curr+Vector2d(x,y) ); //macth到的curr上的点不一定是integer
                mean_curr += value_curr;
                
                values_ref.push_back(value_ref);
                values_curr.push_back(value_curr);
            }
            
        mean_ref /= ncc_area;
        mean_curr /= ncc_area;
        
        // 计算 Zero mean NCC （去均值化的）
        double numerator = 0, demoniator1 = 0, demoniator2 = 0;
        for ( int i=0; i<values_ref.size(); i++ )
        {
            double n = (values_ref[i]-mean_ref) * (values_curr[i]-mean_curr);
            numerator += n;
            demoniator1 += (values_ref[i]-mean_ref)*(values_ref[i]-mean_ref);
            demoniator2 += (values_curr[i]-mean_curr)*(values_curr[i]-mean_curr);
        }
        return numerator / sqrt( demoniator1*demoniator2+1e-10 );   // 防止分母出现零
    }

    // helper function for NCC: 双线性灰度插值
    inline double getBilinearInterpolatedValue( const Mat& img, const Vector2d& pt ) {
        uchar* d = & img.data[ int(pt(1,0))*img.step+int(pt(0,0)) ];
        double xx = pt(0,0) - floor(pt(0,0)); 
        double yy = pt(1,0) - floor(pt(1,0));
        return  (( 1-xx ) * ( 1-yy ) * double(d[0]) +
                xx* ( 1-yy ) * double(d[1]) +
                ( 1-xx ) *yy* double(d[img.step]) +
                xx*yy*double(d[img.step+1]))/255.0;
    }

    //helper function for update： 三角化测量 + 高斯融合
    bool updateDepthFilter(const Vector2d& pt_ref, const Vector2d& pt_curr, const SE3& T_C_R, 
                            Mat& depth, Mat& depth_cov)
    {
        // 用三角化计算深度
        SE3 T_R_C = T_C_R.inverse();
        Vector3d f_ref = px2cam( pt_ref );
        f_ref.normalize();
        Vector3d f_curr = px2cam( pt_curr );
        f_curr.normalize();
        
        // 方程
        // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
        // => [ f_ref^T f_ref, -f_ref^T f_cur ] [d_ref] = [f_ref^T t]
        //    [ f_cur^T f_ref, -f_cur^T f_cur ] [d_cur] = [f_cur^T t]
        // 二阶方程用克莱默法则求解并解之
        Vector3d t = T_R_C.translation();
        Vector3d f2 = T_R_C.rotation_matrix() * f_curr; 
        Vector2d b = Vector2d ( t.dot ( f_ref ), t.dot ( f2 ) );
        double A[4];
        A[0] = f_ref.dot ( f_ref );
        A[2] = f_ref.dot ( f2 );
        A[1] = -A[2];
        A[3] = - f2.dot ( f2 );
        double d = A[0]*A[3]-A[1]*A[2];
        Vector2d lambdavec = 
            Vector2d (  A[3] * b ( 0,0 ) - A[1] * b ( 1,0 ),
                        -A[2] * b ( 0,0 ) + A[0] * b ( 1,0 )) /d;
        Vector3d xm = lambdavec ( 0,0 ) * f_ref;
        Vector3d xn = t + lambdavec ( 1,0 ) * f2;
        Vector3d d_esti = ( xm+xn ) / 2.0;  // 三角化算得的深度向量
        double depth_estimation = d_esti.norm();   // 深度值
        
        // 计算不确定性（以一个像素为误差）
        Vector3d p = f_ref*depth_estimation;
        Vector3d a = p - t; 
        double t_norm = t.norm();
        double a_norm = a.norm();
        double alpha = acos( f_ref.dot(t)/t_norm );
        double beta = acos( -a.dot(t)/(a_norm*t_norm));
        double beta_prime = beta + atan(1/fx);
        double gamma = M_PI - alpha - beta_prime;
        double p_prime = t_norm * sin(beta_prime) / sin(gamma);
        double d_cov = p_prime - depth_estimation; 
        double d_cov2 = d_cov*d_cov;
        
        // 高斯融合
        double mu = depth.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ];
        double sigma2 = depth_cov.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ];
        
        double mu_fuse = (d_cov2*mu+sigma2*depth_estimation) / ( sigma2+d_cov2);
        double sigma_fuse2 = ( sigma2 * d_cov2 ) / ( sigma2 + d_cov2 );
        
        depth.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ] = mu_fuse; 
        depth_cov.ptr<double>( int(pt_ref(1,0)) )[ int(pt_ref(0,0)) ] = sigma_fuse2;
        
        return true;
    }

    // 作图工具
    void plotDepth(const Mat& depth)
    {
        imshow( "depth", depth*0.4 );
        waitKey(1);
    }

    // 像素到相机坐标系 (pixels to m)
    inline Vector3d px2cam ( const Vector2d px ) {
        return Vector3d ( 
            (px(0,0) - cx)/fx,
            (px(1,0) - cy)/fy, 
            1
        );
    }

    // 相机坐标系到像素 
    inline Vector2d cam2px ( const Vector3d p_cam ) {
        return Vector2d (
            p_cam(0,0)*fx/p_cam(2,0) + cx, 
            p_cam(1,0)*fy/p_cam(2,0) + cy 
        );
    }

    // 检测一个点是否在图像边框内
    inline bool inside( const Vector2d& pt ) {
        return pt(0,0) >= boarder && pt(1,0)>=boarder 
            && pt(0,0)+boarder<width && pt(1,0)+boarder<=height;
    }
};

#endif
