//
// Created by jianghc on 19-7-30.
//

#ifndef DEECAMP_UTILS_H
#define DEECAMP_UTILS_H
#include <math.h>
#include <iomanip>
#include <iostream>
#include <Eigen/Dense> //稠密矩阵代数运算,迹,特征值
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


//#include <opencv2/opencv.hpp>


class Utils {

public:
    Utils()
    {

    }

    virtual ~Utils()
    {

    }


    inline double deg2rad(double deg)
    {
        return (deg * M_PI) / 180;
    }

    inline double rad2deg(double rad)
    {
        return (rad * 180) / M_PI;
    }

    class new3s_PointXYZ
    {
    public:
        new3s_PointXYZ():x(0),y(0),z(0)
        {

        }
        new3s_PointXYZ(double x_ , double y_ , double z_):x(x_),y(y_),z(z_)
        {

        }

        new3s_PointXYZ(const new3s_PointXYZ &other)
        {
            *this = other ;
        }

        virtual ~new3s_PointXYZ()
        {

        }

        new3s_PointXYZ &operator=(const new3s_PointXYZ &other)
        {
            x = other.x ;
            y = other.y ;
            z = other.z ;
            return *this ;
        }

        new3s_PointXYZ operator-(const new3s_PointXYZ &other)
        {
            x -= other.x ;
            y -= other.y ;
            z -= other.z ;
            return *this ;
        }

        double get_x()const{return x;}
        double get_y()const{return y;}
        double get_z()const{return z;}
        void set_x(double x_)
        {
            x = x_ ;
        }
        void set_y(double y_)
        {
            y = y_ ;
        }
        void set_z(double z_)
        {
            z = z_ ;
        }

    private:
        double x , y , z ;
    };

protected:


public:

    double transformLat(double x, double y);
    double transformLon(double x, double y);
    void convertGCJO2ToLonlat(const new3s_PointXYZ GCJ02_coord, new3s_PointXYZ& lat_lon_coord);
    void convertLLHToXYZ(const new3s_PointXYZ lat_lon_coord, new3s_PointXYZ& xyz_coord);
    void convertXYZToLLH(const Utils::new3s_PointXYZ xyz_coord, Utils::new3s_PointXYZ& lat_lon_coord);
    void convertXYZToENU(Utils::new3s_PointXYZ llh_coord, const Utils::new3s_PointXYZ xyz_coord, Utils::new3s_PointXYZ& enu_coord);
    void convertCJC02ToENU(const Utils::new3s_PointXYZ CJC02_coord, Utils::new3s_PointXYZ &ENU_coord, Utils::new3s_PointXYZ original_CJC02);
    void convertENUToXYZ(Utils::new3s_PointXYZ llh_coord, Utils::new3s_PointXYZ enu_coord ,Utils::new3s_PointXYZ& xyz_coord);
    void convertLonlatToGCJ02(const Utils::new3s_PointXYZ lat_lon_coord, Utils::new3s_PointXYZ& GCJ02_coord);

    void readImgAndDoRectify(const cv::Mat Original, cv::Mat& TargetImg, cv::Mat K, cv::Mat distCoeffs, cv::Mat new_matrix);
    cv::Mat convertAngleToR(double delta);
    cv::Mat updatePose(cv::Mat pose, double header_former, double header_now, double t[3]);


private:
    const double PI = 3.14159265358979324;//圆周率
    const double a = 6378245.0;//克拉索夫斯基椭球参数长半轴a
    const double ee = 0.00669342162296594323;//克拉索夫斯基椭球参数第一偏心率平方
    const double FE_WGS84 = (1.0/298.257223563);
    const double RE_WGS84 = 6378137.0;

    double dot(const double *a, const double *b, int n);
    void xyz2enu(const double *pos, double *E);
    void matmul(const char *tr, int n, int k, int m, double alpha,
                const double *A, const double *B, double beta, double *C);


};


#endif //DEECAMP_UTILS_H
