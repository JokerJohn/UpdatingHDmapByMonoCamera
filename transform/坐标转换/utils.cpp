//
//    create by jianghc 2019.7.28
//
#include <stdio.h>
#include "utils.h"
#include <math.h>
#include <opencv2/opencv.hpp>

/*
// UTM 转 WGS84  这里的分带可以取51或50
void convertUtmToLonlat(const new3s_PointXYZ utm_coord, const int& utmzone, new3s_PointXYZ& lon_lat_coord)
{
    //建立投影坐标系到经纬度坐标系的转换
    OGRSpatialReference *RefSource = new OGRSpatialReference;
    RefSource->SetWellKnownGeogCS("WGS84");
    RefSource->SetProjCS("UTM(WGS84) in northern hemisphere.");
    RefSource->SetUTM(utmzone, TRUE);
    OGRSpatialReference *RefTarget = new OGRSpatialReference;
    RefTarget = RefSource->CloneGeogCS();
    OGRCoordinateTransformation *poTranform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

    OGRPoint *poPoint = new OGRPoint();
    double tempx = utm_coord.get_x();
    double tempy = utm_coord.get_y();
    double tempz = utm_coord.get_z();

    poTranform->Transform(1, &tempx, &tempy, NULL);
    lon_lat_coord = new3s_PointXYZ(tempx, tempy, tempz);
}*/

double transformLat(double x, double y) {
    double PI = 3.14159265358979324;//圆周率
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

double transformLon(double x, double y) {
    double PI = 3.14159265358979324;//圆周率
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
    return ret;
}

// 火星坐标系转WGS84
void convertGCJO2ToLonlat(const new3s_PointXYZ GCJ02_coord, new3s_PointXYZ& lon_lat_coord)
{
    // 输入x是经度，有对应的是纬度     x：lon    y：lat
    double lon = GCJ02_coord.get_x();
    double lat = GCJ02_coord.get_y();
    double PI = 3.14159265358979324;//圆周率
    double a = 6378245.0;//克拉索夫斯基椭球参数长半轴a
    double ee = 0.00669342162296594323;//克拉索夫斯基椭球参数第一偏心率平方
    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * PI;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * PI);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * PI);
    lon_lat_coord.set_x(lon - dLon);
    lon_lat_coord.set_y(lat - dLat);
}

// WGS84 转 东北天  BLHtoXYZ
void convertLonlatToENU(const new3s_PointXYZ lon_lat_coord, new3s_PointXYZ& enu_coord)
{
    //name note:  B:lon   L:lat  H:height
    //  这里的顺序要注意一下需要在查询手册
    double lon = lon_lat_coord.get_x();
    double lat = lon_lat_coord.get_y();
    double L0 = (int((lon - 1.5)/3.0) + 1) * 3.0;
    double ee = (a * a - b * b) / (a * a);
    double e2 = (a * a - b * b) / (b * b);
    double n = (a - b) / (a + b);
    double n2 = (n * n);
    double n3 = (n2 * n);
    double n4 = (n2 * n2);
    double n5 = (n4 * n);
    double al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2;
    double bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32;
    double gm = 15 * n2 / 16 - 15 * n4 / 32;
    double dt = -35 * n3 / 48 + 105 * n5 / 256;
    double ep = 315 * n4 / 512;

    lat = lat * iPI;
    lon = lon * iPI;
    L0 = L0 * iPI;
    double l = lon - L0;
    double cl = cos(lat) * l;
    double cl2 = (cl * cl);
    double cl3 = (cl2 * cl);
    double cl4 = (cl2 * cl2);
    double cl5 = (cl4 * cl);
    double cl6 = (cl5 * cl);
    double cl7 = (cl6 * cl);
    double cl8 = (cl4 * cl4);

    double lB = al * (lat + bt * sin(2 * lat) + gm * sin(4 * lat) + dt * sin(6 * lat) + ep * sin(8 * lat));
    double t = tan(lat);
    double t2 = (t * t);
    double t4 = (t2 * t2);
    double t6 = (t4 * t2);

    double Nn = a / sqrt(1 - ee * sin(lat) * sin(lat));
    double yt = e2 * cos(lat) * cos(lat);
    double N = lB;
    N += t * Nn * cl2 / 2;
    N += t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
    N += t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
    N += t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;

    double E = Nn * cl;
    E += Nn * cl3 * (1 - t2 + yt) / 6;
    E += Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
    E += Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;

    E += 500000;
    N = 0.9999 * N;
    E = 0.9999 * (E - 500000.0) + 250000.0;

    enu_coord.set_x(E);
    enu_coord.set_y(N);

}

void readUtmData()
{

}

void readImgAndDoRectify(const cv::Mat Original, cv::Mat& TargetImg)
{

    // 这步看需不要加速计算转变为 initUndistortRectifyMap + remap
    cv::undistort(Original, TargetImg, K, distCoeffs, new_matrix);
}


cv::Mat updatePose(cv::Mat pose, double header_former, double header_now， double t[3])
{
    // 是不是初始化，如果是初始化的时候返回一个位姿，如果非初始化返回一个位姿
    // 这里假定是左乘更新，以东北天作为静止
    double delta_angle = header_now - header_former;
    cv::Mat Rb1b2 = convertAngleToR(delta_angle);
    cv::Mat Tb1b2 = cv::Mat::eye(4, 4, CV_64F);
    Rb1b2.copyTo(Tb1b2.rowRange(0, 3).colRange(0, 3));
    Tb1b2.row(0).col(3) = t[0];
    Tb1b2.row(1).col(3) = t[1];
    Tb1b2.row(2).col(3) = t[2];
    return Tb1b2*pose;
}

cv::Mat convertAngleToR(double delta)
{
    cv::Mat R;
    R = (cv::Mat_<float>(3, 3) <<
            cos(delta), sin(delta), 0.0,
            -sin(delta), cos(delta), 0.0,
            0.0, 0.0, 1.0);
    return R;
}

int main(int argc, char const *argv[]) {
    /* code */
    // 加载相关参数文件
    const std::string strCameraPath = "../Config/param.yml";
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
    cv::Mat new_matrix;
    K = (cv::Mat_<float>(3, 3) <<
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);
    distCoeffs = (cv::Mat_<float>(4, 1) << k1, k2, p1, p2);

    // 正常Tcb的值的初始化和计算只需进行一次，因此这里代码应该放在主函数里
    mTbw = cv::Mat::eye(4, 4, CV_64F);
    mTcb = cv::Mat::eye(3, 3, CV_64F);
    // 这里读取到第一个帧的车的偏角
    double header_former;
    mRbw = convertAngleToR(header_former);
    mRbw.copyTo(mTbw.rowRange(0, 3).colRange(0, 3));
    // 这里假定我得到了第二个帧的车的偏角
    double header_now;
    // 这个是返回的NEU坐标系的点的差
    double add_t[3];
    // 对于初始的pose
    cv::Mat pose = mTbw;
    cv::Mat now_pose = updatePose(pose, header_former, header_now, add_t);
    cv::Mat camera_pose = mTcb * now_pose;

    return 0;
}
