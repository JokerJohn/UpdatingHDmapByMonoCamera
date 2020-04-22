//
// Created by jianghc on 19-7-30.
//

#include "Utils.h"

double Utils::transformLat(double x, double y) {
    double PI = 3.14159265358979324;//圆周率
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

double Utils::transformLon(double x, double y) {
    double PI = 3.14159265358979324;//圆周率
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
    return ret;
}

double Utils::dot(const double *a, const double *b, int n)
{
    double c=0.0;

    while (--n>=0) c+=a[n]*b[n];
    return c;
}

void Utils::xyz2enu(const double *pos, double *E)
{
    double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);

    E[0]=-sinl;      E[3]=cosl;       E[6]=0.0;
    E[1]=-sinp*cosl; E[4]=-sinp*sinl; E[7]=cosp;
    E[2]=cosp*cosl;  E[5]=cosp*sinl;  E[8]=sinp;
}

void Utils::matmul(const char *tr, int n, int k, int m, double alpha,
            const double *A, const double *B, double beta, double *C)
{
    double d;
    int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

    for (i=0;i<n;i++) for (j=0;j<k;j++) {
            d=0.0;
            switch (f) {
                case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
                case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
                case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
                case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
            }
            if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
        }
}


// 火星坐标系转WGS84 这里统一都是先纬度后经度
void Utils::convertGCJO2ToLonlat(const Utils::new3s_PointXYZ GCJ02_coord, Utils::new3s_PointXYZ& lat_lon_coord)
{
    // 输入x是经度，有对应的是纬度     x：lat    y：lon
    double lat = GCJ02_coord.get_x();
    double lon = GCJ02_coord.get_y();
    double height = GCJ02_coord.get_z();
    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * PI;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * PI);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * PI);
    lat_lon_coord.set_x(lat - dLat);
    lat_lon_coord.set_y(lon - dLon);
    lat_lon_coord.set_z(height);
}

void Utils::convertLonlatToGCJ02(const Utils::new3s_PointXYZ lat_lon_coord, Utils::new3s_PointXYZ& GCJ02_coord)
{
    double lat = lat_lon_coord.get_x();
    double lon = lat_lon_coord.get_y();
    double height = lat_lon_coord.get_z();
    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * PI;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * PI);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * PI);
    GCJ02_coord.set_x(lat + dLat);
    GCJ02_coord.set_y(lon + dLon);
    GCJ02_coord.set_z(height);
}

// WGS84 转 ECEF
void Utils::convertLLHToXYZ(const Utils::new3s_PointXYZ lat_lon_coord, Utils::new3s_PointXYZ& xyz_coord)
{
    //name note:  B:lat   L:lon  H:height lat经度 lon纬度
    //  这里的顺序要注意一下需要在查询手册
    double lat = lat_lon_coord.get_x();
    double lon = lat_lon_coord.get_y();
    double height = lat_lon_coord.get_z();
    lon = deg2rad(lon);
    lat = deg2rad(lat);
    double sinp = sin(lat),cosp = cos(lat),sinl = sin(lon),cosl = cos(lon);
    double  e2 = FE_WGS84 *(2.0- FE_WGS84),v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    double x =(v + height)* cosp * cosl ;
    double y =(v + height)* cosp * sinl ;
    double z =(v *(1.0 - e2)+ height)* sinp;

    xyz_coord.set_x(x);
    xyz_coord.set_y(y);
    xyz_coord.set_z(z);

}


void Utils::convertXYZToENU(Utils::new3s_PointXYZ llh_coord, const Utils::new3s_PointXYZ xyz_coord, Utils::new3s_PointXYZ& enu_coord)
{
    double E[9];

    double pos[3];
    double r[3];
    double e[3];
    pos[0] = deg2rad(llh_coord.get_x());
    pos[1] = deg2rad(llh_coord.get_y());
    pos[2] = 0;
    r[0] = xyz_coord.get_x();
    r[1] = xyz_coord.get_y();
    r[2] = xyz_coord.get_z();
    xyz2enu(pos,E);
    matmul("NN",3,1,3,1.0,E,r,0.0,e);

    enu_coord.set_x(e[0]);
    enu_coord.set_y(e[1]);
    enu_coord.set_z(e[2]);

}

void Utils::convertENUToXYZ(Utils::new3s_PointXYZ llh_coord, Utils::new3s_PointXYZ enu_coord ,Utils::new3s_PointXYZ& xyz_coord)
{
    double E[9];
    double pos[3];
    double r[3];
    double e[3];
    pos[0] = deg2rad(llh_coord.get_x());
    pos[1] = deg2rad(llh_coord.get_y());
    pos[2] = 0;

    e[0] = enu_coord.get_x();
    e[1] = enu_coord.get_y();
    e[2] = enu_coord.get_z();
    xyz2enu(pos,E);
    matmul("TN",3,1,3,1.0,E,e,0.0,r);
    xyz_coord.set_x(r[0]);
    xyz_coord.set_y(r[1]);
    xyz_coord.set_z(r[2]);
}

void Utils::convertXYZToLLH(const Utils::new3s_PointXYZ xyz_coord, Utils::new3s_PointXYZ& lat_lon_coord)
{

    double r[3];
    r[0] = xyz_coord.get_x();
    r[1] = xyz_coord.get_y();
    r[2] = xyz_coord.get_z();
    double e2=FE_WGS84*(2.0-FE_WGS84),r2 = dot(r,r,2),z,zk,v=RE_WGS84,sinp;

    for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) {
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
        z=r[2]+v*e2*sinp;
    }
    double pos[3];
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?M_PI/2.0:-M_PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
    lat_lon_coord.set_x(rad2deg(pos[0]));
    lat_lon_coord.set_y(rad2deg(pos[1]));
    lat_lon_coord.set_z(pos[2]);
}

void Utils::convertCJC02ToENU(const Utils::new3s_PointXYZ CJC02_coord, Utils::new3s_PointXYZ &ENU_coord, Utils::new3s_PointXYZ original_CJC02)
{
    Utils::new3s_PointXYZ WGS84_coord;
    Utils::new3s_PointXYZ XYZ_coord;
    Utils::new3s_PointXYZ Original_WGS;
    Utils::new3s_PointXYZ Original_XYZ;
    Utils::new3s_PointXYZ Origianl_ENU;
    Utils transform;
    transform.convertGCJO2ToLonlat(CJC02_coord, WGS84_coord);
    transform.convertGCJO2ToLonlat(original_CJC02, Original_WGS);
    transform.convertLLHToXYZ(WGS84_coord, XYZ_coord);
    transform.convertLLHToXYZ(Original_WGS, Original_XYZ);
    transform.convertXYZToENU(Original_WGS, Original_XYZ, Origianl_ENU);
    transform.convertXYZToENU(Original_WGS, XYZ_coord, ENU_coord);
    ENU_coord = ENU_coord - Origianl_ENU;
    ENU_coord.set_z(0);
}



void Utils::readImgAndDoRectify(const cv::Mat Original, cv::Mat& TargetImg, cv::Mat K, cv::Mat distCoeffs, cv::Mat new_matrix)
{
    // 这步看需不要加速计算转变为 initUndistortRectifyMap + remap
    cv::undistort(Original, TargetImg, K, distCoeffs, new_matrix);
}

cv::Mat Utils::convertAngleToR(double delta)
{
    cv::Mat R;
    delta = Utils::deg2rad(delta);
    R = (cv::Mat_<float>(3, 3) <<
            cos(delta), sin(delta), 0.0,
            -sin(delta), cos(delta), 0.0,
            0.0, 0.0, 1.0);
    return R;
}

cv::Mat Utils::updatePose(cv::Mat pose, double header_former, double header_now, double t[3])
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
    std::cout<<"delta_R: "<<Tb1b2<<std::endl;
    return Tb1b2*pose;
}



