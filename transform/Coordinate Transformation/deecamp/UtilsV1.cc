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

// 火星坐标系转WGS84 这里统一都是先纬度后经度
void Utils::convertGCJO2ToLonlat(const Utils::new3s_PointXYZ GCJ02_coord, Utils::new3s_PointXYZ& lat_lon_coord)
{
    // 输入x是经度，有对应的是纬度     x：lat    y：lon
    double lat = GCJ02_coord.get_x();
    double lon = GCJ02_coord.get_y();
    double height = GCJ02_coord.get_z();
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
    lat_lon_coord.set_x(lat - dLat);
    lat_lon_coord.set_y(lon - dLon);
    lat_lon_coord.set_z(height);
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

void Utils::convertXYZToENU(Utils::new3s_PointXYZ xyz_coord, const Utils::new3s_PointXYZ orgxyz_coord, Utils::new3s_PointXYZ& enu_coord)
{
    /*
           inputs ::
           xyz_coord --> ECEF xyz coordinates [meter]
           orgxyz_coord --> ECEF xyz origin coordinates [meter]
           outputs ::
           osENU --> ENU position coordinates [meters]
    */
    Eigen::Vector3d posDiff;
    posDiff[0] = (xyz_coord - orgxyz_coord).get_x();
    posDiff[1] = (xyz_coord - orgxyz_coord).get_y();
    posDiff[2] = (xyz_coord - orgxyz_coord).get_z();

    Utils::new3s_PointXYZ orgLLH_coord;
    convertXYZToLLH(orgxyz_coord, orgLLH_coord);
    double sinPhi = sin(orgLLH_coord.get_x());
    double cosPhi = cos(orgLLH_coord.get_x());
    double sinLam = sin(orgLLH_coord.get_y());
    double cosLam = cos(orgLLH_coord.get_y());
    Eigen::Matrix3d R;
    R << (-1*sinLam), cosLam, 0,
    ((-1*sinPhi)*cosLam), ((-1*sinPhi)*sinLam), cosPhi,
    (cosPhi*cosLam), (cosPhi*sinLam), sinPhi ;
    Eigen::Vector3d pos;
    pos = R * posDiff;

    enu_coord.set_x(pos[0]);
    enu_coord.set_y(pos[1]);
    enu_coord.set_z(pos[2]);

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

