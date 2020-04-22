//
//    create by jianghc 2019.7.28
//
#include <string>
#include <Eigen/Dense> //稠密矩阵代数运算,迹,特征值
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
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

	new3s_PointXYZ operator -(const new3s_PointXYZ &other)
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

double a = 6378245.0; // 卫星椭球坐标投影因子
double F = 298.257223563; // 曲率
double iPI = 0.0174532925199433; // 角单位 度
double f = 1/F; // 扁率
cv::Mat K;
cv::Mat distCoeffs;
cv::Mat Rcb;
cv::Mat mTcb;
