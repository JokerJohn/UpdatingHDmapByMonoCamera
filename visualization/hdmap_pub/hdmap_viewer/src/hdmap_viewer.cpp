#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <vector>
#include <hdmap_viewer/element.h>
#include <hdmap_viewer/elements.h>
#include "Utils.h"

class cloudHandler {
public:
    cloudHandler() : viewer("HDmap Viewer") {
        pcl_sub1 = nh.subscribe("/dividers_topic", 1000, &cloudHandler::divider_callback, this);
        pcl_sub2 = nh.subscribe("/trafficlight_topic", 1000, &cloudHandler::trafficlight_callback, this);
        pcl_sub3 = nh.subscribe("/landmark_topic", 1000, &cloudHandler::landmark_callback, this);

        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);

//	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, outviewer);
        viewer.setBackgroundColor(0, 0, 0);
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
        divider_line_iter = 0;
        divider_cloud_iter = 0;
        trafficlight_line_iter = 0;
        trafficlight_cloud_iter = 0;
        lanemarking_line_iter = 0;
        lanemarking_cloud_iter = 0;
        cloud_iter = 0;
        divider_flag = false;
        trafficlight_flag = false;
        lanemarking_flag = false;
        original.set_x(22.6804);
        original.set_y(114.362);
        original.set_z(0);
    }

    int divider_line_iter;
    int divider_cloud_iter;
    int trafficlight_line_iter;
    int trafficlight_cloud_iter;
    int lanemarking_line_iter;
    int lanemarking_cloud_iter;
    int cloud_iter;
    bool divider_flag;
    bool trafficlight_flag;
    bool lanemarking_flag;
    Utils::new3s_PointXYZ original;
    Utils trans;

    void cleanallshape() {
        if (divider_flag && trafficlight_flag && lanemarking_flag) {
//            viewer.removeAllShapes();
            divider_flag = false;
            trafficlight_flag = false;
            lanemarking_flag = false;
        }
    }

    void divider_callback(const hdmap_viewer::elements &input) {
        pcl::PointCloud <pcl::PointXYZ> cloud;
        std::cout << "======one divider elements========="
                  << "elements.size:" << input.elements.size() << std::endl;
//        viewer.removeAllPointClouds();
//        viewer.removeAllShapes();
        for (int i = 0; i < divider_cloud_iter; i++) {
            viewer.removePointCloud("divider-cloud" + std::to_string(i));
        }
        cleanallshape();
        divider_cloud_iter = 0;
        divider_line_iter = 0;
        Utils::new3s_PointXYZ enu_coord;

        for (int i = 0; i < input.elements.size(); i++) {
            divider_cloud_iter++;
            std::cout << "======one divider======" << std::endl;
            pcl::fromROSMsg(input.elements[i].pointclouds, cloud);

            for (int nu = 0; nu < cloud.size(); nu++) {
                Utils::new3s_PointXYZ xyz;
                xyz.set_x(cloud[nu].x);
                xyz.set_y(cloud[nu].y);
                xyz.set_z(cloud[nu].z);
                trans.convertCJC02ToENU(xyz, enu_coord, original);
                cloud[nu].x = enu_coord.get_x();
                cloud[nu].y = enu_coord.get_y();
                cloud[nu].z = enu_coord.get_z();
//                cloud[nu].x = (cloud[nu].x *10000-226770)*30;  //226820
//                cloud[nu].y = (cloud[nu].y *10000-1143600)*30;  //1143700
//                cloud[nu].z = cloud[nu].z *10000;
            }
//            std::cout<< "cloud.x: "<< cloud[1].x <<std::endl;
//            std::cout<< "cloud.y: "<< cloud[1].y <<std::endl;
//            std::cout<< "cloud.z: "<< cloud[1].z <<std::endl;
            pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> theroyCircleCloud_color(cloud.makeShared(),
                                                                                                     255, 0, 0);
            //点云颜色渲染
            viewer.addPointCloud(cloud.makeShared(), theroyCircleCloud_color, "divider-cloud" + std::to_string(i));
            //line
            connect_points(cloud, divider_line_iter, "divider");
            //设置点云大小
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                    "divider-cloud" + std::to_string(i));
        }
        if (cloud_iter == 0) {
            viewer.resetCamera();
            cloud_iter++;
        }
        divider_flag = true;
        viewer.spinOnce();
//        viewer.updatePointCloud(cloud.makeShared(), "cloud");
    }

    void trafficlight_callback(const hdmap_viewer::elements &input) {
        pcl::PointCloud <pcl::PointXYZ> cloud;
        std::cout << "======one trafficlight elements======="
                  << "elements.size:" << input.elements.size() << std::endl;
//        viewer.removeAllPointClouds();
//        viewer.removeAllShapes();
        for (int i = 0; i < trafficlight_cloud_iter; i++) {
            viewer.removePointCloud("trafficlight-cloud" + std::to_string(i));
        }
        cleanallshape();
        trafficlight_line_iter = 0;
        trafficlight_cloud_iter = 0;
        Utils::new3s_PointXYZ enu_coord;
        std::cout << "trafficlight size " << input.elements.size() << std::endl;
        for (int i = 0; i < input.elements.size(); i++) {
            trafficlight_cloud_iter++;
            std::cout << "======one trafficlight======" << std::endl;
            pcl::fromROSMsg(input.elements[i].pointclouds, cloud);
            std::cout << "trafficlight cloud size " << cloud.size() << std::endl;
            for (int nu = 0; nu < cloud.size(); nu++) {
                Utils::new3s_PointXYZ xyz;
                xyz.set_x(cloud[nu].x);
                xyz.set_y(cloud[nu].y);
                xyz.set_z(cloud[nu].z);
                trans.convertCJC02ToENU(xyz, enu_coord, original);
                cloud[nu].x = enu_coord.get_x();
                cloud[nu].y = enu_coord.get_y();
                cloud[nu].z = enu_coord.get_z();
                std::cout << "cloud.x: " << cloud[nu].x << std::endl;
                std::cout << "cloud.y: " << cloud[nu].y << std::endl;
                std::cout << "cloud.z: " << cloud[nu].z << std::endl;
            }

            pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> theroyCircleCloud_color(cloud.makeShared(),
                                                                                                     0, 255, 0);
            //点云颜色渲染
            viewer.addPointCloud(cloud.makeShared(), theroyCircleCloud_color, "trafficlight-cloud" + std::to_string(i));
            //绘制交通灯

            //设置点云大小
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
                                                    "trafficlight-cloud" + std::to_string(i));

        }
        if (cloud_iter == 0) {
            viewer.resetCamera();
            cloud_iter++;
        }
        trafficlight_flag = true;

        viewer.spinOnce();

    }

    void landmark_callback(const hdmap_viewer::elements &input) {
        pcl::PointCloud <pcl::PointXYZ> cloud;
//        pcl::visualization::PCLVisualizer::Ptr pvis;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr  Ptrcloud;
        std::cout << "======one landmark elements======="
                  << "elements.size:" << input.elements.size() << std::endl;
//        viewer.removeAllPointClouds();
//        viewer.removeAllShapes();
        for (int i = 0; i < lanemarking_cloud_iter; i++) {
            viewer.removePointCloud("landmark-cloud" + std::to_string(i));
        }
        cleanallshape();
        lanemarking_line_iter = 0;
        lanemarking_cloud_iter = 0;
        Utils::new3s_PointXYZ enu_coord;

        for (int i = 0; i < input.elements.size(); i++) {
            lanemarking_cloud_iter++;
            std::cout << "======one landmark======" << std::endl;
            pcl::fromROSMsg(input.elements[i].pointclouds, cloud);
            for (int nu = 0; nu < cloud.size(); nu++) {
                Utils::new3s_PointXYZ xyz;
                xyz.set_x(cloud[nu].x);
                xyz.set_y(cloud[nu].y);
                xyz.set_z(cloud[nu].z);
                trans.convertCJC02ToENU(xyz, enu_coord, original);
                cloud[nu].x = enu_coord.get_x();
                cloud[nu].y = enu_coord.get_y();
                cloud[nu].z = enu_coord.get_z();
            }

            pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> theroyCircleCloud_color(cloud.makeShared(),
                                                                                                     255, 0, 0);
            //点云颜色渲染
            viewer.addPointCloud(cloud.makeShared(), theroyCircleCloud_color, "landmark-cloud" + std::to_string(i));
            //line
            connect_points(cloud, lanemarking_line_iter, "landmark");
            //设置点云大小
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                    "landmark-cloud" + std::to_string(i));

        }
        if (cloud_iter == 0) {
            viewer.resetCamera();
            cloud_iter++;
        }
        lanemarking_flag = true;

        viewer.spinOnce();

    }

    void timerCB(const ros::TimerEvent &) {
        viewer.spinOnce();
        if (viewer.wasStopped()) {
            ros::shutdown();
        }
    }

    void connect_points(pcl::PointCloud <pcl::PointXYZ> connect_points, int &iter, std::string type) {
        for (int ddd = 1; ddd < connect_points.points.size(); ++ddd) {
            pcl::PointXYZ point1 = connect_points.points[ddd];
            pcl::PointXYZ point2 = connect_points.points[ddd - 1];
            viewer.addLine<pcl::PointXYZ>(point1, point2, 255, 255, 255,
                                          type + "line" + std::to_string(iter)); //红色线段,线的名字叫做"line1
//            viewer.spinOnce(10);
            ++iter;
        }
    }

    void createTheoryCircle(pcl::PointXYZ planeNormal, pcl::PointXYZ centerPoint, double R) {
        double nx = planeNormal.x, ny = planeNormal.y, nz = planeNormal.z;
        double cx = centerPoint.x, cy = centerPoint.y, cz = centerPoint.z;
        double r = R;

        double ux = ny, uy = -nx, uz = 0;
        double vx = nx * nz,
                vy = ny * nz,
                vz = -nx * nx - ny * ny;

        double sqrtU = sqrt(ux * ux + uy * uy + uz * uz);
        double sqrtV = sqrt(vx * vx + vy * vy + vz * vz);

        double ux_ = (1 / sqrtU) * ux;
        double uy_ = (1 / sqrtU) * uy;
        double uz_ = (1 / sqrtU) * uz;

        double vx_ = (1 / sqrtV) * vx;
        double vy_ = (1 / sqrtV) * vy;
        double vz_ = (1 / sqrtV) * vz;

        double xi, yi, zi;
        double t = 0;
        double angle = (t / 180.0) * 3.14;
        std::vector<double> x, y, z;

        while (t < 360.0) {
            xi = cx + r * (ux_ * cos(angle) + vx_ * sin(angle));
            yi = cy + r * (uy_ * cos(angle) + vy_ * sin(angle));
            zi = cz + r * (uz_ * cos(angle) + vz_ * sin(angle));
            x.push_back(xi);
            y.push_back(yi);
            z.push_back(zi);

            t = t + 1;
            angle = (t / 180.0) * 3.14;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr theroyCirclePoints(new pcl::PointCloud <pcl::PointXYZ>);

        //定义cloudPoints大小,无序点云
        theroyCirclePoints->resize(x.size());
        for (int i = 0; i < x.size(); i++) {
            //将三维坐标赋值给PCL点云坐标
            (*theroyCirclePoints)[i].x = x[i];
            (*theroyCirclePoints)[i].y = y[i];
            (*theroyCirclePoints)[i].z = z[i];
        }

        //圆心点云设置
        //设置点云颜色
        viewer.removePointCloud("theroyCircleCloud");
        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> theroyCircleCloud_color(theroyCirclePoints,
                                                                                                 255, 0, 0);
        //点云颜色渲染
        viewer.addPointCloud(theroyCirclePoints, theroyCircleCloud_color, "theroyCircleCloud");
        //设置点云大小
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "theroyCircleCloud");
        viewer.resetCamera();
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub1;
    ros::Subscriber pcl_sub2;
    ros::Subscriber pcl_sub3;
    // pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
    pcl::visualization::PCLVisualizer viewer;
    int outviewer = 0, shape[500];
    int shape_num = 0;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "hdmap_viewer");

    cloudHandler handler;

    ros::spin();

    return 0;
}

