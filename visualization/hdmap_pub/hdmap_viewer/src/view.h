#ifndef _VIEWER_H
#define _VIEWER_H

#include <fstream>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <boost/filesystem.hpp>
// #include <QTimer>
// #include "config.h"
//#include "computepointcloud/computepointcloud.h"

//using namespace computepointcloud;

class Viewer: public pcl::visualization::PCLVisualizer
{
private:
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
public:
    Viewer();
    void addCoordinateSystem(float x,float y, float z, std::string id);
    void addPointCloud(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud, std::string id, int r, int g, int b);
    // void removePointCloud(std::string id);
    // void removeCoordinateSystem(std::string id);
    void update();
    void drawBoundingBox(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud, std::string id);
};


#endif