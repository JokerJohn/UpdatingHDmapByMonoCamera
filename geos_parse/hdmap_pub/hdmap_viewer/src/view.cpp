//
// Created by lfg on 19-8-1.
//

#include "view.h"


View::Viewer():pcl::visualization::PCLVisualizer ("3D Viewer")
{
    // viewer=boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PCLVisualizer::setBackgroundColor (0, 0, 0);
    pcl::visualization::PCLVisualizer::initCameraParameters ();
    pcl::visualization::PCLVisualizer::addCoordinateSystem (0.5);
}

void Viewer::addCoordinateSystem(float x,float y, float z, std::string id)
{
    pcl::visualization::PCLVisualizer::addCoordinateSystem(0.5,x,y,z,id+"CoordinateSystem",0);
}

void Viewer::addPointCloud(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud, std::string id, int r, int g, int b )
{
    // pcl::PointCloud< pcl::PointXYZRGBA >::Ptr out;
    // out = copy_pointcloud(cloud);
    if(r==0 and b==0 and g==0)
    {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
        if (!pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, id))
            pcl::visualization::PCLVisualizer::updatePointCloud(cloud, id);
    }
    else
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgb  (cloud, r, g, b);
        if (!pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, id))
            pcl::visualization::PCLVisualizer::updatePointCloud(cloud, id);
    }
}

// void Viewer::removePointCloud(std::string id)
// {
// 	pcl::visualization::PCLVisualizer::removePointCloud(id,0);
// }

// void Viewer::removeCoordinateSystem(std::string id)
// {
// 	pcl::visualization::PCLVisualizer::removeCoordinateSystem(id,0);
// }

void Viewer::update()
{
    pcl::visualization::PCLVisualizer::spinOnce (10);
}

void Viewer::drawBoundingBox(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud, std::string id)
{
    float min_x, max_x, min_y, max_y, min_z, max_z;
    getBoundingBox(cloud, min_x, max_x, min_y, max_y, min_z, max_z);
    pcl::PointXYZ pt1 (min_x, min_y, min_z);
    pcl::PointXYZ pt2 (min_x, min_y, max_z);
    pcl::PointXYZ pt3 (max_x, min_y, max_z);
    pcl::PointXYZ pt4 (max_x, min_y, min_z);
    pcl::PointXYZ pt5 (min_x, max_y, min_z);
    pcl::PointXYZ pt6 (min_x, max_y, max_z);
    pcl::PointXYZ pt7 (max_x, max_y, max_z);
    pcl::PointXYZ pt8 (max_x, max_y, min_z);
    pcl::visualization::PCLVisualizer::addLine (pt1, pt2, 1.0, 0.0, 0.0, "line_1" + id );
    pcl::visualization::PCLVisualizer::addLine (pt1, pt4, 1.0, 0.0, 0.0, "line_2" + id );
    pcl::visualization::PCLVisualizer::addLine (pt1, pt5, 1.0, 0.0, 0.0, "line_3" + id );
    pcl::visualization::PCLVisualizer::addLine (pt5, pt6, 1.0, 0.0, 0.0, "line_4" + id );
    pcl::visualization::PCLVisualizer::addLine (pt5, pt8, 1.0, 0.0, 0.0, "line_5" + id );
    pcl::visualization::PCLVisualizer::addLine (pt2, pt6, 1.0, 0.0, 0.0, "line_6" + id );
    pcl::visualization::PCLVisualizer::addLine (pt6, pt7, 1.0, 0.0, 0.0, "line_7" + id );
    pcl::visualization::PCLVisualizer::addLine (pt7, pt8, 1.0, 0.0, 0.0, "line_8" + id );
    pcl::visualization::PCLVisualizer::addLine (pt2, pt3, 1.0, 0.0, 0.0, "line_9" + id );
    pcl::visualization::PCLVisualizer::addLine (pt4, pt8, 1.0, 0.0, 0.0, "line_10" + id );
    pcl::visualization::PCLVisualizer::addLine (pt3, pt4, 1.0, 0.0, 0.0, "line_11" + id );
    pcl::visualization::PCLVisualizer::addLine (pt3, pt7, 1.0, 0.0, 0.0, "line_12" + id );
}