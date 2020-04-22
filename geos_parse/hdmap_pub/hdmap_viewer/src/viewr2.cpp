//
// Created by lfg on 19-8-1.
//

#include "viewr2.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PCLPointCloud2 PointCloud2;
#else
typedef sensor_msgs::PointCloud2 PointCloud2;
#endif

typedef pcl::PointXYZ Point;
typedef pcl::visualization::PointCloudColorHandler<PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef PointCloud2::Ptr PointCloudPtr;
typedef PointCloud2::ConstPtr PointCloudConstPtr;
PointCloudConstPtr cloud_, cloud_old_;
boost::mutex m;

bool paused = false;
bool record_continuously = false;
bool record_single = false;
std::string topic_name = "";

bool record_fixed_number = false;
int rec_max_frames = 100;
int rec_nr_frames = 0;
class cloudHandler {
public:
    cloudHandler() : viewer("HDmap Viewer") {
        pcl_sub1 = nh.subscribe("/dividers_topic", 1000, &cloudHandler::divider_callback, this);
        pcl_sub2 = nh.subscribe("/trafficlight_topic", 1000, &cloudHandler::trafficlight_callback, this);
        pcl_sub3 = nh.subscribe("/landmark_topic", 1000, &cloudHandler::landmark_callback, this);

        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);

//	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, outviewer);
        viewer.setBackgroundColor(0, 0, 0, outviewer);
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();

    }

    void divider_callback(const hdmap_viewer::elements& input){
        #if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
        std_msgs::Header header = pcl_conversions::fromPCL(cloud->header);
        #else
        std_msgs::Header header = cloud->header;
        #endif
        float stamp = header.stamp.toSec ();

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::visualization::PCLVisualizer::Ptr pvis;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr  Ptrcloud;
        //Ptrcloud=cloud.makeShared();
        std::cout<< "======one divider elements=========" <<std::endl;
        for(int i=0;i< input.elements.size();i++){

            pcl::fromROSMsg(input.elements[i].pointclouds, cloud);
            std::cout<< "points size"<<cloud.points.size() <<std::endl;
            viewer.addPointCloud<pcl::PointXYZ> (cloud.makeShared(), std::to_string(i), outviewer);
            connect_points(cloud);
        }
        std::cout<< "======end==========" <<std::endl;

//        viewer.removeAllPointClouds(outviewer);
//        for(int i=0;i<cloud.points.size();++i){
//            viewer.removeShape(std::to_string(i));
//        }

        viewer.updatePointCloud(cloud.makeShared(), "cloud");
    }

    void trafficlight_callback(const hdmap_viewer::elements& input){

    }

    void landmark_callback(const hdmap_viewer::elements& input){

    }

    void timerCB(const ros::TimerEvent&){
        viewer.spinOnce();
        if (viewer.wasStopped()){
            ros::shutdown();
        }
    }

    void connect_points(pcl::PointCloud<pcl::PointXYZ> connect_points){
        for(int i=1;i<connect_points.points.size();++i ){
            pcl::PointXYZ point1 = connect_points.points[i];
            pcl::PointXYZ point2 = connect_points.points[i-1];
            viewer.addLine<pcl::PointXYZ>(point1, point2, 255, 0, 0, std::to_string(i)); //红色线段,线的名字叫做"line1

        }
    }


protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub1;
    ros::Subscriber pcl_sub2;
    ros::Subscriber pcl_sub3;
    // pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
    pcl::visualization::PCLVisualizer viewer;
    int outviewer=0,shape[500];
    int shape_num=0;

};



int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_viewer2", ros::init_options::AnonymousName);
    cloudHandler handler;



    int queue_size = 1;
    pcl::console::parse_argument (argc, argv, "-qsize", queue_size);

    bool headless = false;
//    ros::NodeHandle private_nh("~");
//    private_nh.getParam("headless", headless);
//    if (headless)
//    {
//        ROS_WARN("Running in headless mode. All point clouds are written to disk");
//        record_continuously = true; // no viewer window, all point clouds are written to disk
//    }

    ros::Subscriber sub = nh.subscribe ("kitti_player/hdl64e", queue_size, cloud_cb);
    topic_name = ros::names::remap("input").c_str();
    pcl::console::print_highlight("Subscribing to %s using a queue size of %d\n", topic_name.c_str(), queue_size);

    pcl::visualization::PCLVisualizer::Ptr p;
    ColorHandlerPtr color_handler;
    pcl::PointCloud<Point>::Ptr cloud_xyz;

    if (!headless)
    {
        p.reset(new pcl::visualization::PCLVisualizer(argc, argv, "Online PointCloud2 Viewer"));
        cloud_xyz.reset(new pcl::PointCloud<Point>);
        p->registerKeyboardCallback(keyboardEventOccurred, (void*)&p);
    }

    int color_handler_idx = 0;
    double psize = 0;
    while (nh.ok ())
    {
        ros::spinOnce ();
        ros::Duration (0.001).sleep ();

        if (headless)
        {
            ros::Duration (0.001).sleep ();
        }
        else
        {
            p->spinOnce (10);

            if (!cloud_)
                continue;

            if (cloud_ == cloud_old_)
                continue;

            color_handler_idx = p->getColorHandlerIndex ("cloud");
            p->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");
            p->removePointCloud ("cloud");
            m.lock ();
            {
                // filter out NaNs
                pcl::PassThrough<PointCloud2> filter;
                PointCloud2::Ptr cloud_filtered(new PointCloud2);
                filter.setInputCloud(cloud_);
                filter.filter(*cloud_filtered);

                // convert point cloud to PCL PointCloud type
#if (defined PCL_MINOR_VERSION && (PCL_MINOR_VERSION >= 7))
                pcl::fromPCLPointCloud2 (*cloud_filtered, *cloud_xyz);
#else
                pcl::fromROSMsg (*cloud_, *cloud_xyz);
#endif

                // create color handlers
                color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<PointCloud2> (cloud_, 255.0, 1.0, 1.0));
                p->addPointCloud<Point>(cloud_xyz, color_handler, "cloud");
                for (size_t i = 0; i < cloud_->fields.size (); ++i)
                {
                    if (cloud_->fields[i].name == "rgb" || cloud_->fields[i].name == "rgba")
                        color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud2> (cloud_));
                    else
                        color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<PointCloud2> (cloud_, cloud_->fields[i].name));
                    p->addPointCloud<Point>(cloud_xyz, color_handler, "cloud");
                }
                p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");
                if (color_handler_idx != -1)
                    p->updateColorHandlerIndex ("cloud", color_handler_idx);
                cloud_old_ = cloud_;
            }
            m.unlock ();
        }

    }

    ros::spin();
    return (0);
}
