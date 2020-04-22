#include "pcl_viewer.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    pcl_viewer w;
    w.show();

    return a.exec();
}

//#include "pcl_viewer.h"
//#include <QApplication>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/io.h>

//int main()
//{
////    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

//    //正方体点云
//    for(int i=0; i<20; i++)
//    {
//        for(int j=0; j<20; j++)
//        {
//            for(int k = 0; k<20; k++)
//            {
//                cloud->push_back(pcl::PointXYZ((i-10)/1.0f, (j-10)/1.0f, (k-10)/1.0f));
//            }
//        }
//    }

//    int v1, v2;
//    viewer.initCameraParameters();
//    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//    viewer.setBackgroundColor(255, 0, 255,v1);
//    viewer.setBackgroundColor(0, 255, 255,v2);

//    viewer.addPointCloud(cloud);
//    viewer.spin();

//    getchar();
//    getchar();

//    //viewer.showCloud(cloud);
////    while(!viewer.wasStopped());
//}
