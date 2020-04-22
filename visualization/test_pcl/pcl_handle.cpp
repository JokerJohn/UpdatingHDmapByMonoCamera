#include "pcl_handle.h"
#include <QTime>


pclHandle::pclHandle(QObject *parent) :
    QObject(parent),
    viewer("Cloud Viewer"),
    cp_raw(new pcl::PointCloud<pcl::PointXYZ>()),
    cluster(2, 4)
{
    qsrand(static_cast<uint>(QTime(0, 0, 0).secsTo(QTime::currentTime())));

    viwer_init();
}

void pclHandle::refresh_viewer(void)
{
    viewer.spinOnce();
}

float generateRand(float scale)
{
    return scale * qrand() / float(RAND_MAX) * 2 - scale;
}

void pclHandle::viwer_init(void)
{
    viewer.initCameraParameters();
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.setBackgroundColor(0, 0, 0, v2);
}

void pclHandle::load_raw_point(void)
{
    float random_r, x, y, z;
    for (int i = 0;i < 100;i++)
    {
        random_r = 3;
        x = 0 + generateRand(random_r);
        y = 0 + generateRand(random_r);
        z = 0 + generateRand(random_r);
        cp_raw->push_back(pcl::PointXYZ(x, y, z));
        cluster.append_datapoint(x, y, z);

        random_r = 3;
        x = 7 + generateRand(random_r);
        y = 8 + generateRand(random_r);
        z = 9 + generateRand(random_r);
        cp_raw->push_back(pcl::PointXYZ(x, y, z));
        cluster.append_datapoint(x, y, z);

        random_r = 3;
        x = -7 + generateRand(random_r);
        y = -8 + generateRand(random_r);
        z = -9 + generateRand(random_r);
        cp_raw->push_back(pcl::PointXYZ(x, y, z));
        cluster.append_datapoint(x, y, z);

        random_r = 50;
        x = 0 + generateRand(random_r);
        y = 0 + generateRand(random_r);
        z = 0 + generateRand(random_r);
        cp_raw->push_back(pcl::PointXYZ(x, y, z));
        cluster.append_datapoint(x, y, z);
    }
    pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> cp_raw_color(cp_raw, 255, 0, 0);
    viewer.addPointCloud(cp_raw, cp_raw_color, "raw_cloudpoint", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloudpoint");
}

void pclHandle::clear_data_point(void)
{
    cp_raw->clear();
    cluster.clear_datapoint();
    viewer.removeAllPointClouds(v1);
    viewer.removeAllPointClouds(v2);
}

void pclHandle::start_cluster(void)
{
    cluster.start_dbscan();

    cp_labeled = new pcl::PointCloud<pcl::PointXYZ>[cluster.clusterId + 1];
    pcl::PointCloud<pcl::PointXYZ> cp_x;
    for (unsigned long j = 0;j < cluster.dataset.size();j++)
    {
        if (cluster.dataset[j].labelID != -1) // clustered
            cp_labeled[cluster.dataset[j].labelID].push_back(pcl::PointXYZ(cluster.dataset[j].x, cluster.dataset[j].y, cluster.dataset[j].z));
        else
            cp_labeled[cluster.clusterId].push_back(pcl::PointXYZ(cluster.dataset[j].x, cluster.dataset[j].y, cluster.dataset[j].z));
    }

    for (int i = 0;i < cluster.clusterId;i++)
    {
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cp_color(cp_labeled[i].makeShared());
        viewer.addPointCloud(cp_labeled[i].makeShared(), cp_color, "class"+std::to_string(i), v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "class"+std::to_string(i));
    }

    // not clustered = red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cp_color(cp_labeled[cluster.clusterId].makeShared(), 255, 0, 0);
    viewer.addPointCloud(cp_labeled[cluster.clusterId].makeShared(), cp_color, "not clustered class", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "not clustered class");

    delete[] cp_labeled;
}
