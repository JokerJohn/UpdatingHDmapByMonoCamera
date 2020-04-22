#ifndef PCL_HANDLE_H
#define PCL_HANDLE_H

#include <QObject>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <vector>
#include <dbscan.h>

class pclHandle : public QObject
{
    Q_OBJECT
public:
    explicit pclHandle(QObject *parent = nullptr);

    void refresh_viewer(void);

    void load_raw_point(void);

    void clear_data_point(void);

    void start_cluster(void);

signals:

public slots:

private:
    pcl::visualization::PCLVisualizer viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cp_raw;
    pcl::PointCloud<pcl::PointXYZ> *cp_labeled;

    int v1, v2;

    DBScan cluster;

    void viwer_init(void);

};

#endif // PCL_HANDLE_H
