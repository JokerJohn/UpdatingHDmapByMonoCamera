#ifndef PCL_VIEWER_H
#define PCL_VIEWER_H

#include <QMainWindow>
#include <QTimer>

#include "pcl_handle.h"

namespace Ui {
class pcl_viewer;
}

class pcl_viewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit pcl_viewer(QWidget *parent = nullptr);
    ~pcl_viewer();

private slots:
    void on_load_raw_point_clicked();

    void pcl_update();

    void on_start_cluster_clicked();

    void on_clear_data_point_clicked();

private:
    Ui::pcl_viewer *ui;
    QTimer *pcl_timer;
    pclHandle *pcl_hander;

};

#endif // PCL_VIEWER_H
