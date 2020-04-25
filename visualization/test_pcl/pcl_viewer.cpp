#include "pcl_viewer.h"
#include "ui_pcl_viewer.h"

pcl_viewer::pcl_viewer(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::pcl_viewer) {
    ui->setupUi(this);

    pcl_timer = new QTimer(this);
    pcl_hander = new pclHandle(this);

    pcl_timer->start(10);
    connect(pcl_timer, SIGNAL(timeout()), this, SLOT(pcl_update()));
}

pcl_viewer::~pcl_viewer() {
    delete ui;
    delete pcl_hander;
    delete pcl_timer;
}

void pcl_viewer::pcl_update() {
    pcl_hander->refresh_viewer();
}

void pcl_viewer::on_load_raw_point_clicked() {
    pcl_hander->load_raw_point();
}

void pcl_viewer::on_start_cluster_clicked() {
    pcl_hander->start_cluster();
}

void pcl_viewer::on_clear_data_point_clicked() {
    pcl_hander->clear_data_point();
}
