//
// Created by catalina on 2019/7/30.
//
#ifndef PROTO_TEST_MAP_READ_H
#define PROTO_TEST_MAP_READ_H
#include "../utils/proto.h"
#include "../utils/calulate.h"
#include "../proto/HDMap.pb.h"
#include "../proto/LaneMarking.pb.h"
#include "../proto/SourceInfo.pb.h"
#include "../proto/TrafficLight.pb.h"
#include "tiff.h"
#include <vector>
#include "Utils.h"
#include<iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
//#include "consts.h"
using namespace std;
using namespace cv;
//#include "read_detection.h"

//点位置
typedef struct{
    int id;
    double x;
    double y;
    double z;
}PointT;

//车道线
typedef struct{
    //编号
    int32  id;
//位置WKT
    vector <PointT>  points_vec;
//    string geometry;
//颜色，0：未知；1：白色；2：黄色
    int32  color;
//类型 0:未知； 1：单虚线；2：单实线；3：双实线；4：左实右虚；5：左虚右实,；6:不可通行减速线；7：可通行减速线
    int32  type;
//遮挡程度，1：无；2：部分遮挡；3：全部遮挡(invalid)
    int32  occlusion;
}DividerEach;

typedef struct{
//编号
    int32  id;
//位置WKT
//    string  geometry;
//类型，1：竖排；2：横排
    PointT points;
    int32  direction;
}LightEach;

typedef struct{
//编号
    int32  id;
//位置WKT
//    PointT point;
    PointT point;
//类型，1：车辆；2：行人

    int32  motor_type;
//灯
   vector<LightEach> lights;
}TrafficLightEach;
typedef struct{
//编号
    int32  id;
//位置WKT
//    string  geometry;
//类型，1：竖排；2：横排
    vector<PointT> points_vec;
    int32  direction;
}LightEachShow;
//读取结果
typedef struct{
//编号
    int32  id;
//位置WKT
//    PointT point;
    vector<PointT> point_vec;
//类型，1：车辆；2：行人

    int32  motor_type;
//灯
    vector<LightEachShow> lights;
}TrafficLightEachShow;

//地面标识
typedef struct{
//编号
    int32 id;
//位置 WKT
//    vector<struct PointT> points;
    PointT points[5];
//类型，1：人行道；2：停止线；3：地面标识
    int32 type;
}LaneMarkingEach;

//GPS point
typedef struct {
    //编号
    int32 id;
//时间
    int64 gpstime;
//航向
    double heading;
//速度
    double speed;
//坐标 WKT
    PointT points;
}GPSPointEach;

// GPS
typedef struct {
    string scene_id;
//设备名称
    string device_id;
//GPS点集合
    vector< GPSPointEach> gpsPoints;
}GPSInfoEach;

typedef struct{
//    GPS帧id
    string scene_id;
//    对应image帧集合
    vector<string> images_vec;

}ImageBatch;
//HDMap
typedef struct {
    //场景id，视频文件名
    string  scene_id;
//车道线
    vector<DividerEach> dividers;
//交通灯
    vector<TrafficLightEach> tafficlights;
//地面标识
    vector<LaneMarkingEach> lanemarkings;
    string  version;
}HDMAP;
/*typedef struct
{
    string scene_id;
    vector<DividerEach> divider_vec;
    vector<TrafficLightEachShow> traffic_lights_vec;
}DetectionBatch;*/
typedef struct {
//    gps帧id
    string scene_id;
//    对应图片文件名
    string image_name;
    GPSPointEach gpsPoint;
}GpsImageBatch;


typedef struct {
    string frame_id;
    vector<DividerEach> dividerEach_vec;

}DetectionDividerPerFrame;

typedef struct {
    string scene_id;
//    vector<>
    vector<DetectionDividerPerFrame> dividerPerFrame_vec;

}DetectionDividerPerCapture;

typedef struct {
    string frame_id;
    vector<TrafficLightEachShow> trafficLight_vec;

}DetectionTrafficPerFrame;

typedef struct {
    string scene_id;
//    vector<>
    vector<DetectionTrafficPerFrame> trafficPerFrame_vec;

}DetectionTrafficPerCapture;

typedef struct {
//    gps帧id
    string scene_id;
//    对应图片文件名
    string image_name;
    GPSPointEach point;
    DetectionTrafficPerFrame trafficPerFrame;
    DetectionDividerPerFrame dividerPerFrame;
}DetchBatch;


namespace ReadHDMap {
    const string gps_file_folder = "../data/gps";
    const string images_file_folder = "../data/images";
    const string detection_result_flie_folder = "../data/detection_result";
    const std::string hdmap_file_name = "../data/hdmap/hdmap_deecamp.pb";


    PointT transform2ENU(PointT point)
    {
        Utils::new3s_PointXYZ original;
        original.set_x(22.68085991);
        original.set_y(114.36478212);
        original.set_z(0);

        Utils::new3s_PointXYZ enu_coord_1;
        Utils transform;

        Utils::new3s_PointXYZ xyz;
        xyz.set_x(point.x);
        xyz.set_y(point.y);
        xyz.set_z(point.z);
        transform.convertCJC02ToENU(xyz, enu_coord_1,original);
        PointT point2;

        point2.x =  enu_coord_1.get_x();
        point2.y =  enu_coord_1.get_y();
        point2.z =  enu_coord_1.get_z();
        return point2;
    }
    /**
     * 解析高精地图所有元素,按index顺序输出
     * @return
     */
    HDMAP getHDMAP() {

        fstream input(hdmap_file_name, ios::in | ios::binary);
        hdmap::HDMap map;
        map.ParseFromIstream(&input);

        HDMAP readMap; //高精地图所有数据
        readMap.version = map.version();
        readMap.scene_id = map.scene_id();

        LaneMarkingEach laneMarkingEach; //高精地图中的所有路标
        TrafficLightEach trafficLightEach;//高精地图中的所有交通灯
        DividerEach dividerEach;//高精地图中的所有车道线

//        cout << "lanemarkings_size size:" << map.lanemarkings_size() << endl;
        //    读取所有的地面标识
        for (int i = 0; i < map.lanemarkings_size(); ++i) {
            hdmap::LaneMarking laneMarking = map.lanemarkings(i);
            string geometry = laneMarking.geometry();
            laneMarkingEach.id = laneMarking.id();
            laneMarkingEach.type = laneMarking.type();
//        提取geomery字段中的坐标数据
            vector<double> geom;
            calulate::extractFiguresFromStr2Vec(laneMarking.geometry(), geom);

//        cout << std::setprecision(14) << laneMarkingEach.id <<"--------- "<< endl;
            if (geom.size() == 15) {
//            cout << geom[0] << " " << geom[1] << " " << geom[2] << endl;
                for (int j = 0; j < 5; ++j) {
                    PointT point;
                    point.x = geom[3 * j];
                    point.y = geom[3 * j + 1];
                    point.z = geom[3 * j + 2];

                    PointT pointT = transform2ENU(point);
                    laneMarkingEach.points[j].x = pointT.x;
                    laneMarkingEach.points[j].y = pointT.y;
                    laneMarkingEach.points[j].z = pointT.z;
//                cout << std::setprecision(14)<< ""<< laneMarkingEach.points[j].x << "," << laneMarkingEach.points[j].y
//                     << "," << laneMarkingEach.points[j].z << endl;
                }
                readMap.lanemarkings.push_back(laneMarkingEach);
            } else {
                cout << "laneMarking 位置读取有误 " << endl;
            }
        }

        cout << "tafficlights_size size:" << map.tafficlights_size() << endl;
//    读取所有的交通灯
        for (int k = 0; k < map.tafficlights_size(); ++k) {
            hdmap::TrafficLight trafficLight = map.tafficlights(k);
            trafficLightEach.id = trafficLight.id();
            trafficLightEach.motor_type = trafficLight.motor_type();
            string geometry = trafficLight.geometry();
//            cout << "traffic light:" << geometry <<endl;
            vector<double> geom;
            calulate::extractFiguresFromStr2Vec(geometry, geom);

            if (geom.size() == 3) {
//                trafficLightEach.point_vec
                PointT point1,pointT1;
                point1.x = geom[0];
                point1.y = geom[1];
                point1.z = geom[2];

                pointT1 = transform2ENU(point1);
                trafficLightEach.point.x = pointT1.x;
                trafficLightEach.point.y = pointT1.y;
                trafficLightEach.point.z = pointT1.z;

//            cout << std::setprecision(14) << ""<<  trafficLightEach.point.x << "," << trafficLightEach.point.y
//                 << "," << trafficLightEach.point.z << endl;
//            Light 里面的数据继续查询

                LightEach lightEach; //第二次进来，lightEach归0
                for (int i = 0; i < trafficLight.lights_size(); ++i) {
                    lightEach.id = trafficLight.lights(i).id();
                    lightEach.direction = trafficLight.lights(i).direction();

                    vector<LightEach> lights_vector;
                    vector<double> geom2;
                    calulate::extractFiguresFromStr2Vec(trafficLight.lights(i).geometry(), geom2);
                    if (geom2.size() == 3) {
                        PointT point;
                        point.x = geom2[0];
                        point.y = geom2[1];
                        point.z = geom2[2];

                        PointT pointT = transform2ENU(point);
                        cout << "" << pointT.x << "," << pointT.y << "," << pointT.z << endl;

                       /* lightEach.points.x = geom2[0];
                        lightEach.points.y = geom2[1];
                        lightEach.points.z = geom2[2];*/
                         lightEach.points.x = pointT.x;
                         lightEach.points.y = pointT.y;
                         lightEach.points.z = pointT.z;
                        lights_vector.push_back(lightEach);
                    }
                    trafficLightEach.lights = lights_vector;
                }
                trafficLightEach.lights.push_back(lightEach);
            } else {
                cout << "trafficLights 位置读取有误 " << endl;
            }
            readMap.tafficlights.push_back(trafficLightEach);
        }

//        ------------------------------------------------------------------
        cout << "dividers_size size:" << map.dividers_size() << endl;
        for (int l = 0; l < map.dividers_size(); ++l) {
            hdmap::Divider divider = map.dividers(l);
            dividerEach.id = divider.id();
            dividerEach.type = divider.type();
            dividerEach.color = divider.color();
            dividerEach.occlusion = divider.occlusion();


//        cout << "geom----"<<geom.size()/3 <<" ----" << geom[0] << " " << geom[1] << " " << geom[2] << endl;
//            int pointsNum = geom.size() / 3;
            cout << "----divider geomery-----" <<divider.geometry()<<endl;
            vector<double> geom;
            calulate::extractFiguresFromStr2Vec(divider.geometry(), geom);
//            cout << "----geom size-----" <<geom.size()<<endl;
            for (int i = 0; i < geom.size()/3; ++i) {
                PointT point3;
                point3.x = geom[i * 3];
                point3.y = geom[i * 3 + 1];
                point3.z = geom[i * 3 + 2];
                PointT pointT3 = transform2ENU(point3);
                dividerEach.points_vec.push_back(pointT3);
//                cout << "----divider pointT-----" << dividerEach.divider_vec[i].x << " ," << dividerEach.divider_vec[i].y << " , " << dividerEach.divider_vec[i].z << endl;
            }
//        cout << "----id----"<< dividerEach.id << endl;
            readMap.dividers.push_back(dividerEach);
        }
        return readMap;
    }

    bool getGPSInfoBySceneId(string scene_id, GPSInfoEach &gps) {
//        std::cout << "scene_id: " << scene_id << endl;
        string file_name = "../data/gps/" + scene_id + "deecamp_gps.pb";
//        std::cout << "file_name: " << file_name << endl;

        fstream input_gps(file_name, ios::in | ios::binary);
        source::GPSInfo gpsInfo;
        gpsInfo.ParseFromIstream(&input_gps);
        source::GPSPoint gpsPoint;

        if (gpsInfo.scene_id()==scene_id) {
//        一帧GPS数据信息
            gps.scene_id = gpsInfo.scene_id();

            for (int l = 0; l < gpsInfo.pts_size(); ++l) {
                source::GPSPoint pts = gpsInfo.pts(l);
                GPSPointEach gpsPointEach;
                gpsPointEach.heading = pts.heading();
                gpsPointEach.id = pts.id();
                gpsPointEach.gpstime = pts.gpstime();
                gpsPointEach.speed = pts.speed();
//        处理geometry
                vector<double> geom;
                calulate::extractFiguresFromStr2Vec(pts.geometry(), geom);
                gpsPointEach.points.x = geom[0];
                gpsPointEach.points.y = geom[1];
                gpsPointEach.points.z = geom[2];
//                坐标转换
//                PointT pointT3 = transform2ENU(gpsPointEach.points);
//                gpsPointEach.points.x = pointT3.x;
//                gpsPointEach.points.y = pointT3.y;
//                gpsPointEach.points.z = pointT3.z;

             /*   cout << "原始:" << gpsPointEach.points.x << " , " << gpsPointEach.points.y << " ," << gpsPointEach.points.z << endl;
                cout << "enu:" << pointT3.x << " , " <<pointT3.y << " ," << pointT3.z << endl;*/

                gps.gpsPoints.push_back(gpsPointEach);

//            std::cout << std::setprecision(14) <<geom[0] << " " << geom[1] << " " <<geom[2]<<endl;
//            std::cout << "pts:" << pts.geometry().substr(9, 34)<< endl;
                //        gpsInfoEach.gpsPoints.push_back(gpsPointEach);
            }
            return true;
        } else {
            std::cout << "不存在此帧数据: " << endl;
            return false;
        }


    }
    /**
     * 获取每一帧GPS信息并按时间顺序输出
     * @param gpsInfo_vec
     * @return
     */
    bool getGPSInfo(vector<GPSInfoEach> &gpsInfo_vec)
    {
        vector<string> gpsFileNames; //GPS文件名
        //    1.读gps, 2读image
        bool flag = calulate::getAllFiles(gps_file_folder, gpsFileNames);
        if (flag)
        {
            // 读取之后对vector内的image和gps进行排序
            calulate::sortedVector(gpsFileNames);
//        遍历每一帧gps获取sceneId
            for (int i = 0; i < gpsFileNames.size(); i++) {
//            获取scene id
                int nops = gpsFileNames[i].find_first_of(".");
                string scene_id = gpsFileNames[i].substr(0, nops-11);
                string gps_file_name = "../data/gps/"+scene_id+"deecamp_gps.pb";
                fstream input_gps(gps_file_name, ios::in | ios::binary);
                source::GPSInfo gpsInfo;
                gpsInfo.ParseFromIstream(&input_gps);
                source::GPSPoint pts;

//                cout << "-----gps scene_id:------" << scene_id<<endl;
//                读每一帧的gps点 存在gpsInfoEach里面
                GPSInfoEach gpsInfoEach;
                vector<GPSPointEach> gpsPoint_vec = gpsInfoEach.gpsPoints;
                gpsInfoEach.scene_id = scene_id;
//                cout << "-----gps point size:------" << gpsInfo.pts_size()<<endl;

//每一帧gps点集封装
                for (int j = 0; j < gpsInfo.pts_size(); j++) {
                    GPSPointEach gpsPointEach;
                    gpsPointEach.heading = gpsInfo.pts(j).heading();
                    gpsPointEach.gpstime = gpsInfo.pts(j).gpstime();
                    gpsPointEach.id = gpsInfo.pts(j).id();
                    gpsPointEach.speed = gpsInfo.pts(j).speed();

                    string geomerys = gpsInfo.pts(j).geometry();
                    vector <double> geom;
                    calulate::extractFiguresFromStr2Vec(geomerys,geom);
                    gpsPointEach.points.x = geom[0];
                    gpsPointEach.points.y = geom[1];
                    gpsPointEach.points.z = geom[2];
//                    cout << "原始:" << gpsPointEach.points.x << " , " << gpsPointEach.points.y << " ," << gpsPointEach.points.z << endl;
//                    坐标转换
                    PointT pointT3 = transform2ENU(gpsPointEach.points);
                    gpsPointEach.points.x = pointT3.x;
                    gpsPointEach.points.y = pointT3.y;
                    gpsPointEach.points.z = pointT3.z;

//                    cout << "enu:" << pointT3.x << " , " <<pointT3.y << " ," << pointT3.z << endl;
                    gpsInfoEach.gpsPoints.push_back(gpsPointEach);
                    gpsPoint_vec.push_back(gpsPointEach);
//                    transform2ENU(gpsPointEach);
                }
                gpsInfo_vec.push_back(gpsInfoEach);
//                cout << "-----gpsPoint id------" << gpsPoint_vec.size() <<endl;
            }
//            cout << "-----gpsInfo_vec size------" << gpsInfo_vec.size() <<endl;
            return true;
        } else{
            cout << "fileName为空"<< endl;
            return false;
        }
    }


    /**
     * 获取每一帧Gps对应的image图片集,按vector index索引顺序输出
     * @param imageBatch_vec image文件名(不包括后缀)
     * @return
     */
    bool getAllImageBatch(vector<ImageBatch> &imageBatch_vec) {

        vector<string> gpsFileNames; //GPS文件名
        vector<string> imagesFileNames;//所有image文件名
//        vector<ImageBatch> dataFiles; //装好的scieId和Image集合
        //    1.读gps, 2读image
        bool flag = calulate::getAllFiles(gps_file_folder, gpsFileNames);
        bool flag2 = calulate::getAllFiles(images_file_folder, imagesFileNames);

//        cout<<"flag1:"<< flag <<" flag2:"<< flag2<<endl;
        if (flag && flag2) {
            //// 读取之后对vector内的image和gps进行排序
            calulate::sortedVector(gpsFileNames);
            calulate::sortedVector(imagesFileNames);
//            cout << "gps size:" << gpsFileNames.size() << endl;
//            cout << "image size:" << imagesFileNames.size() << endl;

//        获取sceneId
            for (int i = 0; i < gpsFileNames.size(); ++i) {
                ImageBatch imageBatch;
//            获取scene id
                int nops = gpsFileNames[i].find_first_of(".");
                string scene_id = gpsFileNames[i].substr(0, nops - 11);
//                cout << "scene_id:" << scene_id<< endl;
                imageBatch.scene_id = scene_id;

//        根据scene_id去查询images id，返回images vetor
                vector<string> image_vec;
                for (int j = 0; j < imagesFileNames.size(); ++j) {
                    int nops_image = imagesFileNames[j].find_first_of(".");
                    string image_id = imagesFileNames[j].substr(0, nops_image);
//                cout << "image_id:" << image_id << endl;

                    string::size_type idx;
                    idx = image_id.find(scene_id);//在a中查找b.
                    if (idx != string::npos) {
                        //存在, 装进去IMages
                        image_vec.push_back(image_id);
                    }
                }
//                cout << "-----image_vec:------" << image_vec.size()<<endl;
                imageBatch.images_vec = image_vec;
                imageBatch_vec.push_back(imageBatch);
            }
            return true;
//            cout << "dfgfb:"<< dataFiles.size()<<" dfgfd:"<< dataFiles[100].images_vec.size()<< endl;
        } else {
            cout << "fileName为空" << endl;
            return false;
        }
    }

    /**
     * 通过scene_id获取对应的vetor图片集合
     * @param scene_id2
     * @param imageBatch
     * @return
     */
    bool getImageBatchBySceneId(string scene_id2, ImageBatch &imageBatch) {
        vector<string> gpsFileNames; //GPS文件名
        vector<string> imagesFileNames;//所有image文件名
//        vector<ImageBatch> dataFes; //装好的scieId和Image集合
        //    1.读gps, 2读image
        bool flag = calulate::getAllFiles(gps_file_folder, gpsFileNames);
        bool flag2 = calulate::getAllFiles(images_file_folder, imagesFileNames);

        if (flag && flag2) {
            //// 读取之后对vector内的image和gps进行排序
            calulate::sortedVector(gpsFileNames);
            calulate::sortedVector(imagesFileNames);
//            ImageBatch imageBatch;

            for (int i = 0; i < gpsFileNames.size(); ++i) {
                int nops = gpsFileNames[i].find_first_of(".");
                string scene_id = gpsFileNames[i].substr(0, nops - 11);

                if (scene_id==scene_id2) {
//                    cout << "查找到scene_id维 " << scene_id<< endl;
                    imageBatch.scene_id = scene_id2;

                    vector<string> image_vec;
                    for (int j = 0; j < imagesFileNames.size(); ++j) {
                        int nops_image = imagesFileNames[j].find_first_of(".");
                        string image_id = imagesFileNames[j].substr(0, nops_image);

                        string::size_type idx;
                        idx = image_id.find(scene_id);//在a中查找b.
                        if (idx != string::npos) {
                            //存在, 装进去IMages
                            imageBatch.images_vec.push_back(image_id);
                        }
                    }
//                    cout << "-----image_vec:------" << image_vec.size()<<endl;
//                    imageBatch.images_vec = image_vec;
                    return true;
                }
            }
        } else {
            cout << "fileName为空" << endl;
            return false;
        }
    }

    /**
     * 获取每一个GPS点对应的image
     * @param gpsImageBatch_vec
     * @return
     */
    bool getAllGpsImageBatch(vector<GpsImageBatch> &gpsImageBatch_vec) {
        vector<string> gpsFileNames; //GPS文件名
        vector<string> imagesFileNames;//所有image文件名
//        vector<GpsImageBatch> dataFiles; //装好的scieId和Image集合
        //    1.读gps, 2读image
        bool flag = calulate::getAllFiles(gps_file_folder, gpsFileNames);
        bool flag2 = calulate::getAllFiles(images_file_folder, imagesFileNames);

        if (flag == 0 && flag2 == 0) {
            //// 读取之后对vector内的image和gps进行排序
            calulate::sortedVector(gpsFileNames);
            calulate::sortedVector(imagesFileNames);
//            cout << "gps size:" << gpsFileNames.size() << endl;
//            cout << "image size:" << imagesFileNames.size() << endl;

//        遍历每一帧gps获取sceneId
            for (int i = 0; i < gpsFileNames.size(); ++i) {
                GpsImageBatch gpsImageBatch;
//            获取scene id
                int nops = gpsFileNames[i].find_first_of(".");
                string scene_id = gpsFileNames[i].substr(0, nops - 11);
                gpsImageBatch.scene_id = scene_id;

//        根据scene_id去查询images id，返回images vetor
                vector<string> image_vec;
                for (int j = 0; j < imagesFileNames.size(); ++j) {
                    int nops_image = imagesFileNames[j].find_first_of(".");
                    string image_id = imagesFileNames[j].substr(0, nops_image);
//                cout << "image_id:" << image_id << endl;

                    string::size_type idx;
                    idx = image_id.find(scene_id);//在a中查找b.
                    if (idx != string::npos) {
                        //存在, 装进去IMages
                        image_vec.push_back(image_id);
                    }
                }
                calulate::sortedVector(image_vec);//图片编号排序
//                读每一帧的gps点
                GPSInfoEach gpsInfoEach;
                bool flage = getGPSInfoBySceneId(scene_id, gpsInfoEach);

                cout << "-----gpsInfoEach!------" << gpsInfoEach.gpsPoints.size() << "-------image------------"
                     << image_vec.size() << endl;


                if (gpsInfoEach.gpsPoints.size() == image_vec.size()) {
                    cout << "-----GPS数据点和image数量已匹配!------" << image_vec.size() << endl;
                    for (int j = 1; j < gpsInfoEach.gpsPoints.size(); ++j) {

                        gpsImageBatch.gpsPoint.heading = gpsInfoEach.gpsPoints[j].heading;
                        gpsImageBatch.gpsPoint.id = gpsInfoEach.gpsPoints[j].heading;
                        gpsImageBatch.gpsPoint.speed = gpsInfoEach.gpsPoints[j].speed;
                        gpsImageBatch.gpsPoint.gpstime = gpsInfoEach.gpsPoints[j].gpstime;
                        gpsImageBatch.gpsPoint.points.x = gpsInfoEach.gpsPoints[j].points.x;
                        gpsImageBatch.gpsPoint.points.y = gpsInfoEach.gpsPoints[j].points.y;
                        gpsImageBatch.gpsPoint.points.z = gpsInfoEach.gpsPoints[j].points.z;

                        gpsImageBatch.image_name = image_vec[j];
//                        cout << "-----image id------" << image_vec[j] << "----point id----"<< gpsImageBatch.gpsPoint.id<<endl;
                    }
                    gpsImageBatch_vec.push_back(gpsImageBatch);
                    return true;
                } else {
                    cout << "-----此帧GPS数据点和image数量不对应!------" << image_vec.size() << endl;
                    return false;
                }
            }
        } else {
            cout << "fileName为空" << endl;
            return false;
        }
    }
    /**
       * 通过images_id获取每一个GPS点对应的image
       * @param gpsImageBatch_vec
       * @return
       */
    bool getGpsImageBatchByImageId(string scene_id2, int index, GpsImageBatch &gpsImageBatch) {
        vector<string> gpsFileNames; //GPS文件名
        vector<string> imagesFileNames;//所有image文件名
        //    1.读gps, 2读image
        bool flag = calulate::getAllFiles(gps_file_folder, gpsFileNames);
        bool flag2 = calulate::getAllFiles(images_file_folder, imagesFileNames);

        if (flag && flag2) {
            //// 读取之后对vector内的image和gps进行排序
            calulate::sortedVector(gpsFileNames);
            calulate::sortedVector(imagesFileNames);

//            GpsImageBatch gpsImageBatch;
//        遍历每一帧gps获取sceneId
            for (int i = 0; i < gpsFileNames.size(); ++i) {
                int nops = gpsFileNames[i].find_first_of(".");
                string scene_id = gpsFileNames[i].substr(0, nops - 11);

                if (scene_id==scene_id2) {
//                    获取此帧的GPSInfo
                    gpsImageBatch.scene_id = scene_id2;
//                    此帧的Point集合
                    GPSInfoEach gpsInfoEach;
                    bool flag = getGPSInfoBySceneId(scene_id, gpsInfoEach);
                    vector<GPSPointEach> gpsPointEach = gpsInfoEach.gpsPoints;
                    int size = gpsPointEach.size();
                    if (size > 0 && index < size && index > 0) {
                        gpsImageBatch.gpsPoint.points = gpsPointEach[index].points;
                        gpsImageBatch.gpsPoint.gpstime = gpsPointEach[index].gpstime;
                        gpsImageBatch.gpsPoint.speed = gpsPointEach[index].speed;
                        gpsImageBatch.gpsPoint.heading = gpsPointEach[index].heading;
//                        cout << "----GPS Point---"<< gpsImageBatch.gpsPoint.points.x<< ", "<<gpsImageBatch.gpsPoint.points.y<< ", "<<gpsImageBatch.gpsPoint.points.z << "---GPS heading---"<< gpsImageBatch.gpsPoint.heading
//                        << "---- ----" <<  endl;
                    }
                }
            }
            return true;
        } else {
            cout << "fileName为空" << endl;
            return false;
        }
    }

    /**
     *  根据scene_id读取车道线检测结果
     * @param scene_id 读取的frame_id
     * @param detectionDividerPerCapture  收到的frame数据包
     * @return
     */
    bool getDetectionDividerBySceneId(const string scene_id, DetectionDividerPerCapture &capture) {
        vector<string> dividerFileNames;//所有divider文件名
        vector<DetectionDividerPerFrame> detectionBatch_vec; //装好的scieId和Image集合
        bool flag = calulate::getAllFiles(detection_result_flie_folder + "/divider", dividerFileNames);
//        获取文件名
        string divider_file_name = detection_result_flie_folder + "/divider/" + scene_id + "divider.pb";
//        std::cout << "divider_size: " << divider_file_name.size() << endl;
        fstream input_divider(divider_file_name, ios::in | ios::binary);
        hdmap::DividerPerCapture dividerPerCapture_origin;
        dividerPerCapture_origin.ParseFromIstream(&input_divider);
//        遍历perCapure
//        std::cout << "divider_frames_size: " << dividerPerCapture_origin.divider_frames_size() << endl;


        if (flag) {
            vector<DetectionDividerPerFrame> dividerFrame_vec;
//           一帧divider_frames
            int index=0;
            bool flag=false;
            for (int i = 0; i < dividerFileNames.size(); ++i) {
                string file_name = dividerFileNames[i].substr(0, dividerFileNames[i].length()-10);
//                cout << "filenaem:" << file_name << endl;
                if (scene_id==file_name) {
//                    找到此帧divider,开始解析
                    index = i;
                    flag=true;
//                    std::cout << "成功找到scene_id: " <<  file_name<< endl;
//                    std::cout << "成功找到fileane: " <<  file_name<< endl;
//                    std::cout << "成功找到index: " <<  index<< endl;

                }
            }

            if (flag) {
//            解析此帧

//                解析frame
                DetectionDividerPerFrame dividerPerFrame;
                for (int j = 0; j < dividerPerCapture_origin.divider_frames_size(); ++j) {
                    hdmap::DividerPerFrame frame= dividerPerCapture_origin.divider_frames(j);
//                    解析divider
                    for (int i = 0; i < frame.dividers_size(); ++i) {
                        DividerEach dividerEach;
                        dividerEach.id = frame.dividers(i).id();
                        dividerEach.type = frame.dividers(i).type();
                        dividerEach.occlusion = frame.dividers(i).occlusion();
                        dividerEach.color = frame.dividers(i).color();

                        string geomery = frame.dividers(i).geometry();
//                        cout << "geomery" << geomery << endl;
                        vector<double> geom;
                        calulate::extractFiguresFromStr2Vec(geomery, geom);
                        for (int k = 0; k < geom.size()/2; k++) {
                            PointT point;
                            point.x = geom[2 * k];
                            point.y = geom[2 * k + 1];
                            dividerEach.points_vec.push_back(point);
                        }
//                        std::cout << "points_size: " << dividerEach.points_vec.size() << endl;
                        dividerPerFrame.dividerEach_vec.push_back(dividerEach);
                    }
                    capture.dividerPerFrame_vec.push_back(dividerPerFrame);
//                    std::cout << "divider_size: " << dividerPerFrame.dividerEach_vec.size() << endl;
                }
//                std::cout << "dividerPerFrame_vec_size: " << capture.dividerPerFrame_vec.size() << endl;
            } else{
                cout << "未找到此帧数据" << endl;
            }
        }
        return true;
    }

    /**
     * 根据scene_id获取traffic-detection
     * @param scene_id
     * @param detectionTrafficPerCapture
     * @return
     */
    bool getDetctionTrafficlights(const string scene_id, DetectionTrafficPerCapture &capture)
    {
//        获取文件夹列表并排序
        vector<string> trafficFileNames;//所有divider文件名
        bool flag = calulate::getAllFiles(detection_result_flie_folder + "/trafficlight", trafficFileNames);
//        获取文件名
        string traffic_file_name = detection_result_flie_folder + "/trafficlight/" + scene_id + "detect_trafficlight.pb";
//         从文件夹列表中匹配scene_id
//        std::cout << "trafficlight_size: " << traffic_file_name.size() << endl;
        hdmap::TrafficLightPerCapture trafficLightPerCapture_origin;
//       解析出来的PerCapture
        fstream input_traffic(traffic_file_name, ios::in | ios::binary);
        trafficLightPerCapture_origin.ParseFromIstream(&input_traffic);
//        遍历perCapure
//        std::cout << "traffic_frames_size: " << trafficLightPerCapture_origin.traffic_light_frames_size() << endl;
//20190123112752_7ac6ab9d61d94314188426910d324c39_4detect_trafficlight.pb
        if (flag) {
            vector<DetectionDividerPerFrame> trafficFrame_vec;
//           一帧traffic_frames
            bool flag2 = false;
            int index = 0;
            for (int i = 0; i < trafficFileNames.size(); ++i) {
                string file_name = trafficFileNames[i].substr(0, trafficFileNames[i].length()-22);
                if (scene_id==file_name) {
                    index = i;
                    flag2 = true;
//                    std::cout << "成功找到scene_id2: " <<  file_name<< endl;
//                    std::cout << "成功找到fileane2: " <<  file_name<< endl;
//                    std::cout << "成功找到index2: " <<  index<< endl;
                }
            }

            if (index>0 && flag2) {
                vector<DetectionDividerPerFrame> trafficPerFrame_vec;
                for (int j = 0; j < trafficLightPerCapture_origin.traffic_light_frames_size(); ++j) {
                    hdmap::TrafficLightPerFrame frame = trafficLightPerCapture_origin.traffic_light_frames(j);

                    DetectionTrafficPerFrame trafficPerFrame;
                    trafficPerFrame.frame_id = frame.frame_id();
                    for (int i = 0;i < frame.traffic_lights_size(); i++) {
                        TrafficLightEachShow trafficLightEach;
                        hdmap::TrafficLight trafficLight_origin = frame.traffic_lights(i);
                        trafficLightEach.id = trafficLight_origin.id();
                        trafficLightEach.motor_type = trafficLight_origin.motor_type();
//                       计算traffic geomery
                        string geomery = trafficLight_origin.geometry();
                        vector<double> geom;
                        vector<PointT> trafficLightEach_point_vec;
                        calulate::extractFiguresFromStr2Vec(geomery, geom);
                        for (int k = 0; k < geom.size()/2; ++k) {
                            PointT point;
                            point.x = geom[2 * k];
                            point.y = geom[2 * k + 1];
                            trafficLightEach_point_vec.push_back(point);
                        }
//                        std::cout << "trafficLight points_size: " << trafficLightEach_point_vec.size() << endl;
//                        cout << "------traffic light:---------" << geomery << endl;
//                      循环获取light_vec
                        vector<LightEachShow> light_vec;
                        for (int l = 0; l < trafficLight_origin.lights_size(); ++l) {
                            LightEachShow lightEach;
                            hdmap::Light light_origin = trafficLight_origin.lights(l);

                            lightEach.id = light_origin.id();
                            lightEach.direction = light_origin.direction();
                            string geomery2 = light_origin.geometry();
                            vector<double> geom2;
                            vector<PointT> light_point_vec;
                            calulate::extractFiguresFromStr2Vec(geomery2, geom2);
                            for (int k = 0; k < geom2.size(); ++k) {
                                PointT point2;
                                point2.x = geom[2 * k];
                                point2.y = geom[2 * k + 1];
                                light_point_vec.push_back(point2);
                            }
                            light_vec.push_back(lightEach);
                        }
                        trafficPerFrame.trafficLight_vec.push_back(trafficLightEach);
//                        std::cout << "light_vec_size: " << trafficPerFrame.trafficLight_vec.size() << endl;
                    }
                    capture.trafficPerFrame_vec.push_back(trafficPerFrame);
//                    std::cout << "trafficLight_vec_size: " << trafficPerFrame.trafficLight_vec.size() << endl;
                }
//                cout << "trafficPerFrame_vec_size: " << trafficFrame_vec.size() << endl;
                return true;
            } else{
                cout << "未找到此帧数据" << endl;
                return false;
            }
        }
    }

    /**
     * 取一个GPS点对应的图片的所有检测结果
     */
     bool getAllDetectionBatchByIndex( string scene_id, int index,DetchBatch &batch)
    {
//        遍历pb
        GpsImageBatch gpsImageBatch;
        bool flag = getGpsImageBatchByImageId(scene_id, index, gpsImageBatch);

//        根据index和scene_id  取trafficlight
        DetectionTrafficPerCapture detectionTrafficPerCapture;
        getDetctionTrafficlights(scene_id, detectionTrafficPerCapture);
        DetectionDividerPerCapture detectionDividerPerCapture;
        bool flag2 = getDetectionDividerBySceneId(scene_id, detectionDividerPerCapture);

        if (index < detectionTrafficPerCapture.trafficPerFrame_vec.size() && index >= 0 && index < detectionTrafficPerCapture.trafficPerFrame_vec.size())
        {
            DetectionTrafficPerFrame trafficPerFrame = detectionTrafficPerCapture.trafficPerFrame_vec[index];
            DetectionDividerPerFrame dividerPerFrame = detectionDividerPerCapture.dividerPerFrame_vec[index];
            batch.scene_id = scene_id;
            batch.dividerPerFrame.dividerEach_vec = dividerPerFrame.dividerEach_vec;
            batch.trafficPerFrame.trafficLight_vec = trafficPerFrame.trafficLight_vec;

            ImageBatch imageBatch;
            getImageBatchBySceneId(scene_id, imageBatch);
            batch.image_name = imageBatch.images_vec[index];

            GPSInfoEach gpsInfo;
            bool flag= getGPSInfoBySceneId(scene_id, gpsInfo);
            batch.point = gpsInfo.gpsPoints[index];
        }
    }

    bool getIndexByImageId(string image_id, int & index)
    {
//        20190123112752_7ac6ab9d61d94314188426910d324c39_4_021
        string scene_id = image_id.substr(0, image_id.length()-4);
        cout << "scene_id:" << scene_id << endl;
        ImageBatch imageBatch;
        bool flag = getImageBatchBySceneId(scene_id,imageBatch);
        vector<string> images_vec = imageBatch.images_vec;
        cout << "image size:" << images_vec.size() << endl;
        for (int i = 0; i < images_vec.size() ; i++) {
            //cout << "images_vec:" << images_vec[i] << " image_id:"<< image_id<<endl;

            if (images_vec[i]==image_id)
            {
                index = i;
            }
        }
//        return 0;
    }
}


#endif //PROTO_TEST_MAP_READ_H
