//
// Created by catalina on 2019/7/30.
//
#ifndef PROTO_TEST_MAP_READ_H
#define PROTO_TEST_MAP_READ_H
#include "utils/proto.h"
#include "utils/calulate.h"
#include "proto/HDMap.pb.h"
#include "proto/LaneMarking.pb.h"
#include "proto/SourceInfo.pb.h"
#include "proto/TrafficLight.pb.h"
#include "tiff.h"
#include <vector>

//点位置
typedef struct PointT{
    int id;
    double x;
    double y;
    double z;
};

//HDMap
typedef struct HDMAP{
    //场景id，视频文件名
    string  scene_id;
//车道线
    vector<struct DividerEach> dividers;
//交通灯
   vector<struct TrafficLightEach> tafficlights;
//地面标识
    vector<struct LaneMarkingEach> lanemarkings;
    string  version;
};
//车道线
typedef struct DividerEach{
    //编号
    int32  id;
//位置WKT
    vector <struct PointT>  divider_vec;
//    string geometry;
//颜色，0：未知；1：白色；2：黄色
    int32  color;
//类型 0:未知； 1：单虚线；2：单实线；3：双实线；4：左实右虚；5：左虚右实,；6:不可通行减速线；7：可通行减速线
    int32  type;
//遮挡程度，1：无；2：部分遮挡；3：全部遮挡(invalid)
    int32  occlusion;
};

typedef struct TrafficLightEach{
//编号
    int32  id;
//位置WKT
    struct PointT point;
//类型，1：车辆；2：行人

    int32  motor_type;
//灯
   vector<struct LightEach> lights;
};

typedef struct LightEach{
//编号
    int32  id;
//位置WKT
//    string  geometry;
//类型，1：竖排；2：横排
    struct PointT points;
    int32  direction;
};
//地面标识
typedef struct LaneMarkingEach{
//编号
    int32 id;
//位置 WKT
//    vector<struct PointT> points;
    struct PointT points[5];
//类型，1：人行道；2：停止线；3：地面标识
    int32 type;
};

// GPS
typedef struct GPSInfoEach{
    string scene_id;
//设备名称
    string device_id;
//GPS点集合
    vector<struct GPSPointEach> gpsPoints;
};

//GPS point
typedef struct GPSPointEach{
    //编号
    int32 id;
//时间
    int64 gpstime;
//航向
    double heading;
//速度
    double speed;
//坐标 WKT
    struct PointT points;
};

struct ImageBatch{
//    GPS帧id
    string scene_id;
//    对应image帧集合
    vector<string> images_vec;

};
/*class ReadHDMap {
protected:

public:
//    GPSInfoEach getGPSInfoBySceneId(string scene_id);
//    HDMAP getLandMarkingInHDMAP();
};*/
namespace ReadHDMap{
    const string  gps_file_folder = "../data/gps";
    const string  images_file_folder = "../data/images";
//    GPSInfoEach getGPSInfoBySceneId(string scene_id);
//    HDMAP getLandMarkingInHDMAP();
    HDMAP getHDMAP() {
        std::string file_name = "../data/hdmap/hdmap_deecamp.pb";

        fstream input(file_name, ios::in | ios::binary);
        hdmap::HDMap map;
        map.ParseFromIstream(&input);

        HDMAP readMap; //高精地图所有数据
        readMap.version = map.version();
        readMap.scene_id = map.scene_id();

        LaneMarkingEach laneMarkingEach; //高精地图中的所有路标
        TrafficLightEach trafficLightEach;//高精地图中的所有交通灯
        DividerEach dividerEach;//高精地图中的所有车道线

        cout<<"lanemarkings_size size:"<<map.lanemarkings_size() << endl;
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
                    laneMarkingEach.points[j].x = geom[3 * j];
                    laneMarkingEach.points[j].y = geom[3 * j + 1];
                    laneMarkingEach.points[j].z = geom[3 * j + 2];
//                cout << std::setprecision(14)<<  laneMarkingEach.points[j].x << " " << laneMarkingEach.points[j].y
//                     << " " << laneMarkingEach.points[j].z << endl;
                }
                readMap.lanemarkings.push_back(laneMarkingEach);
            } else
            {
                cout <<  "laneMarking 位置读取有误 "<< endl;
            }
        }

        cout<<"tafficlights_size size:"<<map.tafficlights_size() << endl;
//    读取所有的交通灯
        for (int k = 0; k < map.tafficlights_size(); ++k) {
            hdmap::TrafficLight trafficLight = map.tafficlights(k);
            trafficLightEach.id = trafficLight.id();
            trafficLightEach.motor_type = trafficLight.motor_type();
            string geometry = trafficLight.geometry();
            cout << "traffic light:" << geometry <<endl;
            vector<double> geom;
            calulate::extractFiguresFromStr2Vec(geometry, geom);

            if(geom.size() == 3)
            {
                trafficLightEach.point.x = geom[0];
                trafficLightEach.point.y = geom[1];
                trafficLightEach.point.z = geom[2];

//            cout << std::setprecision(14)<<  trafficLightEach.point.x << " " << trafficLightEach.point.y
//                 << " " << trafficLightEach.point.z << endl;
//            Light 里面的数据继续查询

                LightEach lightEach; //第二次进来，lightEach归0
                for (int i = 0; i < trafficLight.lights_size(); ++i) {
                    lightEach.id = trafficLight.lights(i).id();
                    lightEach.direction = trafficLight.lights(i).direction();

                    vector<LightEach> lights_vector;
                    vector<double > geom2;
                    calulate::extractFiguresFromStr2Vec(trafficLight.lights(i).geometry(), geom2);
                    if (geom2.size()==3)
                    {
                        PointT point;
                        point.x = geom2[0];
                        point.y = geom2[1];
                        point.z = geom2[2];

                        lightEach.points.x = geom2[0];
                        lightEach.points.y = geom2[1];
                        lightEach.points.z = geom2[2];
                        lights_vector.push_back(lightEach);
                    }
                    trafficLightEach.lights = lights_vector;
                }
                trafficLightEach.lights.push_back(lightEach);
            } else{
                cout <<  "trafficLights 位置读取有误 "<< endl;
            }
            readMap.tafficlights.push_back(trafficLightEach);
        }

        cout<<"dividers_size size:"<<map.dividers_size() << endl;
        for (int l = 0; l < map.dividers_size(); ++l) {
            hdmap::Divider divider = map.dividers(l);
            dividerEach.id = divider.id();
            dividerEach.type = divider.type();
            dividerEach.color = divider.color();
            dividerEach.occlusion = divider.occlusion();

            vector<double > geom;
            calulate::extractFiguresFromStr2Vec(divider.geometry(), geom);
//        cout << "geom----"<<geom.size()/3 <<" ----" << geom[0] << " " << geom[1] << " " << geom[2] << endl;
            int pointsNum = geom.size()/3;
//        cout << "----id----"<< dividerEach.id <<"-----" <<pointsNum << "------"<<endl;
            vector<PointT> vetor;
            for (int i = 0; i < geom.size(); ++i) {
                PointT point;
                point.x = geom[i*3];
                point.y = geom[i*3+1];
                point.z = geom[i*3+2];
                if (i%3==0)
                {
                    vetor.push_back(point);
                }
//            cout <<std::setprecision(14) << geom[i*3] <<" " << geom[i*3+1] << " " << geom[i*3+2] << endl;
            }
            dividerEach.divider_vec = vetor;

//        cout << "----id----"<< dividerEach.id << endl;
            readMap.dividers.push_back(dividerEach);
        }
        return readMap;
    }

    GPSInfoEach getGPSInfoBySceneId(string scene_id) {
        std::cout << "scene_id: " << scene_id << endl;
        string file_name = "../data/gps/"+scene_id+"deecamp_gps.pb";
        std::cout << "file_name: " << file_name << endl;

        fstream input_gps(file_name, ios::in | ios::binary);
        source::GPSInfo gpsInfo;

        if (gpsInfo.scene_id().compare(scene_id))
        {
            gpsInfo.ParseFromIstream(&input_gps);
            source::GPSPoint gpsPoint;
            gpsPoint.ParseFromIstream(&input_gps);

//        一帧GPS数据信息
            GPSInfoEach gpsInfoEach;
            gpsInfoEach.scene_id = gpsInfo.scene_id();

            for (int l = 0; l < gpsInfo.pts_size(); ++l) {
                source::GPSPoint pts = gpsInfo.pts(l);
                GPSPointEach gpsPointEach;
                gpsPointEach.heading = pts.heading();
                gpsPointEach.id = pts.id();
                gpsPointEach.gpstime = pts.gpstime();
                gpsPointEach.speed = pts.speed();
//        处理geometry
                vector<double> geom;
                calulate::extractFiguresFromStr2Vec(pts.geometry(),geom);
                gpsPointEach.points.x = geom[0];
                gpsPointEach.points.y = geom[1];
                gpsPointEach.points.z = geom[2];
                gpsInfoEach.gpsPoints.push_back(gpsPointEach);

//            std::cout << std::setprecision(14) <<geom[0] << " " << geom[1] << " " <<geom[2]<<endl;
//            std::cout << "pts:" << pts.geometry().substr(9, 34)<< endl;
                //        gpsInfoEach.gpsPoints.push_back(gpsPointEach);
            }
            return gpsInfoEach;
        } else{
            std::cout << "不存在此帧数据: " << endl;
        }


    }

    vector<ImageBatch> getAllImageBatch(){
//        string  gps_file_folder = "../data/gps";
//        string  images_file_folder = "../data/images";

        vector<string> gpsFileNames; //GPS文件名
        vector<string> imagesFileNames;//所有image文件名
        vector<ImageBatch> dataFiles; //装好的scieId和Image集合
        //    1.读gps, 2读image
        bool flag = calulate::getAllFiles(gps_file_folder, gpsFileNames);
        bool flag2 = calulate::getAllFiles(images_file_folder, imagesFileNames);

        if (flag==0 && flag2 ==0)
        {
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
                string scene_id = gpsFileNames[i].substr(0, nops-11);
//                cout << "scene_id:" << scene_id<< endl;
                imageBatch.scene_id = scene_id;

//        根据scene_id去查询images id，返回images vetor
                vector<string> image_vec;
                for (int j = 0; j < imagesFileNames.size(); ++j) {
                    int nops_image = imagesFileNames[j].find_first_of(".");
                    string image_id = imagesFileNames[j].substr(0, nops_image);
//                cout << "image_id:" << image_id << endl;

                    string::size_type idx;
                    idx=image_id.find(scene_id);//在a中查找b.
                    if(idx != string::npos ){
                        //存在, 装进去IMages
                        image_vec.push_back(image_id);
                    }
                }
//                cout << "-----image_vec:------" << image_vec.size()<<endl;
                imageBatch.images_vec = image_vec;
                dataFiles.push_back(imageBatch);
            }

//            cout << "dfgfb:"<< dataFiles.size()<<" dfgfd:"<< dataFiles[100].images_vec.size()<< endl;
            return dataFiles;
        } else{
            cout << "fileName为空"<<   endl;
        }

    }

    ImageBatch getImageBatchBySceneId(string scene_id2)
    {
        vector<string> gpsFileNames; //GPS文件名
        vector<string> imagesFileNames;//所有image文件名
        vector<ImageBatch> dataFes; //装好的scieId和Image集合
        //    1.读gps, 2读image
        bool flag = calulate::getAllFiles(gps_file_folder, gpsFileNames);
        bool flag2 = calulate::getAllFiles(images_file_folder, imagesFileNames);

        if (flag==0 && flag2 ==0)
        {
            //// 读取之后对vector内的image和gps进行排序
            calulate::sortedVector(gpsFileNames);
            calulate::sortedVector(imagesFileNames);
            ImageBatch imageBatch;

            for (int i = 0; i < gpsFileNames.size(); ++i) {
                int nops = gpsFileNames[i].find_first_of(".");
                string scene_id = gpsFileNames[i].substr(0, nops-11);

                if (scene_id.compare(scene_id2))
                {
//                    cout << "查找到scene_id维 " << scene_id<< endl;
                    imageBatch.scene_id = scene_id2;

                    vector<string> image_vec;
                    for (int j = 0; j < imagesFileNames.size(); ++j) {
                        int nops_image = imagesFileNames[j].find_first_of(".");
                        string image_id = imagesFileNames[j].substr(0, nops_image);

                        string::size_type idx;
                        idx=image_id.find(scene_id);//在a中查找b.
                        if(idx != string::npos ){
                            //存在, 装进去IMages
                            image_vec.push_back(image_id);
                        }
                    }
//                    cout << "-----image_vec:------" << image_vec.size()<<endl;
                    imageBatch.images_vec = image_vec;
                }
            }
//            cout <<"imagbatch "<< imageBatch.scene_id << " image size "<< imageBatch.images_vec.size()<<endl;
            return imageBatch;

        } else{
            cout << "fileName为空"<<   endl;
        }

    }
}


#endif //PROTO_TEST_MAP_READ_H
