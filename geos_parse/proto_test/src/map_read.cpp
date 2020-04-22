//
// Created by catalina on 2019/7/30.
//

#include "map_read.h"
#include <iomanip>
int main(){

//    cout<<"GEOS库版本为："<<GEOS_VERSION<<endl;

    std::string file_name = "../config/hdmap_deecamp.pb";
    std::string file_name2 = "../config/hdmap.txt";

    fstream input(file_name, ios::in | ios::binary);
    hdmap::HDMap map;
    map.ParseFromIstream(&input);
    int taff_size = map.tafficlights().size();
    int lane_size = map.dividers_size();

//
    vector<hdmap::TrafficLight> bigLight_vetor(map.tafficlights_size());
    vector<hdmap::Divider> divider_vetor(map.dividers_size());
    vector<hdmap::LaneMarking> laneMarking_vector(map.lanemarkings_size());
    vector<hdmap::Light> light;
//    地面标识
    for (int i = 0; i < map.lanemarkings_size(); ++i) {
        hdmap::LaneMarking laneMarking = map.lanemarkings(i);
        std::string geometry = laneMarking.geometry();
//        int last = geometry.find_first_of(")");
//        int first = geometry.find_last_of("(");
//        std::cout << "first:" <<last-2<<endl;

        string::size_type nPos1 = string::npos;
        string::size_type nPos2 = string::npos;
        nPos1 = geometry.find_last_of("((");
        nPos2 =geometry.find_last_of("))", nPos1-1);

        cout << "Npos1: "<< nPos1 << " nPos2:" << nPos2;
        if (nPos1!=-1 && nPos2!=-1)
        {
            geometry = geometry.substr(nPos2+1, nPos1-nPos2-1);
        }
        std::cout <<geometry << endl;
        laneMarking_vector.push_back(laneMarking);
    }
//    红绿灯
    for (int j = 0; j < map.tafficlights_size() ; ++j) {
        hdmap::TrafficLight bigLight = map.tafficlights(j);
        std::cout << "bigLight:" << bigLight.geometry() << endl;
        bigLight_vetor.push_back(bigLight);
        for (int i = 0; i < bigLight.lights_size(); ++i) {
            hdmap::Light light = bigLight.lights(i);
            std::cout << "light:" << light.geometry() << endl;
        }
    }

//    车道线
    for (int k = 0; k < map.dividers_size(); ++k) {
        hdmap::Divider divider = map.dividers(k);
        divider_vetor.push_back(divider);
        std::cout << "divider:" << divider.geometry() << endl;
    }


//    cout << map.scene_id() << endl;
    vector<double> vec;
    hdmap::Divider divider = map.dividers(0);
    extractFiguresFromStr2Vec(divider.geometry(),vec);
    std::cout << "divider_sssss:" << divider.geometry() << endl;
    std::cout <<std::setprecision(12)<< "divider:" << vec[0] << endl;

}