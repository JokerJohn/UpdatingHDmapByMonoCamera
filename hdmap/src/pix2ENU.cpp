#include <iostream>
#include "utils/Utils.h"
#include "utils/read_hdmap.h"
#include "utils/pix2enu.h"

int main() {

    //read scene
//    string scene_id = "20190130161123_c6a0dc163825d772bed42152c9e9b9f0_4";
//    string scene_id = "20190123112838_3faf30bde99e0f126cda2432ec90a621_4";
    string scene_id = "20190130161123_c6a0dc163825d772bed42152c9e9b9f0_4";


/*
    std::ofstream f;
    f.open("../GPStrafficLine.txt");
    std::cout<< "gps_points.size:" <<gps_points.size()<< std::endl;
*/

    vector<DividerEach> divider_vec;
    pix2enu2(divider_vec, scene_id, 4);
    for (int i = 0; i < divider_vec.size() ; ++i) {
        vector<PointT> points_vec = divider_vec[i].points_vec;
        cout << " divider points size:" << divider_vec[i].points_vec.size() << endl;
    }
}