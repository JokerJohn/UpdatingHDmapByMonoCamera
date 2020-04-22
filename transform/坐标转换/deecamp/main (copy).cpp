#include <iostream>
#include "Utils.h"
#include <math.h>
#include <iomanip>


int main() {
    Utils::new3s_PointXYZ original;
    Utils::new3s_PointXYZ object1, object2;
    // lon是经度 lat是纬度
    original.set_x(22.68085797);
    original.set_y(114.36477538);
    original.set_z(1.32000000);

    object1.set_x(22.68086392);
    object1.set_y(114.36479710);
    object1.set_z(1.32000000);

    object2.set_x(22.68086588);
    object2.set_y(114.36480534);
    object2.set_z(1.32000000);
    //new3s_PointXYZ lon_lat_coord;
    //Utils::new3s_PointXYZ enu_coord_1;
    Utils::new3s_PointXYZ enu_coord_1, enu_coord_2;
    Utils::new3s_PointXYZ object1_WGS, object2_WGS, original_WGS;
    Utils transform;
    Utils::new3s_PointXYZ original_xyz, object1_xyz, object2_xyz;
    transform.convertGCJO2ToLonlat(original, original_WGS);
    //std::cout<<std::setprecision(11)<<original_WGS.get_x()<<" "<<original_WGS.get_y()<<" "<<original_WGS.get_z()<<std::endl;
    //transform.convertLLHToXYZ(object, xyz);
    //std::cout<<std::setprecision(11)<<xyz.get_x()<<" "<<xyz.get_y()<<" "<<xyz.get_z()<<std::endl;
    transform.convertGCJO2ToLonlat(object1, object1_WGS);
    //std::cout<<std::setprecision(11)<<object1_WGS.get_x()<<" "<<object1_WGS.get_y()<<" "<<object1_WGS.get_z()<<std::endl;
    transform.convertGCJO2ToLonlat(object2, object2_WGS);
    //std::cout<<std::setprecision(11)<<object1_WGS.get_x()<<" "<<object1_WGS.get_y()<<" "<<object1_WGS.get_z()<<std::endl;
    transform.convertLLHToXYZ(original_WGS, original_xyz);
    //std::cout<<std::setprecision(11)<<original_xyz.get_x()<<" "<<original_xyz.get_y()<<" "<<original_xyz.get_z()<<std::endl;
    transform.convertLLHToXYZ(object1_WGS, object1_xyz);
    transform.convertLLHToXYZ(object2_WGS, object2_xyz);
    transform.convertXYZToENU(object1_xyz, original_xyz, enu_coord_1);
    transform.convertXYZToENU(object2_xyz, original_xyz, enu_coord_2);


    std::cout<<std::setprecision(11)<<sqrt((enu_coord_1.get_x()-enu_coord_2.get_x())*(enu_coord_1.get_x()-enu_coord_2.get_x())+(enu_coord_1.get_y()-enu_coord_2.get_y())*(enu_coord_1.get_y()-enu_coord_2.get_y()));
    std::cout<<std::setprecision(11)<<enu_coord_1.get_x()<<" "<<enu_coord_1.get_y()<<" "<<enu_coord_1.get_z()<<std::endl;
    //std::cout<<std::setprecision(11)<<enu_coord_2.get_x()<<" "<<enu_coord_2.get_y()<<" "<<enu_coord_2.get_z();
    return 0;
}