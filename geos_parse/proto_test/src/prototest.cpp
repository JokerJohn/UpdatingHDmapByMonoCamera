////
//// Created by lfg on 19-7-30.
////
//#include "prototest.h"
//
//int main(){
//    test::ConstraintSetConfig constraint_set_config_;
//    std::string file_name = "../config/test.prototxt";
//    std::string new_file_name = "../config/newwww.prototxt";
//
//    bool read_state = protoio::ReadProtoFromTextFile(file_name, &constraint_set_config_);
////    get date
//    test::ArmorSize  aromor = constraint_set_config_.armor_size();
//    auto num = constraint_set_config_.num();
//    bool using_hsv_ = constraint_set_config_.using_hsv();
//    std::cout << "aromor " << aromor.width() << std::endl;
//    std::cout << "num " << num << std::endl;
//    std::cout << "aromor " << using_hsv_ << std::endl;
//
////    set data
//    constraint_set_config_.set_using_hsv(false);
//    constraint_set_config_.set_num(324.2324);
//    aromor.set_width(1.2);
//    aromor.set_height(3.21);
//    constraint_set_config_.set_allocated_armor_size(&aromor);
//
//    test::ConstraintSetConfig constraint_set_config_2;
//    test::ArmorSize  aromor2 = constraint_set_config_2.armor_size();
//    constraint_set_config_2.set_using_hsv(false);
//    constraint_set_config_2.set_num(324.2324);
//    aromor2.set_width(1.2);
//    aromor2.set_height(3.21);
//    constraint_set_config_2.set_allocated_armor_size(&aromor2);
//
//    protoio::WriteProtoToTextFile(constraint_set_config_2, new_file_name);
//    protoio::WriteProtoToTextFile(constraint_set_config_, new_file_name);
//
//    float ok = constraint_set_config_.num();
//    std::cout<< using_hsv_<<std::endl;
//    std::cout<< ok<<std::endl;
//}
