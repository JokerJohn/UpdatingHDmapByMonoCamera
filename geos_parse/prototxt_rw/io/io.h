//
// Created by lfg on 19-7-30.
//

#ifndef HDMAPPROJECT_READPROTO_H
#define HDMAPPROJECT_READPROTO_H

#include <iomanip>
#include <iostream>
#include <string>
#include <stdint.h>

#include <fcntl.h>
#include <unistd.h>

#include <google/protobuf/message.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>


namespace protoio{
    const int kProtoReadBytesLimit = INT_MAX;  // Max size of 2 GB minus 1 byte.

    template<class T>
    inline bool ReadProtoFromTextFile(const char *file_name, T *proto) {
        using google::protobuf::io::FileInputStream;
        using google::protobuf::io::FileOutputStream;
        using google::protobuf::io::ZeroCopyInputStream;
        using google::protobuf::io::CodedInputStream;
        using google::protobuf::io::ZeroCopyOutputStream;
        using google::protobuf::io::CodedOutputStream;
        using google::protobuf::Message;
        //ros::package::getPath("roborts")
        std::string full_path = std::string(file_name);
//        ROS_INFO("Load prototxt: %s", full_path.c_str());

        int fd = open(full_path.c_str(), O_RDONLY);
        if (fd == -1) {
//            ROS_ERROR("File not found: %s", full_path.c_str());
            return false;
        }
        FileInputStream *input = new FileInputStream(fd);
        bool success = google::protobuf::TextFormat::Parse(input, proto);
        delete input;
        close(fd);
        return success;
    }

    template<class T>
    inline bool ReadProtoFromTextFile(const std::string &file_name, T *proto) {
        return ReadProtoFromTextFile(file_name.c_str(), proto);
    }

    template<class T>
    inline bool ReadProtoFromBinaryFile(const char *file_name, T *proto) {
        using google::protobuf::io::FileInputStream;
        using google::protobuf::io::FileOutputStream;
        using google::protobuf::io::ZeroCopyInputStream;
        using google::protobuf::io::CodedInputStream;
        using google::protobuf::io::ZeroCopyOutputStream;
        using google::protobuf::io::CodedOutputStream;
        using google::protobuf::Message;

        int fd = open(file_name, O_RDONLY);
        if (fd == -1) {
            proto = NULL;
//            ROS_ERROR("File not found: %s", file_name);
        }

        ZeroCopyInputStream *raw_input = new FileInputStream(fd);
        CodedInputStream *coded_input = new CodedInputStream(raw_input);
        coded_input->SetTotalBytesLimit(kProtoReadBytesLimit, 536870912);

        bool success = proto->ParseFromCodedStream(coded_input);

        delete coded_input;
        delete raw_input;
        close(fd);
        return success;
    }
    template<class T>
    inline bool ReadProtoFromBinaryFile(const std::string &file_name, T *proto) {
        return ReadProtoFromBinaryFile(file_name.c_str(), proto);
    }

    template<class T>
    inline bool ReadYmlFromFile(const char *file_name, T *yml_type);
    template<class T>
    inline bool ReadYmlFromFile(const std::string &file_name, T *yml_type) {
        return ReadYmlFromFile(file_name.c_str(), yml_type);
    }



}
#endif //HDMAPPROJECT_READPROTO_H
