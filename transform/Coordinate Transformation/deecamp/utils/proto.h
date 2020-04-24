//
// Created by lfg on 19-7-30.
//

#ifndef IO2_H
#define IO2_H

#include <iomanip>
#include <iostream>  // NOLINT(readability/streams)
#include <string>

#include "google/protobuf/message.h"
#include <fcntl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <stdint.h>

#include <algorithm>
#include <fstream>  // NOLINT(readability/streams)
#include <vector>
#include <limits>
#ifdef WIN32
#define open _open
#else
#include <unistd.h>
#define O_BINARY 0x00
#endif

const int kProtoReadBytesLimit = std::numeric_limits<int>::max();  // Max size of 2 GB minus 1 byte.

using namespace std;
namespace protoio{
    using google::protobuf::io::FileInputStream;
    using google::protobuf::io::FileOutputStream;
    using google::protobuf::io::ZeroCopyInputStream;
    using google::protobuf::io::CodedInputStream;
    using google::protobuf::Message;

    bool ReadProtoFromTextFile(const char* filename, Message* proto) {
        int fd = open(filename, O_RDONLY);
//        CHECK_NE(fd, -1) << "File not found: " << filename;
        FileInputStream* input = new FileInputStream(fd);
        bool success = google::protobuf::TextFormat::Parse(input, proto);
        delete input;
        close(fd);
        return success;
    }
    inline bool ReadProtoFromTextFile(const string& filename, Message* proto) {
        return ReadProtoFromTextFile(filename.c_str(), proto);
    }

    inline void ReadProtoFromTextFileOrDie(const char* filename, Message* proto) {
//        CHECK(ReadProtoFromTextFile(filename, proto));
    }

    inline void ReadProtoFromTextFileOrDie(const string& filename, Message* proto) {
        ReadProtoFromTextFileOrDie(filename.c_str(), proto);
    }

    void WriteProtoToTextFile(const Message& proto, const char* filename) {
        int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        FileOutputStream* output = new FileOutputStream(fd);
        google::protobuf::TextFormat::Print(proto, output);
        delete output;
        close(fd);
    }
    inline void WriteProtoToTextFile(const Message& proto, const string& filename) {
        WriteProtoToTextFile(proto, filename.c_str());
    }


    bool ReadProtoFromBinaryFile(const char* filename, Message* proto) {
        int fd = open(filename, O_RDONLY | O_BINARY);
//        CHECK_NE(fd, -1) << "File not found: " << filename;
        ZeroCopyInputStream* raw_input = new FileInputStream(fd);
        CodedInputStream* coded_input = new CodedInputStream(raw_input);
        coded_input->SetTotalBytesLimit(kProtoReadBytesLimit, 536870912);

        bool success = proto->ParseFromCodedStream(coded_input);

        delete coded_input;
        delete raw_input;
        close(fd);
        return success;
    }
    inline bool ReadProtoFromBinaryFile(const string& filename, Message* proto) {
        return ReadProtoFromBinaryFile(filename.c_str(), proto);
    }

    inline void ReadProtoFromBinaryFileOrDie(const char* filename, Message* proto) {
//        CHECK(ReadProtoFromBinaryFile(filename, proto));
    }

    inline void ReadProtoFromBinaryFileOrDie(const string& filename,
                                             Message* proto) {
        ReadProtoFromBinaryFileOrDie(filename.c_str(), proto);
    }


    void WriteProtoToBinaryFile(const Message& proto, const char* filename) {
        std::fstream output(filename, std::ios::out | std::ios::trunc | std::ios::binary);
//        CHECK(proto.SerializeToOstream(&output));
    }
    inline void WriteProtoToBinaryFile(
            const Message& proto, const string& filename) {
        WriteProtoToBinaryFile(proto, filename.c_str());
    }

}

#endif //IO2_H
