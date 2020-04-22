//
// Created by catalina on 2019/7/30.
//

#ifndef PROTO_TEST_MAP_READ_H
#define PROTO_TEST_MAP_READ_H
#include "io/io2.h"
#include "proto/HDMap.pb.h"
#include "proto/LaneMarking.pb.h"
#include "proto/SourceInfo.pb.h"
#include "proto/TrafficLight.pb.h"
//#include "geos.h"

struct PointT{
    int id;
    double x;
    double y;
    double z;
};
struct PointT PointsP[5];
struct PointT PointsL[3];
 struct POLYGON
 {
     int id;
     struct PointP;
 };
 struct LINESTRING{
     int id;
     struct PointsL;
 };

class map_read {
protected:

public:

};

void extractFiguresFromStr2Vec(string str, vector<double> &vec){
 const char *s = str.c_str();
 const char *pstr;
 int i = 0, j = 0;
 int k, m;
 int e10;
 int digit;
 int ndigit = 0;
 pstr = &s[0];

 for (i = 0; *(pstr + i) != '\0'; i++){
  if ((*(pstr + i) >= '0') && (*(pstr + i) <= '9') || *(pstr + i)=='.')
   j++;
  else{
   if (j > 0){
    string str;
    for (k = j; k > 1; k--){
     str.append(pstr + i  - k);
    }
    vec.push_back(atof(str.c_str()));
    ndigit++;
    j = 0;
   }
  }
 }
 if (j > 0){
  string str;
  for (k = j; k > 1; k--){
   str.append(pstr + i - k);
  }
  vec.push_back(atof(str.c_str()));
  ndigit++;
  j = 0;
 }
}
void extractFiguresFromStr2Vec(string str, vector<int> &vec){
 const char *s = str.c_str();
 const char *pstr;
 int i = 0, j = 0;
 int k, m;
 int e10;
 int digit;
 int ndigit = 0;
 pstr = &s[0];

 for (i = 0; *(pstr + i) != '\0'; i++){
  if ((*(pstr + i) >= '0') && (*(pstr + i) <= '9'))
   j++;
  else{
   if (j > 0){
    digit = *(pstr + i - 1) - 48;
    for (k = 1; k < j; k++){
     e10 = 1;
     for (m = 1; m <= k; m++)
      e10 = e10 * 10;
     digit = digit + (*(pstr + i - 1 - k) - 48)*e10;
    }
    vec.push_back(digit);
    ndigit++;
    j = 0;
   }
  }
 }
 if (j > 0){
  digit = *(pstr + i - 1) - 48;
  for (k = 1; k < j; k++){
   e10 = 1;
   for (m = 1; m <= k; m++)
    e10 = e10 * 10;
   digit = digit + (*(pstr + i - 1 - k) - 48)*e10;
  }
  vec.push_back(digit);
  ndigit++;
  j = 0;
 }
}

#endif //PROTO_TEST_MAP_READ_H
