#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

// 单个数据点类
class DataPoint{

public:
    DataPoint(float x, float y, float z, unsigned long dpId):x(x),y(y),z(z),dpId(dpId){}
    ~DataPoint(){}

public:
    float x, y, z;
    int labelID = -1;                 //所属聚类ID

    unsigned long get_dpId(void)
    {
        return this->dpId;
    }

    bool get_isKey(void)
    {
        return this->isKey;
    }

    bool get_isVisited(void)
    {
        return this->isVisited;
    }

    void set_isKey(bool logic)
    {
        this->isKey = logic;
    }

    void set_isVisited(bool logic)
    {
        this->isVisited = logic;
    }

    vector<unsigned long> &get_arrivalPoints(void)
    {
        return arrivalPoints;
    }

private:
    unsigned long dpId;                     //点的ID
    bool isKey = false;                     //是否是核心点
    bool isVisited = false;                 //是否已访问
    vector<unsigned long> arrivalPoints;    //领域数据点id列表
};

// DBScan 聚类算法实现类
class DBScan{
public:
    DBScan(float minDis, int minPts)
    {
        this->minDis = minDis;
        this->minPoints = minPts;
    }
    ~DBScan(){}

    vector<DataPoint> dataset;
    int clusterId = 0;                        // 聚类id计数，初始化为0
    // 喂数据点
    void append_datapoint(float x, float y, float z)
    {
        DataPoint data(x, y, z, dataset.size());
        dataset.push_back(data);
    }

    // clear data point
    void clear_datapoint(void)
    {
        clusterId = 0;
        dataset.clear();
    }

    // 开始dbscan
    bool start_dbscan(void)
    {
        // 离散点为空
        if (dataset.empty())
            return false;

        // Step1: 设置领域点
        for(unsigned long i = 0; i < dataset.size(); i++)
        {
            SetArrivalPoints(dataset[i]);            // 计算数据点领域内对象
        }

        // Step2: 开始逐点聚类
        clusterId = 0;
        for(unsigned long i = 0; i < dataset.size();i++)
        {
            DataPoint& dp = dataset[i];
            if(!dp.get_isVisited() && dp.get_isKey())       // 若对象没被访问过，并且是核心对象执行
            {
                dp.labelID = clusterId;                     // 设置该对象所属簇ID为clusterId
                dp.set_isVisited(true);               // 设置该对象已被访问过
                KeyPointCluster(i,clusterId);               // 对该对象领域内点进行聚类
                clusterId++;                                // 下一类
            }
        }
        cout << "聚类数量 = " << clusterId << endl;        //算法完成后，输出聚类个数

        return true;
    }

private:
    float minDis = 0;
    int minPoints = 0;

private:
    // 欧式距离
    double GetDistance(DataPoint &a, DataPoint &b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    }

    // 设置领域点
    void SetArrivalPoints(DataPoint& dp)
    {
        for(unsigned long i = 0; i < dataset.size(); i++)
        {
            if (i == dp.get_dpId())                                 // 自身距离不用计算
                continue;

            double distance = GetDistance(dataset[i], dp);   // 获取与特定点之间的距离
            if(distance <= minDis)                                  // 若距离小于半径,则为邻域
                dp.get_arrivalPoints().push_back(i);
        }
        if(dp.get_arrivalPoints().size() >= minPoints)              // 若dp领域内数据点数据量 >= minPTs，则为核心点
        {
            dp.set_isKey(true);
            return;
        }
        dp.set_isKey(false);
    }

    // 关键点聚类(深度优先搜索)
    void KeyPointCluster(unsigned long dpID, unsigned long clusterId)
    {
        DataPoint& srcDp = dataset[dpID];        // 获取数据点对象
        if(!srcDp.get_isKey())      // 递归终止条件，不是核心点就停止
            return;

        vector<unsigned long>& arrPoints = srcDp.get_arrivalPoints();   // 获取对象领域内点ID列表
        for(unsigned long i = 0; i < arrPoints.size(); i++)
        {
            DataPoint& desDp = dataset[arrPoints[i]];   //获取领域内点数据点
            if(!desDp.get_isVisited())                  //若该对象没有被访问过执行
            {
                desDp.labelID = clusterId;              //设置该对象所属簇的ID为clusterId，即将该对象吸入簇中
                desDp.set_isVisited(true);        //设置该对象已被访问
                if(desDp.get_isKey())                   //若该对象是核心对象，递归搜索
                {
                    KeyPointCluster(desDp.get_dpId(), clusterId);    //递归地对该领域点数据的领域内的点执行聚类操作，采用深度优先方法
                }
            }
        }
    }

};

#endif // DBSCAN_H
