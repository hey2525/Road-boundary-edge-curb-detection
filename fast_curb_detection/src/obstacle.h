//
// Created by gotok on 18-12-27.
//

#ifndef FAST_CURB_DETECTION_OBSTACLE_H
#define FAST_CURB_DETECTION_OBSTACLE_H

#include <array>
#include <pcl/io/pcd_io.h>
#include "../../velodyne-master/velodyne_pointcloud/include/velodyne_pointcloud/point_types.h"

using namespace std;
using namespace pcl;
#define VPoint velodyne_pointcloud::PointXYZIR

//const int numGrid = 100;
const int numGrid = 200;//100 150 200
extern float roiM;
extern int kernelSize;

typedef struct {
    double Xmax;
    double Xmin;
    double Ymax;
    double Ymin;
    double lowest_z;
}bounding_box;

void componentClustering(PointCloud<VPoint> & elevatedCloud,
                         array<array<int, numGrid>, numGrid> & Map_Data,
                         int & numCluster);


void makeClusteredCloud(PointCloud<VPoint> & elevatedCloud,
                        array<array<int, numGrid>, numGrid> Map_Data,
                        PointCloud<pcl::PointXYZRGB>::Ptr& clusterCloud);

void getClusteredCloud(PointCloud<VPoint> & elevatedCloud,
                        array<array<int, numGrid>, numGrid> Map_Data,
                        vector<PointCloud<VPoint>>&  clusteredPoints);

void getBoundingBox(vector<bounding_box>& good_box,
                    vector<PointCloud<VPoint>>& good_obstacle);

void find_curb_obs(std::vector<pcl::PointCloud<VPoint>::Ptr>& curb,
                   vector<bounding_box>& good_box,
                    int flag,
                    double area_xmax,double area_xmin,double area_ymax,double area_ymin);

class obstacle {};


#endif //FAST_CURB_DETECTION_OBSTACLE_H
