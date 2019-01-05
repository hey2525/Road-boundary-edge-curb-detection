//
// Created by gotok on 18-12-27.
//

#include "obstacle.h"
#include <array>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

///int numGrid = 200; 100*100
float roiM = 50;// 50*50
int kernelSize = 3;

void mapCartesianGrid(PointCloud<VPoint> & elevatedCloud,
                      array<array<int, numGrid>, numGrid> & Map_Data){
#pragma omp parallel for
    {
        for(int i = 0; i < elevatedCloud.size(); i++)
        {
            float x = elevatedCloud.points[i].x;
            float y = elevatedCloud.points[i].y;
            float xC = x+roiM/2;
            float yC = y+roiM/2;

            if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue; //exclude x`y >25 points
            int xI = (int)floor(numGrid*xC/roiM);
            int yI = (int)floor(numGrid*yC/roiM);
            Map_Data[xI][yI] = -1;
        }
    }
}

void search(array<array<int, numGrid>, numGrid> & Map_Data, int clusterId, int cellX, int cellY){
    Map_Data[cellX][cellY] = clusterId;
    int mean = kernelSize/2;
    for (int kX = 0; kX < kernelSize; kX++){
        int kXI = kX-mean;
        if((cellX + kXI) < 0 || (cellX + kXI) >= numGrid) continue;
        for( int kY = 0; kY < kernelSize;kY++){
            int kYI = kY-mean;
            if((cellY + kYI) < 0 || (cellY + kYI) >= numGrid) continue;

            if(Map_Data[cellX + kXI][cellY + kYI] == -1){
                search(Map_Data, clusterId, cellX +kXI, cellY + kYI);
            }

        }
    }
}

void findComponent(array<array<int, numGrid>, numGrid> & Map_Data, int &clusterId){
    for(int cellX = 0; cellX < numGrid; cellX++){
        for(int cellY = 0; cellY < numGrid; cellY++){
            if(Map_Data[cellX][cellY] == -1){
                clusterId ++;
                search(Map_Data, clusterId, cellX, cellY);
            }
        }
    }
}

void componentClustering(PointCloud<VPoint> & elevatedCloud,
                         array<array<int, numGrid>, numGrid> & Map_Data,
                         int & numCluster){
    mapCartesianGrid(elevatedCloud, Map_Data);
    findComponent(Map_Data, numCluster);
}


void makeClusteredCloud(PointCloud<VPoint> & elevatedCloud,
                        array<array<int, numGrid>, numGrid> Map_Data,
                        PointCloud<pcl::PointXYZRGB>::Ptr& clusterCloud){
    for(int i = 0; i < elevatedCloud.size(); i++){
        float x = elevatedCloud.points[i].x;
        float y = elevatedCloud.points[i].y;
        float z = elevatedCloud.points[i].z;
        float xC = x+roiM/2;
        float yC = y+roiM/2;

        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue;
        int xI = (int)floor(numGrid*xC/roiM);
        int yI = (int)floor(numGrid*yC/roiM);

        int clusterNum = Map_Data[xI][yI];
        if(clusterNum != 0){
            PointXYZRGB o;
            o.x = x;
            o.y = y;
            o.z = z;
            o.r = static_cast<uint8_t>((500 * clusterNum) % 255);
            o.g = static_cast<uint8_t>((100 * clusterNum) % 255);
            o.b = static_cast<uint8_t>((150 * clusterNum) % 255);
            clusterCloud->push_back(o);
        }
    }
}


void getClusteredCloud(PointCloud<VPoint> & elevatedCloud,
                        array<array<int, numGrid>, numGrid> Map_Data,
                        vector<PointCloud<VPoint>>&  clusteredPoints)
{
    for (int i = 0; i < elevatedCloud.size(); i++)
    {
        float x = elevatedCloud.points[i].x;
        float y = elevatedCloud.points[i].y;
        float z = elevatedCloud.points[i].z;
        float xC = x + roiM / 2;
        float yC = y + roiM / 2;

        if (xC < 0 || xC >= roiM || yC < 0 || yC >= roiM) continue;
        int xI = (int)floor(numGrid * xC / roiM);
        int yI = (int)floor(numGrid * yC / roiM);

        int clusterNum = Map_Data[xI][yI]; //1 ~ numCluster
        int vectorInd = clusterNum - 1; //0 ~ (numCluster -1 )
        if (clusterNum != 0) {
            VPoint o;
            o.x = x;
            o.y = y;
            o.z = z;
            o.intensity = elevatedCloud.points[i].intensity;
            o.ring = elevatedCloud.points[i].ring;
            clusteredPoints[vectorInd].push_back(o);
        }
    }

}

void getBoundingBox(vector<bounding_box>& good_box,
                    vector<PointCloud<VPoint>>& good_obstacle)
{
    for(int i=0;i<good_obstacle.size();i++)
    {
        if(good_obstacle[i].points.size()<20) continue;

        double xmax,xmin,ymax,ymin;
        xmax=ymax=-100;
        xmin=ymin= 100;
        double z=10;
        for(int k=0;k<good_obstacle[i].points.size();k++)
        {
            if(xmax<good_obstacle[i].points[k].x) xmax=good_obstacle[i].points[k].x;
            if(xmin>good_obstacle[i].points[k].x) xmin=good_obstacle[i].points[k].x;
            if(ymax<good_obstacle[i].points[k].y) ymax=good_obstacle[i].points[k].y;
            if(ymin>good_obstacle[i].points[k].y) ymin=good_obstacle[i].points[k].y;
            if(z>good_obstacle[i].points[k].z) z=good_obstacle[i].points[k].z;
        }

        //clean the curb_box;
        if((xmax-xmin)*(xmax-xmin)>50
            ||xmax-xmin>10
            ||ymax-ymin>10)
            continue;

        bounding_box box;
        box.Xmax=xmax;
        box.Ymax=ymax;
        box.Xmin=xmin;
        box.Ymin=ymin;
        box.lowest_z=z;

        good_box.push_back(box);
    }

}


void find_curb_obs(std::vector<pcl::PointCloud<VPoint>::Ptr>& curb,
                   vector<bounding_box>& good_box,
                   int flag,
                   double area_xmax,double area_xmin,double area_ymax,double area_ymin) {
    vector<bounding_box> this_box;//left/right
    for (int i = 0; i < good_box.size(); i++) {
        if (flag == -1)//left
        {
            if (good_box[i].Xmax + good_box[i].Xmin < 0)
                this_box.push_back(good_box[i]);
        }
        if (flag == 1)//right
        {
            if (good_box[i].Xmax + good_box[i].Xmin > 0)
                this_box.push_back(good_box[i]);
        }
    }


    if(flag==-1)//left
    {
        for (int i = 0; i < this_box.size(); i++) {
            double x, y;//center-point
            x = (this_box[i].Xmin + this_box[i].Xmax) / 2.0;
            y = (this_box[i].Ymin + this_box[i].Ymax) / 2.0;
            if (x - area_xmin > 2)//search
            {
                while(curb.size()>1) {
                    int flag=0;
                    for (int k = 0; k < curb.size(); k++) {
                        double xc = curb[k]->points[curb[k]->points.size() - 1].x;
                        double yc = curb[k]->points[curb[k]->points.size() - 1].y;

                        if (xc >= this_box[i].Xmin - 0.5
                            && xc <= this_box[i].Xmax + 0.5
                            && yc >= this_box[i].Ymin - 1.0
                            && yc <= this_box[i].Ymax + 1.0) //in the obstacle's area
                        {
                            curb.erase(curb.begin() + k);
                            //cout << "L--" << endl;
                            break;
                        }
                        if(k==curb.size()-1)
                        {
                            flag=1;
                        }
                    }
                    if(flag==1) break;
                }
            }
        }
    }

    if(flag==1)//right
    {
        for (int i = 0; i < this_box.size(); i++) {
            double x, y;//center-point
            x = (this_box[i].Xmin + this_box[i].Xmax) / 2.0;
            y = (this_box[i].Ymin + this_box[i].Ymax) / 2.0;
            if (x - area_xmax < -2)//search
            {
                while(curb.size()>1) {
                    int flag=0;
                    for (int k = 0; k < curb.size(); k++) {
                        double xc = curb[k]->points[curb[k]->points.size() - 1].x;
                        double yc = curb[k]->points[curb[k]->points.size() - 1].y;

                        if (xc >= this_box[i].Xmin - 1.0
                            && xc <= this_box[i].Xmax + 1.0
                            && yc >= this_box[i].Ymin - 1.0
                            && yc <= this_box[i].Ymax + 1.0) //in the obstacle's area
                        {
                            curb.erase(curb.begin() + k);
                            //cout << "R--" << endl;
                            break;
                        }
                        if(k==curb.size()-1)
                        {
                            flag=1;
                        }
                    }
                    if(flag==1) break;
                }
            }
        }
    }



}





