//
// Created by gotok on 18-12-18.
//
#include "algorithm.h"

#include <iostream>
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VPoint velodyne_pointcloud::PointXYZIR
#define PI 3.1415926
#include <Eigen/Dense>
#include "../../velodyne-master/velodyne_pointcloud/include/velodyne_pointcloud/point_types.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace message_filters;

extern ros::Publisher pub0;
extern ros::Publisher pub1;
extern ros::Publisher pub2;
extern ros::Publisher pub3;
extern ros::Publisher pub4;

extern ros::Publisher marker_Bline;
extern ros::Publisher marker_square;
extern ros::Publisher marker_box;

bool my_point_cmp_k(VPoint a, VPoint b){
    return fabs(a.y/a.x)>fabs(b.y/b.x);
}

void getFloor(const  pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
              const  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_nofloor_cloud_ptr,
              const  pcl::PointCloud<pcl::PointXYZI>::Ptr &out_onlyfloor_cloud_ptr,
              float in_max_height=0.3, float in_floor_max_angle=0.1)
{
#pragma omp parallel for
    {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(in_floor_max_angle);

        seg.setDistanceThreshold(in_max_height);//floor distance
        seg.setOptimizeCoefficients(true);
        seg.setInputCloud(in_cloud_ptr);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty()) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        //REMOVE THE FLOOR FROM THE CLOUD
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(in_cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(true);//true removes the indices, false leaves only the indices
        extract.filter(*out_nofloor_cloud_ptr);

        //EXTRACT THE FLOOR FROM THE CLOUD
        extract.setNegative(false);//true removes the indices, false leaves only the indices
        extract.filter(*out_onlyfloor_cloud_ptr);
    }
}

void get_cluster_curb(std::vector<pcl::PointCloud<VPoint>::Ptr>& laser,
                      pcl::PointCloud<VPoint>& laser_curb_all,
                      std::vector<pcl::PointCloud<VPoint>::Ptr>& laser_curb,
                      int quadrant)
{
    ///restrict： x y z r diff
    ///ring : 3-21 for 32-model
    pcl::PointCloud<VPoint> laser2;//step-2 store  all-points
    pcl::PointCloud<VPoint> laser3;//step-3 store  cluster1
    pcl::PointCloud<VPoint> laser4;//step-4 store  cluster2

    int ring_max=22;
    if(quadrant==1||quadrant==2) ring_max=19;

    for(int i=3;i<ring_max;i++)
    {
        if(laser[i]->points.size()<20) continue;//size
        laser2.clear();
        laser3.clear();
        laser4.clear();

        ///step-1 sorting / find the lowest ground-z / find the max_r
        sort(laser[i]->points.begin(), laser[i]->points.end(), my_point_cmp_k);//azimuth sort

        double lowest_z=0;//lowest ground-z
        vector <double> lowest_ground;
        for(int k=0;k<laser[i]->points.size();k++) lowest_ground.push_back(laser[i]->points[k].z);
        sort(lowest_ground.begin(),lowest_ground.end());
        for(int k=10;k<20;k++) lowest_z+=lowest_ground[k];
        lowest_z=lowest_z/10.0;

        double max_r=0;//max_r
        vector <double> max_vector;
        for(int k=0;k<laser[i]->points.size();k++)
            max_vector.push_back(sqrt(laser[i]->points[k].x*laser[i]->points[k].x+
                                      laser[i]->points[k].y*laser[i]->points[k].y));
        sort(max_vector.begin(),max_vector.end());
        max_r=max_vector[max_vector.size()-5];

        ///step-2 get laser2
        for(int k=0;k<laser[i]->points.size();k++)
        {
            //your restrict
            //if(0) continue;
            laser2.push_back(laser[i]->points[k]);
        }
        if(laser2.size()<20) continue;

        ///step-3 detection
        int k_begin=0;//find the first point for detection
        for(int k=0+1;k<laser2.size()-1;k++)
        {
            if(laser2[k].z-lowest_z>0.5) continue;
            else
            {
                k_begin = k;break;
            }
        }
        for(int k=k_begin+1;k<laser2.size()-1;k++)
        {
            //curb's feature
            //x's diff < ?
            //y's diff > ?
            //z's diff > ?
            double x_diff,y_diff,z_diff;
            if(i<=9)
            {
                x_diff = 0.02;
                y_diff = 0.04;
                z_diff = 0.015;
            }
            else if(i>=10&&i<=17)
            {
                x_diff = 0.025;
                y_diff = 0.045;//0.08
                z_diff = 0.015;
            }
            else//>=18
            {
                x_diff = 0.03;
                y_diff = 0.05;//0.1
                z_diff = 0.012;
            }

            double z_diff1,z_diff2;
            z_diff1=z_diff2=z_diff;
            if(fabs(laser2[k].x)>10) {
                double x_dist = 0;
                if (fabs(laser2[k].x) > 20) //x-far
                    x_dist = 10;
                else x_dist=fabs(laser2[k].x)-10;//0-10
                z_diff1=z_diff-x_dist*0.0012;
                y_diff=y_diff-x_dist*0.005;
                if (fabs(laser2[k].y) > 20)
                    z_diff1-=0.005;
            }
            if(fabs(laser2[k].y) < 10&&i>=10)
            {
                z_diff2=z_diff-(10-fabs(laser2[k].y))*0.0012;

            }

            z_diff=z_diff1<z_diff2 ? z_diff1 : z_diff2;

            x_diff=x_diff/1.0;
            y_diff=y_diff*1.0;
            z_diff=z_diff*1.0;

            double x_change=0;
            if(quadrant==1||quadrant==4) x_change=1;
            else if(quadrant==2||quadrant==3) x_change=-1;
            double y_change=0;
            if(quadrant==1||quadrant==2) y_change=-1;
            else if(quadrant==3||quadrant==4) y_change=1;

            int flag=0;
            if((laser2[k+1].x-laser2[k].x)*x_change < x_diff
                &&(laser2[k+1].x-laser2[k].x)*x_change > -x_diff) flag++;
            if((laser2[k+1].y-laser2[k].y)*y_change > y_diff) flag++;
            if( laser2[k+1].z-laser2[k].z  > z_diff) flag++;
            if( laser2[k+1].z-laser2[k].z  < 0.004) flag--;

            if(flag>=2
                &&fabs(fabs(laser2[k].x)-fabs(laser2[k-1].x))<1
                &&fabs(fabs(laser2[k].y)-fabs(laser2[k-1].y))<1)
            {
                laser_curb_all.push_back(laser2[k]);//all curb-points
                laser3.push_back(laser2[k]);//clustering
            }
            ///step-4 clustering
            //x:0-10m:3points
            //x:>10m:5points
            else
            {
                int cluster_num;
                if(fabs(laser2[k].x)<=5) cluster_num=3;
                else if(fabs(laser2[k].x)>5&&fabs(laser2[k].x)<=10) cluster_num=4;
                else if(fabs(laser2[k].x)>10&&fabs(laser2[k].x)<=15) cluster_num=5;
                else cluster_num=8;

                if((quadrant==1||quadrant==2)&&i==18&&fabs(laser2[k].x)>10)
                    cluster_num=9;

                if(laser3.size()<cluster_num) laser3.clear();//defeat
                else
                {
                    double laser_long=0;
                    laser_long=pow((laser3[laser3.size()-1].x-laser3[0].x),2)+
                               pow((laser3[laser3.size()-1].y-laser3[0].y),2)+
                               pow((laser3[laser3.size()-1].z-laser3[0].z),2);
                    laser_long=sqrt(laser_long);
                    if(i>=17&&laser_long<0.5)
                        laser3.clear();//defeat
                    else
                    {
                        //push into laser_curb[]
                        pcl::PointCloud<VPoint>::Ptr item(new pcl::PointCloud<VPoint>());
                        laser_curb.push_back(item);
                        //for(int q=0;q<cluster_num;q++)
                        for (int q = 0; q < laser3.size(); q++) {
                            laser_curb[laser_curb.size() - 1]->push_back(laser3[q]);
                        }
                        break;
                    }
                }
            }
        }
    }

}

void Add_curb(std::vector<pcl::PointCloud<VPoint>::Ptr>& curb,
              std::vector<pcl::PointCloud<VPoint>::Ptr>& up,
              std::vector<pcl::PointCloud<VPoint>::Ptr>& down)
{
    for(int i=0;i<up.size();i++)
    {
        pcl::PointCloud<VPoint>::Ptr item(new pcl::PointCloud<VPoint>());
        curb.push_back(item);
        auto number=up.size()-1-i;
        for(int j=0;j<up[number]->size();j++)
        {
            curb[curb.size()-1]->push_back(up[number]->points[j]);
        }
    }
    for(int i=0;i<down.size();i++)
    {
        pcl::PointCloud<VPoint>::Ptr item(new pcl::PointCloud<VPoint>());
        curb.push_back(item);
        auto number=i;
        for(int j=0;j<down[number]->size();j++)
        {
            curb[curb.size()-1]->push_back(down[number]->points[down[number]->size()-1-j]);
        }
    }
}

void Add_cluster(pcl::PointCloud<VPoint>& cluster,
                 std::vector<pcl::PointCloud<VPoint>::Ptr>& left,
                 std::vector<pcl::PointCloud<VPoint>::Ptr>& right)
{
    for(int i=0;i<left.size();i++)
    {
        for(int j=0;j<left[i]->size();j++)
        {
            cluster.push_back(left[i]->points[j]);
        }
    }
    for(int i=0;i<right.size();i++)
    {
        for(int j=0;j<right[i]->size();j++)
        {
            cluster.push_back(right[i]->points[j]);
        }
    }
}


void errorpoints_detection(std::vector<pcl::PointCloud<VPoint>::Ptr>& curb)
{
    //middle
    while(curb.size()>3)
    {
        int flag=0;
        for (int i = 1; i < curb.size() - 1; i++)
        {
            double x1,y1;//point1
            x1=curb[i-1]->points[0].x;y1=curb[i-1]->points[0].y;
            double x2,y2;//point2
            x2=curb[i]->points[0].x;y2=curb[i]->points[0].y;
            double x3,y3;//point3
            x3=curb[i+1]->points[0].x;y3=curb[i+1]->points[0].y;
            if(fabs(x2)-fabs(x1)>5
               &&fabs(x2)-fabs(x3)>5)
            {
                curb.erase(curb.begin()+i);
                break;
            }
            if(i==curb.size()-2) flag=1;
        }
        if(flag==1) break;
    }

    while(curb.size()>3) // up
    {
        auto top=0;
        double x1,y1;//point1
        x1=curb[top]->points[0].x;y1=curb[top]->points[0].y;
        double x2,y2;//point2
        x2=curb[top+1]->points[0].x;y2=curb[top+1]->points[0].y;
        double x3,y3;//point3
        x3=curb[top+2]->points[0].x;y3=curb[top+2]->points[0].y;
        double dx1, dx2, dy1, dy2;
        double angle;
        dx1 = x1 - x2;
        dy1 = y1 - y2;
        dx2 = x3 - x2;
        dy2 = y3 - y2;
        double c = sqrt(dx1 * dx1 + dy1 * dy1) * sqrt(dx2 * dx2 + dy2 * dy2);
        if(c==0) c+=0.00001;
        angle = acos((dx1 * dx2 + dy1 * dy2) / c);
        angle = angle* 180.0 / 3.1415926;
        if(c==0) angle =180;

        int flag=1;
        if(fabs(curb[top]->points[0].ring - curb[top+1]->points[0].ring)<=2
            && ((y1>=0&&y2>=0)||(y1<0&&y2<0)))
        {
            if(sqrt(pow(x2-x1,2)+pow(y2-y1,2)) > 10)
            {
                flag=0;
            }
        }

        if(fabs(fabs(x1)-fabs(x2))>5)
        {
            flag=0;
        }

        if(angle<120||flag==0)
        {
            curb.erase(curb.begin()+top);
        }
        else break;

    }
    while(curb.size()>3) // down
    {
        auto top=curb.size()-1;
        double x1,y1;//point1
        x1=curb[top]->points[0].x;y1=curb[top]->points[0].y;
        double x2,y2;//point2
        x2=curb[top-1]->points[0].x;y2=curb[top-1]->points[0].y;
        double x3,y3;//point3
        x3=curb[top-2]->points[0].x;y3=curb[top-2]->points[0].y;
        double dx1, dx2, dy1, dy2;
        double angle;
        dx1 = x1 - x2;
        dy1 = y1 - y2;
        dx2 = x3 - x2;
        dy2 = y3 - y2;
        double c = sqrt(dx1 * dx1 + dy1 * dy1) * sqrt(dx2 * dx2 + dy2 * dy2);
        if(c==0) c+=0.00001;
        angle = acos((dx1 * dx2 + dy1 * dy2) / c);
        angle = angle* 180.0 / 3.1415926;
        if(c==0) angle =180;
        //cout<<x1<<" "<<y1<<"   "<<x2<<" "<<y2<<"   "<<x3<<" "<<y3<<"    "<<angle<<endl;

        int flag=1;
        if(fabs(curb[top]->points[0].ring - curb[top-1]->points[0].ring)<=2
           && ((y1>=0&&y2>=0)||(y1<0&&y2<0)))
        {
            if(sqrt(pow(x2-x1,2)+pow(y2-y1,2)) > 10)
            {
                flag=0;
            }
        }

        if(fabs(fabs(x1)-fabs(x2))>5)
        {
            flag=0;
        }

        if(angle<120||flag==0)
        {
            curb.erase(curb.begin()+top);
        }
        else break;
    }


}

void errorpoints_detection2(std::vector<pcl::PointCloud<VPoint>::Ptr>& curb)
{
    //middle
    while(curb.size()>2)
    {
        int flag=0;
        for (int i = 1; i < curb.size() - 1; i++)
        {
            double x1,y1;//point1
            x1=curb[i-1]->points[0].x;y1=curb[i-1]->points[0].y;
            double x2,y2;//point2
            x2=curb[i]->points[0].x;y2=curb[i]->points[0].y;
            double x3,y3;//point3
            x3=curb[i+1]->points[0].x;y3=curb[i+1]->points[0].y;
            if(fabs(x2)-fabs(x1)<-5
               &&fabs(x2)-fabs(x3)<-5)
            {
                curb.erase(curb.begin()+i);
                break;
            }
            if(i==curb.size()-2) flag=1;
        }
        if(flag==1) break;
    }

    while(curb.size()>2)
    {
        double x1,y1;//point1
        x1=curb[0]->points[0].x;y1=curb[0]->points[0].y;
        double x2,y2;//point2
        x2=curb[1]->points[0].x;y2=curb[1]->points[0].y;
        if(fabs(x1)-fabs(x2)<-5)
        {
            curb.erase(curb.begin());
        }
        else
            break;
    }
    while(curb.size()>2)
    {
        double x1,y1;//point1
        auto flag=curb.size()-1;
        x1=curb[flag]->points[0].x;y1=curb[flag]->points[0].y;
        double x2,y2;//point2
        x2=curb[flag-1]->points[0].x;y2=curb[flag-1]->points[0].y;
        //cout<<"x1"<<x1<<" x2"<<x2<<endl;
        if(fabs(x1)-fabs(x2)<-5)
        {
            curb.erase(curb.begin()+flag);
        }
        else
            break;
    }
}

double bezierPoint(vector<double>& P, double t,unsigned long number)
{
    vector<double> P1;
    for(int j=0; j<number; j++)
        P1.push_back((P[j]));
    auto n = number - 1;
    for(int r=0; r<n; r++)
    {
        for(int i=0; i<n-r; i++)
        {
            P1[i] = (1.0 - t) * P1[i] + t * P1[i+1];
        }
    }
    return P1[0];
}

void bezierLine(geometry_msgs::Point p[1000],std::vector<pcl::PointCloud<VPoint>::Ptr>& curb)
{
    vector<double> Px;
    vector<double> Py;
    for(int i=0;i<curb.size();i++)
    {
        for(int j=0;j<curb[i]->size();j++)
        {
            Px.push_back(curb[i]->points[j].x);
            Py.push_back(curb[i]->points[j].y);
        }
    }
    double delta = 1.0 / 1000;
    double t = 0.0;
    for(int i=0; i<1000; i++)
    {
        double x = bezierPoint(Px, t,Px.size());
        double y = bezierPoint(Py, t,Py.size());
        t += delta;
        p[i].x=x;p[i].y=y;p[i].z=-2;
    }
}

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg)
{
    ///1-get_ground
    ///2-first detection and get area
    ///3-obs detection
    ///4-second detection
    ///5-B-spline
    ///input cloud: in_sensor_cloud

    ///step-1 : in_sensor_cloud -> ground_cloud
    ///step-1.1 : in_sensor_cloud -> XYZI
    pcl::PointCloud<VPoint> VPoint_cloud;//XYZIR-ground
    pcl::PointCloud<VPoint> VPoint_elevated;//XYZIR-elevated
    pcl::PointCloud<pcl::PointXYZI>::Ptr VPoint2_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);//XYZI
    pcl::fromROSMsg(*in_cloud_msg, *VPoint2_cloud_ptr);//convert->XYZI
    ///step-1.2 : XYZI -> ground_cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr onlyfloor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getFloor(VPoint2_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr);
    sensor_msgs::PointCloud2 ground_cloud;
    sensor_msgs::PointCloud2 elevated_cloud;
    pcl::toROSMsg(*onlyfloor_cloud_ptr,ground_cloud);
    pcl::toROSMsg(*nofloor_cloud_ptr,elevated_cloud);
    pcl::fromROSMsg(ground_cloud, VPoint_cloud);
    pcl::fromROSMsg(elevated_cloud, VPoint_elevated);
    ///step-1.3 :publish:ground_cloud/elevated cloud
    elevated_cloud.header.stamp = in_cloud_msg->header.stamp;
    elevated_cloud.header.frame_id = in_cloud_msg->header.frame_id;
    pub0.publish(elevated_cloud);
    ground_cloud.header.stamp = in_cloud_msg->header.stamp;
    ground_cloud.header.frame_id = in_cloud_msg->header.frame_id;
    pub1.publish(ground_cloud);

    ///step-2:VPoint_cloud(only ground)->first curbs points->area
    ///step-2.1 : laser all/1/2/3/4
    std::vector<pcl::PointCloud<VPoint>::Ptr> laserCloudScans;//all
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser1;//quadrant1 right up
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser2;//quadrant2 left  up
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser3;//quadrant3 left  down
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser4;//quadrant4 right down
    auto sensor_model = 32;//lidar's sensor_model_ lines

    for(int i=0;i<sensor_model;i++){
        pcl::PointCloud<VPoint>::Ptr item( new pcl::PointCloud<VPoint>());
        pcl::PointCloud<VPoint>::Ptr item1( new pcl::PointCloud<VPoint>());
        pcl::PointCloud<VPoint>::Ptr item2( new pcl::PointCloud<VPoint>());
        pcl::PointCloud<VPoint>::Ptr item3( new pcl::PointCloud<VPoint>());
        pcl::PointCloud<VPoint>::Ptr item4( new pcl::PointCloud<VPoint>());
        laserCloudScans.push_back(item);
        laser1.push_back(item1);
        laser2.push_back(item2);
        laser3.push_back(item3);
        laser4.push_back(item4);
    }
    std::vector<int> indices_;//store the reserve points index
    pcl::removeNaNFromPointCloud(VPoint_cloud, VPoint_cloud, indices_);
    for(int i=0;i<sensor_model;i++)
    {
        laserCloudScans[i]->clear();
        laser1[i]->clear();
        laser2[i]->clear();
        laser3[i]->clear();
        laser4[i]->clear();
    }

    VPoint point_temp;
    for(int i=0;i<VPoint_cloud.points.size();i++)
    {
        point_temp.x = VPoint_cloud.points[i].x;
        point_temp.y = VPoint_cloud.points[i].y;
        point_temp.z = VPoint_cloud.points[i].z;
        point_temp.intensity = VPoint_cloud.points[i].intensity;
        point_temp.ring = VPoint_cloud.points[i].ring;
        if(point_temp.ring<sensor_model&&point_temp.ring>=0)
        {
            laserCloudScans[point_temp.ring]->push_back(point_temp);
            if(fabs(point_temp.x)>25||fabs(point_temp.y)>25) continue;//50*50 for 32-model
            if(point_temp.x>=0&&point_temp.y>=0) laser1[point_temp.ring]->push_back(point_temp);
            else if(point_temp.x<0&&point_temp.y>=0) laser2[point_temp.ring]->push_back(point_temp);
            else if(point_temp.x<0&&point_temp.y<0) laser3[point_temp.ring]->push_back(point_temp);
            else if(point_temp.x>=0&&point_temp.y<0) laser4[point_temp.ring]->push_back(point_temp);
        }
    }
    ///step-2.2 : first time detection
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser1_curb;//clustering curb-points
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser2_curb;
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser3_curb;
    std::vector<pcl::PointCloud<VPoint>::Ptr> laser4_curb;
    pcl::PointCloud<VPoint> first_curb_all;//all curb-points
    pcl::PointCloud<VPoint> first_curb_cluster;//cluster curb-points
    get_cluster_curb(laser1,first_curb_all,laser1_curb,1);
    get_cluster_curb(laser2,first_curb_all,laser2_curb,2);
    get_cluster_curb(laser3,first_curb_all,laser3_curb,3);
    get_cluster_curb(laser4,first_curb_all,laser4_curb,4);

    ///step-2.3 : add&&publish: first curb all/cluster
    std::vector<pcl::PointCloud<VPoint>::Ptr> first_left_curb;//cluster left-boundary
    std::vector<pcl::PointCloud<VPoint>::Ptr> first_right_curb;//cluster right-boundary
    Add_curb(first_right_curb,laser1_curb,laser4_curb);//1 4->right
    Add_curb(first_left_curb,laser2_curb,laser3_curb);//2 3->left

        ///step-2.3.1:publish: first curb-all
    sensor_msgs::PointCloud2 first_curb_cloud_all;
    pcl::toROSMsg(first_curb_all,first_curb_cloud_all);
    first_curb_cloud_all.header.stamp = in_cloud_msg->header.stamp;
    first_curb_cloud_all.header.frame_id = in_cloud_msg->header.frame_id;
    pub2.publish(first_curb_cloud_all);

        ///step-2.3.2:delete error curb-points
    if(first_left_curb.size()>3)
        errorpoints_detection(first_left_curb);
    if(first_right_curb.size()>3)
        errorpoints_detection(first_right_curb);

       ///step-2.3.3:publish: first curb-cluster
    sensor_msgs::PointCloud2 first_curb_cloud_cluster;
    Add_cluster(first_curb_cluster,first_left_curb,first_right_curb);
    pcl::toROSMsg(first_curb_cluster,first_curb_cloud_cluster);
    first_curb_cloud_cluster.header.stamp = in_cloud_msg->header.stamp;
    first_curb_cloud_cluster.header.frame_id = in_cloud_msg->header.frame_id;
    pub3.publish(first_curb_cloud_cluster);

    ///step-3.1:obstacle-clustering
        ///step-3.1.1:find clustering area
    double area_xmax,area_xmin,area_ymax,area_ymin;
    area_xmax=area_xmin=area_ymax=area_ymin=0;
    for(int i=0;i<first_curb_cluster.size();i++)
    {
        if(area_xmax<first_curb_cluster[i].x) area_xmax=first_curb_cluster[i].x;
        if(area_xmin>first_curb_cluster[i].x) area_xmin=first_curb_cluster[i].x;
        if(area_ymax<first_curb_cluster[i].y) area_ymax=first_curb_cluster[i].y;
        if(area_ymin>first_curb_cluster[i].y) area_ymin=first_curb_cluster[i].y;
    }
    pcl::PointCloud<VPoint> VPoint_area;//elevated in the area
#pragma omp parallel for
    {
        for(int i=0;i<VPoint_elevated.size();i++)
        {
            if(VPoint_elevated[i].x<area_xmax+2
                &&VPoint_elevated[i].x>area_xmin-2
                &&VPoint_elevated[i].y<area_ymax+2
                &&VPoint_elevated[i].y>area_ymin-2)
            {
                if(VPoint_elevated[i].z<0
                    &&fabs(VPoint_elevated[i].x)<1
                    &&VPoint_elevated[i].y>-3
                    &&VPoint_elevated[i].y< 3);
                else VPoint_area.push_back(VPoint_elevated[i]);//clean points from the car
            }
        }
    }
        ///step-3.1.2:show the bounding-square
    {
        visualization_msgs::Marker square_list;
        square_list.header.frame_id = in_cloud_msg->header.frame_id;
        square_list.ns = "square";
        square_list.action = visualization_msgs::Marker::ADD;
        square_list.pose.orientation.w = 1.0;
        square_list.id = 10;
        square_list.type = visualization_msgs::Marker::LINE_LIST;
        square_list.scale.x = 0.4;
        square_list.color.r = (float)(255/255.0);
        square_list.color.g = (float)(0/255.0);
        square_list.color.b = (float)(128/255.0);
        square_list.color.a = 1.0;
        geometry_msgs::Point NE,NW,SW,SE;//four corners
        NE.z=NW.z=SW.z=SE.z=0;
        NE.x=SE.x=area_xmax+2;
        NW.x=SW.x=area_xmin-2;
        NE.y=NW.y=area_ymax+2;
        SE.y=SW.y=area_ymin-2;
        square_list.points.push_back(NE);
        square_list.points.push_back(NW);
        square_list.points.push_back(NW);
        square_list.points.push_back(SW);
        square_list.points.push_back(SW);
        square_list.points.push_back(SE);
        square_list.points.push_back(SE);
        square_list.points.push_back(NE);
        marker_square.publish(square_list);
    }


        ///step-3.1.3:clustering obs in the area
    int numCluster = 0;//number of all obs-clusters
    array<array<int, numGrid>, numGrid> Map_Data{};
    componentClustering(VPoint_area, Map_Data, numCluster);

        ///step-3.1.4:publish cluster result
    PointCloud<pcl::PointXYZRGB>::Ptr cluster_obs_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    makeClusteredCloud(VPoint_area, Map_Data, cluster_obs_cloud);
    sensor_msgs::PointCloud2 obstacle_cluster;
    pcl::toROSMsg(*cluster_obs_cloud,obstacle_cluster);
    obstacle_cluster.header.stamp = in_cloud_msg->header.stamp;
    obstacle_cluster.header.frame_id = in_cloud_msg->header.frame_id;
    pub4.publish(obstacle_cluster);
    cout<<"total number of obs-Cluster:"<<numCluster<<endl;

    ///step-3.2:cluster real obstacle
    vector<PointCloud<VPoint>> good_obstacle(numCluster);
    getClusteredCloud(VPoint_area, Map_Data,good_obstacle);

    ///step-3.3:get bounding-box and publish
    vector<bounding_box> good_box;
    getBoundingBox(good_box,good_obstacle);

    visualization_msgs::Marker box_list;
    box_list.header.frame_id = in_cloud_msg->header.frame_id;
    box_list.ns = "bounding_box";
    box_list.action = visualization_msgs::Marker::ADD;
    box_list.pose.orientation.w = 1.0;
    box_list.id = 11;
    box_list.type = visualization_msgs::Marker::LINE_LIST;
    box_list.scale.x = 0.13;
    box_list.color.r = (float)(50/255.0);
    box_list.color.g = (float)(255/255.0);
    box_list.color.b = (float)(0/255.0);
    box_list.color.a = 1.0;
    cout<<"number of boxes:"<<good_box.size()<<endl;
    for(int i=0;i<good_box.size();i++)
    {
        geometry_msgs::Point NE,NW,SW,SE;//four corners
        NE.z=NW.z=SW.z=SE.z=0;
        NE.x=SE.x=good_box[i].Xmax;
        NW.x=SW.x=good_box[i].Xmin;
        NE.y=NW.y=good_box[i].Ymax;
        SE.y=SW.y=good_box[i].Ymin;
        box_list.points.push_back(NE);
        box_list.points.push_back(NW);
        box_list.points.push_back(NW);
        box_list.points.push_back(SW);
        box_list.points.push_back(SW);
        box_list.points.push_back(SE);
        box_list.points.push_back(SE);
        box_list.points.push_back(NE);
    }
    marker_box.publish(box_list);



    ///step-4 : second time detection
    find_curb_obs(first_left_curb,good_box,-1,area_xmax,area_xmin,area_ymax,area_ymin);
    find_curb_obs(first_right_curb,good_box,1,area_xmax,area_xmin,area_ymax,area_ymin);



        ///step-4.1:deal with difficult situation
    if(first_left_curb.size()>2)
        errorpoints_detection2(first_left_curb);
    if(first_right_curb.size()>2)
        errorpoints_detection2(first_right_curb);


    ///step-5 : B-spline fitting
    visualization_msgs::Marker Bline_list;
    Bline_list.header.frame_id = in_cloud_msg->header.frame_id;
    Bline_list.ns = "Bline_curb";
    Bline_list.action = visualization_msgs::Marker::ADD;
    Bline_list.pose.orientation.w = 1.0;
    Bline_list.id = 9;
    Bline_list.type = visualization_msgs::Marker::LINE_LIST;
    Bline_list.scale.x = 0.4;
    Bline_list.color.r = (float)(0/255.0);
    Bline_list.color.g = (float)(128/255.0);
    Bline_list.color.b = (float)(255/255.0);
    Bline_list.color.a = 1.0;
    if(first_left_curb.size()>1)
    {
        geometry_msgs::Point p_list1[1000];
        bezierLine(p_list1,first_left_curb);
        for(int ii=0;ii<999;ii++) {
            Bline_list.points.push_back(p_list1[ii]);
            Bline_list.points.push_back(p_list1[ii+1]);
        }
    }
    if(first_right_curb.size()>1)
    {
        geometry_msgs::Point p_list2[1000];
        bezierLine(p_list2,first_right_curb);
        for(int ii=0;ii<999;ii++) {
            Bline_list.points.push_back(p_list2[ii]);
            Bline_list.points.push_back(p_list2[ii+1]);
        }
    }
    marker_Bline.publish(Bline_list);


}