#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>

#include <math.h>
#include <vector>
#include <iostream>
#include <map>

#include "algorithm.h"

#define M_PI		3.14159265358979323846


using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace message_filters;
using namespace cv;

ros::Publisher pub0;//elevated cloud
ros::Publisher pub1;//ground
ros::Publisher pub2;//first curb-1 all-points
ros::Publisher pub3;//first curb-1 cluster-points
ros::Publisher pub4;//obs-cluster

ros::Publisher marker_Bline;//B-spline fitting
ros::Publisher marker_square;//square of the area
ros::Publisher marker_box;//bounding box


int main(int argc, char** argv) {

    ros::init(argc, argv, "a");
    ros::NodeHandle nh("a");
    std::string input_topic="/velodyne_points";
    ///1-get_ground
    ///2-first detection and get area
    ///3-obs detection
    ///4-second detection
    ///5-B-spline
    ROS_INFO("System ready--");

    ///Publisher
    pub0 =nh.advertise<sensor_msgs::PointCloud2> ("elevateds_Cloud", 1);//elevated
    pub1 =nh.advertise<sensor_msgs::PointCloud2> ("ground_Cloud", 1);//ground
    pub2 =nh.advertise<sensor_msgs::PointCloud2> ("first_curb_allpoints", 1);//first curb-1 -all points
    pub3 =nh.advertise<sensor_msgs::PointCloud2> ("first_curb_cluster", 1);//first curb-1 -cluster
    pub4 =nh.advertise<sensor_msgs::PointCloud2> ("obs_cluster", 1);//obstacles -cluster

    marker_Bline= nh.advertise<visualization_msgs::Marker>("Bspline_curb", 10);//B-spline fitting
    marker_square=nh.advertise<visualization_msgs::Marker>("square", 10);//square of the area
    marker_box=nh.advertise<visualization_msgs::Marker>("box", 10);//bounding box

    ///Sub
    ros::AsyncSpinner spinner(8);
    spinner.start();
    ros::Subscriber obs_sub = nh.subscribe (input_topic, 8, velodyne_callback);

    ros::waitForShutdown();


    ///rate

}
