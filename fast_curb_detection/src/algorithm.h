//
// Created by gotok on 18-12-18.
//

#ifndef FAST_CURB_DETECTION_ALGORITHM_H
#define FAST_CURB_DETECTION_ALGORITHM_H

#include <iostream>
#define PCL_NO_PRECOMPILE

#include "obstacle.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include "../../velodyne-master/velodyne_pointcloud/include/velodyne_pointcloud/point_types.h"

#include <sensor_msgs/point_cloud_conversion.h>
#include <pthread.h>
#include <omp.h>

#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>

#include <visualization_msgs/Marker.h>
#include <utility>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <tf/tf.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#define VPoint velodyne_pointcloud::PointXYZIR

using namespace std;
using namespace Eigen;
using namespace pcl;
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

class algorithm {};

///value----------------
















///function------------

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg);










#endif //FAST_CURB_DETECTION_ALGORITHM_H
