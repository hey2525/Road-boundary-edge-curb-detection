# ROS Package for road boundary/edge/curb detection

# This is beta repository.

## Introduce:
  This package includes Ground-Extract, First-Time-Boundary-Detection, Object-Clustering, Obstacle-BoundingBox, Second-Time-Boundary-Detection and B-spline Curves-Fitting.
  
## Setup:
  Make sure that the followings are installed:
  1.ROS Kinetic
  2.PCL 1.7+
  3.Open CV
  4.(Optional) Autoware(autoware_msg)
  5.(Optional) velodyne_master/velodyne
  
## Dataset:
  Using rosbag.
  
  This package using rosbag-data collected by velodyne-lidar. That means u should change your databag which is collected by other lidar to rosbag type in this package. If your bag is velodyne-rosbag, you should install the Velodyne-Package.
    
  Whats more, velodyne-rosbag provide a special parameter "ring" for each point in the point cloud. The "ring" indicate which line/ring/channels this point belongs to. For example, point-a's ring[20] means it belongs to the 21st channels. If your points didn't include this parameter, you can use some common algorithms to categorize your pointcloud into different ring[]arrays.

## Ground-Extact:
  In this package, the Ground-Separate algorithm refers to the Autoware-Ground-Detection. That means you must install the Autoware-Package if you don't change a word. But you can use your own Ground-Separate algorithm to replace this step.
  
## Run
  Terminal 1 : roscore
  
  Terminal 2 : rviz
  
  Terminal 3 : rosrun fast_curb_detection main (in your workspace)
  
  Terminal 4 : rosbag play yourbag.bag
  
## Result
  ![ph1](https://github.com/hey2525/Road-boundary-edge-curb-detection/blob/master/ph1.JPG)
  
  ![ph2](https://github.com/hey2525/Road-boundary-edge-curb-detection/blob/master/ph2.JPG)
  
  ![ph3](https://github.com/hey2525/Road-boundary-edge-curb-detection/blob/master/3.png)
  
  
  

