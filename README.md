softkinetic_camera
===================================

General description
---------------------
This package implements a node that runs an interactive gesture camera (softkinetic) in ROS

To use this package in ROS, the DepthSenseSDK for linux needs to be downloaded. 
The download can be found here:
http://www.softkinetic.com/fr-be/support/download.aspx?EntryId=517
To download, you will need to create a free account

#### Parameters
**camera_link** *(string, default: "/softkinetic_link")*   
 The frame ID of the camera.

...

#### Published Topics
**PointCloud2** *(sensor_msgs::PointCloud2)*   
 Publishes point clouds detected by the camera.

**rgb_data** *(sensor_msgs::Image)*   
 Publishes rgb-images in bgr8 encoding.

...
