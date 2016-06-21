softkinetic_camera
===================================

General description
---------------------
This package implements a node that runs an interactive gesture camera (softkinetic) in ROS

To use this package in ROS, the DepthSenseSDK for linux needs to be downloaded. 
The download can be found here:
http://www.softkinetic.com/fr-be/support/download.aspx?EntryId=517
To download, you will need to create a free account.

#### Parameters
**camera_link** *(string, default: "/softkinetic_link")*   
 The frame ID of the camera.

**confidence_threshold** *(int, default: 150)*   
 Confidence threshold for DepthNode configuration.
 Sensor noise is filtered by increasing the threshold.
 Threshold needs to be within [0, 32767].


#### Published Topics
**depth/points** *(sensor_msgs::PointCloud2)*   
 Publishes point clouds detected by the camera.

**rgb_data** *(sensor_msgs::Image)*   
 Publishes rgb-images in bgr8 encoding.

...

Add a camera to your robot model
--------------------------------
Here we explain how to import a softkinetic camera into your robot xacro file. We have only included a descriptor for the Creative Senz3D camera, but adapt it to any Softkinetic camera should be trivial. You just need to import the xacro file describing the camera macro:
```
  <xacro:include filename="$(find softkinetic_camera)/urdf/senz3d.urdf.xacro"/>
```
And include it somewhere in your robot description file, e.g.:
```
  <sensor_senz3d parent="base_link" name="senz3d_camera">
    <origin xyz="0.05 0.0 0.3" rpy="0.0 0.0 0.0"/>
  </sensor_senz3d>
```
