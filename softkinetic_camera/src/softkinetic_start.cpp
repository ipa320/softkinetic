/*!
 *****************************************************************
 *
 *   Copyright (c) 2014
 *
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA)
 *
 *****************************************************************
 *
 *   ROS package name: softkinetic_camera
 *
 *   Author: Felipe Garcia Lopez, email: flg@ipa.fhg.de
 *
 *   Date of creation: March 2014
 *
 *****************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************/

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <signal.h>
#include <vector>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>

// ROS include files
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/frustum_culling.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <softkinetic_camera/SoftkineticConfig.h>

#include <DepthSense.hxx>

using namespace DepthSense;
using namespace message_filters;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ros::Publisher pub_cloud;
ros::Publisher pub_rgb_info;
ros::Publisher pub_depth_info;
image_transport::Publisher pub_rgb;
image_transport::Publisher pub_mono;
image_transport::Publisher pub_depth;

sensor_msgs::CameraInfo rgb_info;
sensor_msgs::CameraInfo depth_info;

sensor_msgs::Image img_rgb;
sensor_msgs::Image img_mono;
sensor_msgs::Image img_depth;
sensor_msgs::PointCloud2 cloud;

cv::Mat cv_img_rgb; // Open CV image containers
cv::Mat cv_img_yuy2;
cv::Mat cv_img_mono;
cv::Mat cv_img_depth;

std_msgs::Int32 test_int;

/* Confidence threshold for DepthNode configuration */
int confidence_threshold;

/* Parameters for downsampling cloud */
bool use_voxel_grid_filter;
double voxel_grid_size;

/* Parameters for radius filter */
bool use_radius_outlier_filter;
double search_radius;
int min_neighbours;

/* Parameters for passthrough filer */
bool use_passthrough_filter;
double limit_min;
double limit_max;

/* Parameters for frustum culling filer */
bool use_frustum_culling_filter;
double hfov;
double vfov;
double near_plane;
double far_plane;

/* Shutdown request */
bool ros_node_shutdown = false;

/* Depth sensor parameters */
bool depth_enabled;
DepthSense::DepthNode::CameraMode depth_mode;
DepthSense::FrameFormat depth_frame_format;
int depth_frame_rate;

/* Color sensor parameters */
bool color_enabled;
DepthSense::CompressionType color_compression;
DepthSense::FrameFormat color_frame_format;
int color_frame_rate;

/* Serial Number parameters */
bool use_serial;
std::string serial;

DepthSense::DepthNode::CameraMode depthMode(const std::string& depth_mode_str)
{
    if ( depth_mode_str == "long" )
        return DepthNode::CAMERA_MODE_LONG_RANGE;
    else // if ( depth_mode_str == "close" )
        return DepthNode::CAMERA_MODE_CLOSE_MODE;
}

DepthSense::FrameFormat depthFrameFormat(const std::string& depth_frame_format_str)
{
  if ( depth_frame_format_str == "QQVGA" )
    return FRAME_FORMAT_QQVGA;
  else if ( depth_frame_format_str == "QVGA" )
    return FRAME_FORMAT_QVGA;
  else // if ( depth_frame_format_str == "VGA" )
    return FRAME_FORMAT_VGA;
}

DepthSense::CompressionType colorCompression(const std::string& color_compression_str)
{
  if ( color_compression_str == "YUY2" )
    return COMPRESSION_TYPE_YUY2;
  else // if ( color_compression_str == "MJPEG" )
    return COMPRESSION_TYPE_MJPEG;
}

DepthSense::FrameFormat colorFrameFormat(const std::string& color_frame_format_str)
{
  if ( color_frame_format_str == "QQVGA" )
    return FRAME_FORMAT_QQVGA;
  else if ( color_frame_format_str == "QVGA" )
    return FRAME_FORMAT_QVGA;
  else if ( color_frame_format_str == "VGA" )
    return FRAME_FORMAT_VGA;
  else if ( color_frame_format_str == "NHD" )
    return FRAME_FORMAT_NHD;
  else // if ( color_frame_format_str == "WXGA_H" )
    return FRAME_FORMAT_WXGA_H;
}

/*----------------------------------------------------------------------------*/
// New audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
  //printf("A#%u: %d\n",g_aFrames,data.audioData.size());
  ++g_aFrames;
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
  // If this is the first sample, we fill all the constant values
  // on image and camera info messages to increase working rate
  if (img_rgb.data.size() == 0)
  {
    // Create two sensor_msg::Image for color and grayscale images on new camera image
    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);

    img_rgb.width  = w;
    img_rgb.height = h;
    img_rgb.encoding = "bgr8";
    img_rgb.data.resize(w * h * 3);
    img_rgb.step = w * 3;

    img_mono.width  = w;
    img_mono.height = h;
    img_mono.encoding = "mono8";
    img_mono.data.resize(w * h);
    img_mono.step = w;

    cv_img_rgb.create(h, w, CV_8UC3);
    if (color_compression == COMPRESSION_TYPE_YUY2)
      cv_img_yuy2.create(h, w, CV_8UC2);
  }

  if (color_compression == COMPRESSION_TYPE_YUY2)
  {
    // Color images come compressed as YUY2, so we must convert them to BGR
    cv_img_yuy2.data = reinterpret_cast<uchar *>(
          const_cast<uint8_t *>(static_cast<const uint8_t *>(data.colorMap)));
    cvtColor(cv_img_yuy2, cv_img_rgb, CV_YUV2BGR_YUY2);
  }
  else
  {
    // Nothing special to do for MJPEG stream; just cast and reuse the camera data
    cv_img_rgb.data = reinterpret_cast<uchar *>(
          const_cast<uint8_t *>(static_cast<const uint8_t *>(data.colorMap)));
  }
  // Create also a gray-scale image from the BGR one
  cvtColor(cv_img_rgb, cv_img_mono, CV_BGR2GRAY);

  // Dump both on ROS image messages
  std::memcpy(img_rgb.data.data(),  cv_img_rgb.ptr(),  img_rgb.data.size());
  std::memcpy(img_mono.data.data(), cv_img_mono.ptr(), img_mono.data.size());

  // Ensure that all the images and camera info are timestamped and have the proper frame id
  img_rgb.header.stamp = ros::Time::now();
  img_mono.header      = img_rgb.header;
  rgb_info.header      = img_rgb.header;

  // Publish the rgb and mono images and camera info
  pub_rgb.publish(img_rgb);
  pub_mono.publish(img_mono);

  pub_rgb_info.publish(rgb_info);

  ++g_cFrames;
}

/*----------------------------------------------------------------------------*/

void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filter)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_to_filter);
  sor.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
  // apply filter
  ROS_DEBUG_STREAM("Starting downsampling");
  int before = cloud_to_filter->size();
  double old = ros::Time::now().toSec();
  sor.filter(*cloud_to_filter);
  double new_ = ros::Time::now().toSec() - old;
  int after = cloud_to_filter->size();
  ROS_DEBUG_STREAM("downsampled in " << new_ << " seconds; "
                << "points reduced from " << before << " to " << after);
}

void filterCloudRadiusBased(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filter)
{
  // Radius outlier removal filter:
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
  ror.setInputCloud(cloud_to_filter);
  ror.setRadiusSearch(search_radius);
  ror.setMinNeighborsInRadius(min_neighbours);
  ror.setKeepOrganized(true);
  // apply filter
  ROS_DEBUG_STREAM("Starting radius outlier removal filtering");
  int before = cloud_to_filter->size();
  double old = ros::Time::now().toSec();
  ror.filter(*cloud_to_filter);
  double new_ = ros::Time::now().toSec() - old;
  int after = cloud_to_filter->size();
  ROS_DEBUG_STREAM("filtered in " << new_ << " seconds; "
                << "points reduced from " << before << " to " << after);
}

void filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filter)
{
  // Passthrough filter:
  pcl::PassThrough<pcl::PointXYZRGB> pt;
  pt.setInputCloud(cloud_to_filter);
  pt.setFilterFieldName("x");
  pt.setFilterLimits(limit_min, limit_max);
  pt.setKeepOrganized(true);
  // apply filter
  ROS_DEBUG_STREAM("Starting passthrough filtering");
  int before = cloud_to_filter->size();
  double old = ros::Time::now().toSec();
  pt.filter(*cloud_to_filter);
  double new_= ros::Time::now().toSec() - old;
  int after = cloud_to_filter->size();
  ROS_DEBUG_STREAM("filtered in " << new_ << " seconds; "
                << "points reduced from " << before << " to " << after);
}

void filterFrustumCulling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filter)
{
  // Frustum culling filter:
  pcl::FrustumCulling<pcl::PointXYZRGB> fc;
  fc.setInputCloud(cloud_to_filter);
  // PCL assumes a coordinate system where X is forward, Y is up, and Z is right.
  // Therefore we must convert from the traditional camera coordinate system
  // (X right, Y down, Z forward), which is used by this RGBD camera.
  // See:
  // http://docs.pointclouds.org/trunk/classpcl_1_1_frustum_culling.html#ae22a939225ebbe244fcca8712133fcf3
  // XXX We are not using the same frame for the pointcloud as kinect! see issue #46
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T;
  T << 1, 0, 0, 0,
       0, 0, 1, 0,
       0,-1, 0, 0,
       0, 0, 0, 1;
  pose *= T;

  fc.setCameraPose(pose);
  fc.setHorizontalFOV(hfov);
  fc.setVerticalFOV(vfov);
  fc.setNearPlaneDistance(near_plane);
  fc.setFarPlaneDistance(far_plane);
  fc.setKeepOrganized(true);
  // apply filter
  ROS_DEBUG_STREAM("Starting frustum culling filtering");
  int before = cloud_to_filter->size();
  double old = ros::Time::now().toSec();
  fc.filter(*cloud_to_filter);
  double new_= ros::Time::now().toSec() - old;
  int after = cloud_to_filter->size();
  ROS_DEBUG_STREAM("filtered in " << new_ << " seconds; "
                << "points reduced from " << before << " to " << after);
}

void setupCameraInfo(const DepthSense::IntrinsicParameters& params, sensor_msgs::CameraInfo& cam_info)
{
  cam_info.distortion_model = "plumb_bob";
  cam_info.height = params.height;
  cam_info.width  = params.width;

  // Distortion parameters D = [k1, k2, t1, t2, k3]
  cam_info.D.resize(5);
  cam_info.D[0] = params.k1;
  cam_info.D[1] = params.k2;
  cam_info.D[2] = params.p1;
  cam_info.D[3] = params.p2;
  cam_info.D[4] = params.k3;

  // Intrinsic camera matrix for the raw (distorted) images:
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  cam_info.K[0] = params.fx;
  cam_info.K[2] = params.cx;
  cam_info.K[4] = params.fy;
  cam_info.K[5] = params.cy;
  cam_info.K[8] = 1.0;

  // Rectification matrix (stereo cameras only)
  //     [1 0 0]
  // R = [0 1 0]
  //     [0 0 1]
  cam_info.R[0] = 1.0;
  cam_info.R[4] = 1.0;
  cam_info.R[8] = 1.0;

  // Projection/camera matrix; we use the same values as in the raw image, as we are not
  // applying any correction (WARN: is this ok?). For monocular cameras, Tx = Ty = 0.
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  cam_info.P[0] = params.fx;
  cam_info.P[2] = params.cx;
  cam_info.P[5] = params.fy;
  cam_info.P[6] = params.cy;
  cam_info.P[10] = 1.0;
}

// New depth sample event
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
  // If this is the first sample, we fill all the constant values
  // on image and camera info messages to increase working rate
  if (img_depth.data.size() == 0)
  {
    FrameFormat_toResolution(data.captureConfiguration.frameFormat,
                             (int32_t*)&img_depth.width, (int32_t*)&img_depth.height);
    img_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    img_depth.is_bigendian = 0;
    img_depth.step = sizeof(float) * img_depth.width;
    std::size_t data_size = img_depth.width * img_depth.height;
    img_depth.data.resize(data_size * sizeof(float));

    if (rgb_info.D.size() == 0)
    {
      // User didn't provide a calibration file for the color camera, so
      // fill camera info with the parameters provided by the camera itself
      setupCameraInfo(data.stereoCameraParameters.colorIntrinsics, rgb_info);
    }

    if (depth_info.D.size() == 0)
    {
      // User didn't provide a calibration file for the depth camera, so
      // fill camera info with the parameters provided by the camera itself
      setupCameraInfo(data.stereoCameraParameters.depthIntrinsics, depth_info);
    }
  }

  int32_t w = img_depth.width;
  int32_t h = img_depth.height;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  current_cloud->header.frame_id = cloud.header.frame_id;
  current_cloud->width  = w;
  current_cloud->height = h;
  current_cloud->is_dense = true;
  current_cloud->points.resize(w * h);

  ++g_dFrames;

  // Dump depth map on image message, though we must do some post-processing for saturated pixels
  std::memcpy(img_depth.data.data(), data.depthMapFloatingPoint, img_depth.data.size());

  for (int count = 0; count < w * h; count++)
  {
    // Saturated pixels on depthMapFloatingPoint have -1 value, but on openni are NaN
    if (data.depthMapFloatingPoint[count] < 0.0)
    {
      *reinterpret_cast<float*>(&img_depth.data[count*sizeof(float)]) =
          std::numeric_limits<float>::quiet_NaN();

      // We don't process these pixels as they correspond to all-zero 3D points; but
      // we keep them in the pointcloud so the downsampling filter can be applied
      continue;
    }

    // Get mapping between depth map and color map, assuming we have a RGB image
    if (img_rgb.data.size() == 0)
    {
      ROS_WARN_THROTTLE(2.0, "Color image is empty; pointcloud will be colorless");
      continue;
    }
    UV uv = data.uvMap[count];
    if (uv.u != -FLT_MAX && uv.v != -FLT_MAX)
    {
      // Within bounds: depth fov is significantly wider than color's
      // one, so there are black points in the borders of the pointcloud
      int x_pos = (int)round(uv.u*img_rgb.width);
      int y_pos = (int)round(uv.v*img_rgb.height);
      current_cloud->points[count].b = cv_img_rgb.at<cv::Vec3b>(y_pos, x_pos)[0];
      current_cloud->points[count].g = cv_img_rgb.at<cv::Vec3b>(y_pos, x_pos)[1];
      current_cloud->points[count].r = cv_img_rgb.at<cv::Vec3b>(y_pos, x_pos)[2];
    }
    else
    {
      continue;
    }
    // Convert softkinetic vertices into a kinect-like coordinates pointcloud
    current_cloud->points[count].x =   data.verticesFloatingPoint[count].x + 0.025;
    current_cloud->points[count].y = - data.verticesFloatingPoint[count].y;
    current_cloud->points[count].z =   data.verticesFloatingPoint[count].z;
  }

  // Check for usage of voxel grid filtering to downsample point cloud
  // XXX This must be the first filter to be called, as it requires the "squared" point cloud
  // we create (with proper values for width and height) that any other filter would destroy
  if (use_voxel_grid_filter)
  {
    downsampleCloud(current_cloud);
  }

  // Check for usage of passthrough filtering
  if (use_passthrough_filter)
  {
    filterPassThrough(current_cloud);
  }

  // Check for usage of frustum culling filtering
  if (use_frustum_culling_filter)
  {
    filterFrustumCulling(current_cloud);
  }

  // Check for usage of radius outlier filtering
  if (use_radius_outlier_filter)
  {
    // XXX Use any other filter before this one to remove the large amount of all-zero points the
    // camera creates for saturated pixels; if not, radius outlier filter takes really, really long
    if (use_voxel_grid_filter || use_passthrough_filter || use_frustum_culling_filter)
      filterCloudRadiusBased(current_cloud);
    else
      ROS_WARN_THROTTLE(2.0, "Calling radius outlier removal as the only filter would take too long!");
  }

  // Convert current_cloud to PointCloud2 and publish
  pcl::toROSMsg(*current_cloud, cloud);

  img_depth.header.stamp = ros::Time::now();
  cloud.header.stamp = ros::Time::now();
  depth_info.header      = img_depth.header;

  pub_cloud.publish(cloud);
  pub_depth.publish(img_depth);
  pub_depth_info.publish(depth_info);
}

/*----------------------------------------------------------------------------*/
void configureAudioNode()
{
  g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

  AudioNode::Configuration config = g_anode.getConfiguration();
  config.sampleRate = 44100;

  try
  {
    g_context.requestControl(g_anode, 0);

    g_anode.setConfiguration(config);
    g_anode.setInputMixerLevel(0.5f);
  }
  catch (ArgumentException& e)
  {
    printf("Argument Exception: %s\n", e.what());
  }
  catch (UnauthorizedAccessException& e)
  {
    printf("Unauthorized Access Exception: %s\n", e.what());
  }
  catch (ConfigurationException& e)
  {
    printf("Configuration Exception: %s\n", e.what());
  }
  catch (StreamingException& e)
  {
    printf("Streaming Exception: %s\n", e.what());
  }
  catch (TimeoutException&)
  {
    printf("TimeoutException\n");
  }
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
  g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

  DepthNode::Configuration config = g_dnode.getConfiguration();

  config.frameFormat = depth_frame_format;
  config.framerate = depth_frame_rate;
  config.mode = depth_mode;
  config.saturation = true;

  g_context.requestControl(g_dnode, 0);

  g_dnode.setEnableUvMap(true);
  g_dnode.setEnableVerticesFloatingPoint(true);
  g_dnode.setEnableDepthMapFloatingPoint(true);

  g_dnode.setConfidenceThreshold(confidence_threshold);

  try
  {
    g_context.requestControl(g_dnode, 0);

    g_dnode.setConfiguration(config);
  }
  catch (ArgumentException& e)
  {
    printf("Argument Exception: %s\n", e.what());
  }
  catch (UnauthorizedAccessException& e)
  {
    printf("Unauthorized Access Exception: %s\n", e.what());
  }
  catch (IOException& e)
  {
    printf("IO Exception: %s\n", e.what());
  }
  catch (InvalidOperationException& e)
  {
    printf("Invalid Operation Exception: %s\n", e.what());
  }
  catch (ConfigurationException& e)
  {
    printf("Configuration Exception: %s\n", e.what());
  }
  catch (StreamingException& e)
  {
    printf("Streaming Exception: %s\n", e.what());
  }
  catch (TimeoutException&)
  {
    printf("TimeoutException\n");
  }
}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
  // Connect new color sample handler
  g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

  ColorNode::Configuration config = g_cnode.getConfiguration();

  config.frameFormat = color_frame_format;
  config.compression = color_compression;
  config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
  config.framerate = color_frame_rate;

  g_cnode.setEnableColorMap(true);

  try
  {
    g_context.requestControl(g_cnode, 0);

    g_cnode.setConfiguration(config);
  }
  catch (ArgumentException& e)
  {
    printf("Argument Exception: %s\n", e.what());
  }
  catch (UnauthorizedAccessException& e)
  {
    printf("Unauthorized Access Exception: %s\n", e.what());
  }
  catch (IOException& e)
  {
    printf("IO Exception: %s\n", e.what());
  }
  catch (InvalidOperationException& e)
  {
    printf("Invalid Operation Exception: %s\n", e.what());
  }
  catch (ConfigurationException& e)
  {
    printf("Configuration Exception: %s\n", e.what());
  }
  catch (StreamingException& e)
  {
    printf("Streaming Exception: %s\n", e.what());
  }
  catch (TimeoutException&)
  {
    printf("TimeoutException\n");
  }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
  if ((node.is<DepthNode>()) && (!g_dnode.isSet()) && (depth_enabled))
  {
    g_dnode = node.as<DepthNode>();
    configureDepthNode();
    g_context.registerNode(node);
  }

  if ((node.is<ColorNode>()) && (!g_cnode.isSet()) && (color_enabled))
  {
    g_cnode = node.as<ColorNode>();
    configureColorNode();
    g_context.registerNode(node);
  }

  if ((node.is<AudioNode>()) && (!g_anode.isSet()))
  {
    g_anode = node.as<AudioNode>();
    configureAudioNode();
    g_context.registerNode(node);
  }
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
  configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
  if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
    g_anode.unset();
  if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
    g_cnode.unset();
  if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
    g_dnode.unset();
  printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
  if (!g_bDeviceFound)
  {
    data.device.nodeAddedEvent().connect(&onNodeConnected);
    data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
    g_bDeviceFound = true;
  }
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
  g_bDeviceFound = false;
  printf("Device disconnected\n");
}

void sigintHandler(int sig)
{
  g_context.quit();
  ros::shutdown();
}

void reconfigure_callback(softkinetic_camera::SoftkineticConfig& config, uint32_t level)
{
  // @todo this one isn't trivial
  // camera_link = config.camera_link;

  confidence_threshold = config.confidence_threshold;
  g_dnode.setConfidenceThreshold(confidence_threshold);

  use_voxel_grid_filter = config.use_voxel_grid_filter;
  voxel_grid_size       = config.voxel_grid_size;

  use_radius_outlier_filter = config.use_radius_outlier_filter;
  search_radius             = config.search_radius;
  min_neighbours            = config.min_neighbours;

  use_passthrough_filter = config.use_passthrough_filter;
  limit_min              = config.limit_min;
  limit_max              = config.limit_max;

  use_frustum_culling_filter = config.use_frustum_culling_filter;
  hfov                       = config.hfov;
  vfov                       = config.vfov;
  near_plane                 = config.near_plane;
  far_plane                  = config.far_plane;

  depth_enabled      = config.enable_depth;
  depth_mode         = depthMode(config.depth_mode);
  depth_frame_format = depthFrameFormat(config.depth_frame_format);
  depth_frame_rate   = config.depth_frame_rate;

  color_enabled      = config.enable_color;
  color_compression  = colorCompression(config.color_compression);
  color_frame_format = colorFrameFormat(config.color_frame_format);
  color_frame_rate   = config.color_frame_rate;

  use_serial = config.use_serial;
  serial = config.serial;

  ROS_DEBUG_STREAM("New configuration:\n" <<
          //"camera_link = " << camera_link << "\n" <<

          "confidence_threshold = " << confidence_threshold << "\n" <<

          "use_voxel_grid_filter = " << (use_voxel_grid_filter ? "ON" : "OFF" ) << "\n" <<
          "voxel_grid_size = " << voxel_grid_size << "\n" <<

          "use_radius_outlier_filter = " << (use_radius_outlier_filter ? "ON" : "OFF" ) << "\n" <<
          "search_radius = " << search_radius << "\n" <<
          "min_neighbours = " << min_neighbours << "\n" <<

          "use_passthrough_filter = " << (use_passthrough_filter ? "ON" : "OFF" ) << "\n" <<
          "limit_min = " << limit_min << "\n" <<
          "limit_max = " << limit_max << "\n" <<

          "use_frustum_culling_filter = " << (use_frustum_culling_filter ? "ON" : "OFF" ) << "\n" <<
          "hfov = " << hfov << "\n" <<
          "vfov = " << vfov << "\n" <<
          "near_plane = " << near_plane << "\n" <<
          "far_plane = " << far_plane << "\n" <<

          "enable_depth = " << (depth_enabled ? "ON" : "OFF" ) << "\n" <<
          "depth_mode = " << config.depth_mode << "\n" <<
          "depth_frame_format = " << config.depth_frame_format << "\n" <<
          "depth_frame_rate = " << depth_frame_rate << "\n" <<

          "enable_color = " << (color_enabled ? "ON" : "OFF" ) << "\n" <<
          "color_compression = " << config.color_compression << "\n" <<
          "color_frame_format = " << config.color_frame_format << "\n" <<
          "color_frame_rate = " << color_frame_rate << "\n" <<
          
          "use_serial = " << use_serial << "\n" <<
          "serial = " << serial << "\n");
}

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "softkinetic_bringup_node");
  ros::NodeHandle nh("~");

  // Override the default ROS SIGINT handler to call g_context.quit() and avoid escalating to SIGTERM
  signal(SIGINT, sigintHandler);

  // Get frame id from parameter server
  if (!nh.hasParam("camera_link"))
  {
    ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'camera_link' is missing.");
    ros_node_shutdown = true;
  }


  // Fill in the color and depth images message header frame id
  std::string optical_frame;
  if (nh.getParam("rgb_optical_frame", optical_frame))
  {
    cloud.header.frame_id = optical_frame.c_str();
    img_rgb.header.frame_id = optical_frame.c_str();
    img_mono.header.frame_id = optical_frame.c_str();
  }
  else
  {
    img_rgb.header.frame_id = "/softkinetic_rgb_optical_frame";
    img_mono.header.frame_id = "/softkinetic_rgb_optical_frame";
  }

  if (nh.getParam("depth_optical_frame", optical_frame))
  {
    img_depth.header.frame_id = optical_frame.c_str();
  }
  else
  {
    img_depth.header.frame_id = "/softkinetic_depth_optical_frame";
  }

  // Get confidence threshold from parameter server
  if (!nh.hasParam("confidence_threshold"))
  {
    ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'confidence_threshold' is not set on server.");
    ros_node_shutdown = true;
  }
  nh.param<int>("confidence_threshold", confidence_threshold, 150);

  // Check for usage of voxel grid filtering to downsample the point cloud
  nh.param<bool>("use_voxel_grid_filter", use_voxel_grid_filter, false);
  if (use_voxel_grid_filter)
  {
    // Downsampling cloud parameters
    nh.param<double>("voxel_grid_size", voxel_grid_size, 0.01);
  }

  // Check for usage of radius filtering
  nh.param<bool>("use_radius_outlier_filter", use_radius_outlier_filter, false);
  if (use_radius_outlier_filter)
  {
    if (!nh.hasParam("search_radius"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'search_radius' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("search_radius", search_radius, 0.5);

    if (!nh.hasParam("min_neighbours"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'min_neighbours' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<int>("min_neighbours", min_neighbours, 0);
  }

  // Check for usage of passthrough filtering
  nh.param<bool>("use_passthrough_filter", use_passthrough_filter, false);
  if(use_passthrough_filter)
  {
    if (!nh.hasParam("limit_min"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'limit_min' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("limit_min", limit_min, 0.0);

    if (!nh.hasParam("limit_max"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'limit_max' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("limit_max", limit_max, 0.0);
  }

  // Check for usage of frustum culling filtering
  nh.param<bool>("use_frustum_culling_filter", use_frustum_culling_filter, false);
  if(use_frustum_culling_filter)
  {
    if(!nh.hasParam("hfov"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'hfov' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("hfov", hfov, 180.0);

    if(!nh.hasParam("vfov"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'vfov' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("vfov", vfov, 180.0);

    if(!nh.hasParam("near_plane"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'near_plane' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("near_plane", near_plane, 0.0);

    if(!nh.hasParam("far_plane"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'far_plane' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("far_plane", far_plane, 100.0);
  }

  nh.param<bool>("enable_depth", depth_enabled, true);
  std::string depth_mode_str;
  nh.param<std::string>("depth_mode", depth_mode_str, "close");
  depth_mode = depthMode(depth_mode_str);

  std::string depth_frame_format_str;
  nh.param<std::string>("depth_frame_format", depth_frame_format_str, "QVGA");
  depth_frame_format = depthFrameFormat(depth_frame_format_str);

  nh.param<int>("depth_frame_rate", depth_frame_rate, 25);

  nh.param<bool>("enable_color", color_enabled, true);
  std::string color_compression_str;
  nh.param<std::string>("color_compression", color_compression_str, "MJPEG");
  color_compression = colorCompression(color_compression_str);

  std::string color_frame_format_str;
  nh.param<std::string>("color_frame_format", color_frame_format_str, "WXGA_H");
  color_frame_format = colorFrameFormat(color_frame_format_str);

  nh.param<int>("color_frame_rate", color_frame_rate, 25);

  // Initialize image transport object
  image_transport::ImageTransport it(nh);

  // Initialize publishers
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("depth/points", 1);
  pub_rgb = it.advertise("rgb/image_color", 1);
  pub_mono = it.advertise("rgb/image_mono", 1);
  pub_depth = it.advertise("depth/image_raw", 1);
  pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
  pub_rgb_info = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);

  std::string calibration_file;
  if (nh.getParam("rgb_calibration_file", calibration_file))
  {
    camera_info_manager::CameraInfoManager camera_info_manager(nh, "senz3d", "file://" + calibration_file);
    rgb_info = camera_info_manager.getCameraInfo();
  }

  if (nh.getParam("depth_calibration_file", calibration_file))
  {
    camera_info_manager::CameraInfoManager camera_info_manager(nh, "senz3d", "file://" + calibration_file);
    depth_info = camera_info_manager.getCameraInfo();
  }

  g_context = Context::create("softkinetic");

  g_context.deviceAddedEvent().connect(&onDeviceConnected);
  g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

  // Get the list of currently connected devices
  vector<Device> da = g_context.getDevices();

  // In case there are several devices, index of camera to start ought to come as an argument
  // By default, index 0 is taken:
  int device_index = 0;
  ROS_INFO_STREAM("Number of Devices found: " << da.size());
  if (da.size() == 0)
  {
    ROS_ERROR_STREAM("No devices found!!!!!!");
  }

  if (da.size() >= 1)
  {
    // If camera index comes as argument, device_index will be updated
    if (argc > 1)
    {
      device_index = atoi(argv[1]);
    }
    else
    {
      nh.param<bool>("use_serial", use_serial, false);
      if (use_serial)
      {
        nh.getParam("serial", serial);
        bool serialDeviceFound = false;
        for (int i=0; i < da.size(); i++)
        {
          if (!da[i].getSerialNumber().compare(serial))
          {
            device_index = i;
            serialDeviceFound = true;
          }
        }
        if (serialDeviceFound)
        {
          ROS_INFO_STREAM("Serial number device found");
        }
        else
        {
          ROS_ERROR_STREAM("Serial number device not found !!");
          ROS_ERROR_STREAM("Required Serial Number: " << serial);
        }
      }
    }
    ROS_INFO_STREAM("Serial Number: " << da[device_index].getSerialNumber());

    g_bDeviceFound = true;

    da[device_index].nodeAddedEvent().connect(&onNodeConnected);
    da[device_index].nodeRemovedEvent().connect(&onNodeDisconnected);

    vector<Node> na = da[device_index].getNodes();

    for (int n = 0; n < (int)na.size(); n++)
      configureNode(na[n]);
  }

  // Enable dynamic reconfigure
  ros::NodeHandle nh_cfg("~");
  ros::CallbackQueue callback_queue_cfg;
  nh_cfg.setCallbackQueue(&callback_queue_cfg);

  dynamic_reconfigure::Server<softkinetic_camera::SoftkineticConfig> server(nh_cfg);
  server.setCallback(boost::bind(&reconfigure_callback, _1, _2));

  // Handle the dynamic reconfigure server callback on a
  // separate thread from the g_context.run() called below
  ros::AsyncSpinner spinner(1, &callback_queue_cfg);
  spinner.start();

  // Loop while ros core is operational or Ctrl-C is used
  if (ros_node_shutdown)
  {
    ros::shutdown();
  }
  while (ros::ok())
  {
    g_context.startNodes();
    g_context.run();
  }

  // Close out all nodes
  if (g_cnode.isSet())
    g_context.unregisterNode(g_cnode);
  if (g_dnode.isSet())
    g_context.unregisterNode(g_dnode);
  if (g_anode.isSet())
    g_context.unregisterNode(g_anode);

  g_context.stopNodes();

  return 0;
}
