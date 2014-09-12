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

// ros include files
#include <ros/ros.h>
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

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

ProjectionHelper* g_pProjHelper = NULL;

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

cv::Mat cv_img_rgb; //CV image containers
cv::Mat cv_img_mono;
cv::Mat cv_img_depth;

std_msgs::Int32 test_int;

/* confidence threshold for DepthNode configuration*/
int confidence_threshold;

/* parameters for downsampling cloud */
bool use_voxel_grid_filter;
double voxel_grid_side;

/* parameters for radius filter */
bool use_radius_filter;
double search_radius;
int min_neighbours;

/* shutdown request*/
bool ros_node_shutdown = false;

/* depth sensor parameters */
bool depth_enabled;
DepthSense::DepthNode::CameraMode depth_mode;
DepthSense::FrameFormat depth_frame_format;
int depth_frame_rate;

/* color sensor parameters */
bool color_enabled;
DepthSense::CompressionType color_compression;
DepthSense::FrameFormat color_frame_format;
int color_frame_rate;


/*----------------------------------------------------------------------------*/
// New audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
  //printf("A#%u: %d\n",g_aFrames,data.audioData.size());
  g_aFrames++;
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
  // If this is the first sample, we must fill all the constant values on image and
  // camera info messages to increase working rate
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
  }

  cv_img_rgb.data = (uchar *)(const uint8_t *)data.colorMap;
  cvtColor(cv_img_rgb, cv_img_mono, CV_RGB2GRAY);

  for (int i = 0; i < img_rgb.height; i++)
  {
    for (int j = 0; j < img_rgb.width; j++)
    {
      int idx = i * img_rgb.width + j;
      img_mono.data[idx]  = cv_img_mono.at<uchar>(i, j);

      idx *= 3;
      img_rgb.data[idx++] = cv_img_rgb.at<cv::Vec3b>(i, j)[0];
      img_rgb.data[idx++] = cv_img_rgb.at<cv::Vec3b>(i, j)[1];
      img_rgb.data[idx]   = cv_img_rgb.at<cv::Vec3b>(i, j)[2];
    }
  }

  img_rgb.header.stamp = ros::Time::now();
  img_mono.header      = img_rgb.header;
  rgb_info.header      = img_rgb.header;

  // Publish the rgb and mono images and camera info
  pub_rgb.publish(img_rgb);
  pub_mono.publish(img_mono);

  pub_rgb_info.publish(rgb_info);

  g_cFrames++;
}

/*----------------------------------------------------------------------------*/

void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filter)
{
  ROS_DEBUG_STREAM("Starting downsampling");
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_to_filter);
  sor.setLeafSize (voxel_grid_side, voxel_grid_side, voxel_grid_side);
  sor.filter (*cloud_to_filter);
  ROS_DEBUG_STREAM("downsampled!");
}

void filterCloudRadiusBased(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_filter)
{
  // radius based filter:
  pcl::RadiusOutlierRemoval < pcl::PointXYZRGB > ror;
  ror.setInputCloud(cloud_to_filter);
  ror.setRadiusSearch(search_radius);
  ror.setMinNeighborsInRadius(min_neighbours);
  // apply filter
  ROS_DEBUG_STREAM("Starting filtering");
  int before = cloud_to_filter->size();
  double old_ = ros::Time::now().toSec();
  ror.filter(*cloud_to_filter);
  double new_ = ros::Time::now().toSec() - old_;
  int after = cloud_to_filter->size();
  ROS_DEBUG_STREAM("filtered in " << new_ << " seconds;" << "points reduced from " << before << " to " << after);
  cloud.header.stamp = ros::Time::now();
}

void setupCameraInfo(const DepthSense::IntrinsicParameters& params, sensor_msgs::CameraInfo& cam_info)
{
  depth_info.distortion_model = "plumb_bob";
  depth_info.height = params.height;
  depth_info.width  = params.width;

  // Distortion parameters D = [k1, k2, t1, t2, k3]
  depth_info.D.resize(5);
  depth_info.D[0] = params.k1;
  depth_info.D[1] = params.k2;
  depth_info.D[2] = params.p1;
  depth_info.D[3] = params.p2;
  depth_info.D[4] = params.k3;

  // Intrinsic camera matrix for the raw (distorted) images:
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  depth_info.K[0] = params.fx;
  depth_info.K[2] = params.cx;
  depth_info.K[4] = params.fy;
  depth_info.K[5] = params.cy;
  depth_info.K[8] = 1.0;

  // Rectification matrix (stereo cameras only)
  //     [1 0 0]
  // R = [0 1 0]
  //     [0 0 1]
  depth_info.R[0] = 1.0;
  depth_info.R[4] = 1.0;
  depth_info.R[8] = 1.0;

  // Projection/camera matrix; we use the same values as in the raw image, as we are not
  // applying any correction (WARN: is this ok?). For monocular cameras, Tx = Ty = 0.
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  depth_info.P[0] = params.fx;
  depth_info.P[2] = params.cx;
  depth_info.P[5] = params.fy;
  depth_info.P[6] = params.cy;
  depth_info.P[10] = 1.0;
}

// New depth sample event varsace tieshandler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
  if (img_depth.data.size() == 0)
  {
    // Project some 3D points in the Color Frame
    if (!g_pProjHelper)
      g_pProjHelper = new ProjectionHelper(data.stereoCameraParameters);

    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);

    img_depth.width = w;
    img_depth.height = h;
    img_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    img_depth.is_bigendian = 0;
    img_depth.step = sizeof(float) * w;
    std::size_t data_size = img_depth.width * img_depth.height;
    img_depth.data.resize(data_size * sizeof(float));

    cv_img_depth.create(h, w, CV_32FC1); // unused by now; not sure if I can use it to improve speed

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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  current_cloud->header.frame_id = cloud.header.frame_id;
  current_cloud->height = img_depth.height;
  current_cloud->width  = img_depth.width;
  current_cloud->is_dense = false;
  current_cloud->points.resize(img_depth.width * img_depth.height);

  g_dFrames++;

  Vertex p3DPoints[1];
  Point2D p2DPoints[1];

  int count = -1;
  float* depth_img_ptr = reinterpret_cast<float*>(&img_depth.data[0]);

  for (int i = 0; i < img_depth.height; i++)
  {
    for (int j = 0; j < img_depth.width; j++)
    {
      count++;
      current_cloud->points[count].x = -data.verticesFloatingPoint[count].x;
      current_cloud->points[count].y =  data.verticesFloatingPoint[count].y;
      if (data.verticesFloatingPoint[count].z == 32001)
      {
        current_cloud->points[count].z = 0;
      }
      else
      {
        current_cloud->points[count].z = data.verticesFloatingPoint[count].z;
      }

      // Saturated pixels on depthMapFloatingPoint have -1 value, but on openni are NaN
      *depth_img_ptr =
          data.depthMapFloatingPoint[count] < 0.0 ? std::numeric_limits<float>::quiet_NaN() :
          data.depthMapFloatingPoint[count];
      ++depth_img_ptr;

      // get mapping between depth map and color map, assuming we have a RGB image
      if (img_rgb.data.size() == 0)
      {
        ROS_WARN_THROTTLE(2.0, "Color image is empty; pointcloud will be colorless");
        continue;
      }
      p3DPoints[0] = data.vertices[count];
      g_pProjHelper->get2DCoordinates(p3DPoints, p2DPoints, 2, CAMERA_PLANE_COLOR);
      int x_pos = (int)p2DPoints[0].x;
      int y_pos = (int)p2DPoints[0].y;

      if (y_pos < 0 || y_pos > img_rgb.height || x_pos < 0 || x_pos > img_rgb.width)
      {
        current_cloud->points[count].b = 0;
        current_cloud->points[count].g = 0;
        current_cloud->points[count].r = 0;
      }
      else
      {
        current_cloud->points[count].b = cv_img_rgb.at<cv::Vec3b>(y_pos, x_pos)[0];
        current_cloud->points[count].g = cv_img_rgb.at<cv::Vec3b>(y_pos, x_pos)[1];
        current_cloud->points[count].r = cv_img_rgb.at<cv::Vec3b>(y_pos, x_pos)[2];
      }
    }
  }

  // check for usage of voxel grid filtering to downsample point cloud
  if (use_voxel_grid_filter)
  {
    downsampleCloud(current_cloud);
  }

  // check for usage of radius filtering
  if (use_radius_filter)
  {
    // use_voxel_grid_filter should be enabled so that the radius filter doesn't take too long
    filterCloudRadiusBased(current_cloud);
  }

  // convert current_cloud to PointCloud2 and publish
  pcl::toROSMsg(*current_cloud, cloud);

  img_depth.header.stamp = ros::Time::now();
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

  g_dnode.setEnableVertices(true);
  g_dnode.setEnableConfidenceMap(true);
  g_dnode.setConfidenceThreshold(confidence_threshold);
  g_dnode.setEnableVerticesFloatingPoint(true);
  g_dnode.setEnableDepthMapFloatingPoint(true);

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
  // connect new color sample handler
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

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
  // initialize ros
  ros::init(argc, argv, "softkinetic_bringup_node");
  ros::NodeHandle nh("~");

  // Override the default ROS SIGINT handler to call g_context.quit() and avoid escalating to SIGTERM
  signal(SIGINT, sigintHandler);

  // get frame id from parameter server
  std::string softkinetic_link;
  if (!nh.hasParam("camera_link"))
  {
    ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'camera_link' is missing.");
    ros_node_shutdown = true;
  }

  nh.param<std::string>("camera_link", softkinetic_link, "softkinetic_link");
  cloud.header.frame_id = softkinetic_link;

  //fill in the rgb and depth images message header frame id
  std::string optical_frame;
  if (nh.getParam("rgb_optical_frame", optical_frame))
  {
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

  // get confidence threshold from parameter server
  if (!nh.hasParam("confidence_threshold"))
  {
    ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'confidence_threshold' is not set on server.");
    ros_node_shutdown = true;
  }
  nh.param<int>("confidence_threshold", confidence_threshold, 150);

  // check for usage of voxel grid filtering to downsample the point cloud
  nh.param<bool>("use_voxel_grid_filter", use_voxel_grid_filter, false);
  if (use_voxel_grid_filter)
  {
    // downsampling cloud parameters
    nh.param<double>("voxel_grid_side", voxel_grid_side, 0.01);
  }

  // check for usage of radius filtering
  nh.param<bool>("use_radius_filter", use_radius_filter, false);
  if (use_radius_filter)
  {
    if (!nh.hasParam("search_radius"))
    {
      ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'search_radius' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<double>("search_radius", search_radius, 0.5);

    if (!nh.hasParam("min_neighbours"))
    {
      ROS_ERROR_STREAM(
          "For " << ros::this_node::getName() << ", parameter 'min_neighbours' is not set on server.");
      ros_node_shutdown = true;
    }
    nh.param<int>("min_neighbours", min_neighbours, 0);
  }

  std::string depth_mode_str;
  nh.param<std::string>("depth_mode", depth_mode_str, "close");
  if (depth_mode_str == "long")
    depth_mode = DepthNode::CAMERA_MODE_LONG_RANGE;
  else
    depth_mode = DepthNode::CAMERA_MODE_CLOSE_MODE;

  std::string depth_frame_format_str;
  nh.param<std::string>("depth_frame_format", depth_frame_format_str, "QVGA");
  if (depth_frame_format_str == "QQVGA")
    depth_frame_format = FRAME_FORMAT_QQVGA;
  else if (depth_frame_format_str == "QVGA")
    depth_frame_format = FRAME_FORMAT_QVGA;
  else
    depth_frame_format = FRAME_FORMAT_VGA;

  nh.param<bool>("enable_depth", depth_enabled, true);
  nh.param<int>("depth_frame_rate", depth_frame_rate, 25);

  std::string color_compression_str;
  nh.param<std::string>("color_compression", color_compression_str, "MJPEG");
  if (color_compression_str == "YUY2")
    color_compression = COMPRESSION_TYPE_YUY2;
  else
    color_compression = COMPRESSION_TYPE_MJPEG;

  std::string color_frame_format_str;
  nh.param<std::string>("color_frame_format", color_frame_format_str, "WXGA");
  if (color_frame_format_str == "QQVGA")
    color_frame_format = FRAME_FORMAT_QQVGA;
  else if (color_frame_format_str == "QVGA")
    color_frame_format = FRAME_FORMAT_QVGA;
  else if (color_frame_format_str == "VGA")
    color_frame_format = FRAME_FORMAT_VGA;
  else if (color_frame_format_str == "NHD")
    color_frame_format = FRAME_FORMAT_NHD;
  else
    color_frame_format = FRAME_FORMAT_WXGA_H;

  nh.param<bool>("enable_color", color_enabled, true);
  nh.param<int>("color_frame_rate", color_frame_rate, 25);

  // initialize image transport object
  image_transport::ImageTransport it(nh);

  // initialize publishers
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
    // if camera index comes as argument, device_index will be updated
    if (argc > 1)
    {
      device_index = atoi(argv[1]);
    }

    g_bDeviceFound = true;

    da[device_index].nodeAddedEvent().connect(&onNodeConnected);
    da[device_index].nodeRemovedEvent().connect(&onNodeDisconnected);

    vector<Node> na = da[device_index].getNodes();

    for (int n = 0; n < (int)na.size(); n++)
      configureNode(na[n]);
  }
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

  if (g_pProjHelper)
    delete g_pProjHelper;
  g_context.stopNodes();

  return 0;
}
