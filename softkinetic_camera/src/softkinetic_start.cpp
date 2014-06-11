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
#include <vector>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>

//ros include files
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
//#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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
StereoCameraParameters g_scp;

ros::Publisher pub_cloud;
image_transport::Publisher pub_rgb;

sensor_msgs::Image image;
std_msgs::Int32 test_int;
sensor_msgs::PointCloud2 cloud;

int offset;
/* confidence threshold for DepthNode configuration*/
int confidence_threshold;
/* parameters for downsampling cloud */
double voxel_grid_side;
/* parameters for radius filter */
bool use_radius_filter;
double search_radius;
int minNeighboursInRadius;
/* shutdown request*/
bool ros_node_shutdown = false;

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
    ros::NodeHandle n_color("~");
    std::string softkinetic_link;
    if (n_color.getParam("/camera_link", softkinetic_link))
    {
        image.header.frame_id = softkinetic_link.c_str();
    }
    else
    {
        image.header.frame_id = "/softkinetic_link";
    }
    //Create a sensor_msg::Image for ROS based on the new camera image
    //image.header.stamp.nsec = g_cFrames++*1000;
    image.header.stamp = ros::Time::now();
    int count = 0;

    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);
    image.width = w;///2;
    image.height = h;///2;
    image.encoding = "bgr8";
    image.data.resize(w*h*3);
    int count2 = w*h*3-1;

    for(int i = 0;i < w;i++){
        for(int j = 0;j < h; j++){
            image.data[count2]   = data.colorMap[count2];
            image.data[count2+1] = data.colorMap[count2+1];
            image.data[count2+2] = data.colorMap[count2+2];
            count2-=3;
        }
    }

    // Publish the rgb data
    pub_rgb.publish(image);

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
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud(cloud_to_filter);
    ror.setRadiusSearch(search_radius);
    ror.setMinNeighborsInRadius(minNeighboursInRadius);
    // apply filter
    ROS_DEBUG_STREAM("Starting filtering");
    int before = cloud_to_filter->size();
    double old_ = ros::Time::now().toSec();
    ror.filter(*cloud_to_filter);
    double new_= ros::Time::now().toSec() - old_;
    int after = cloud_to_filter->size();
    ROS_DEBUG_STREAM("filtered in " << new_ << " seconds;"
                  << "points reduced from " << before << " to " << after);
    cloud.header.stamp = ros::Time::now();
}

// New depth sample event varsace tieshandler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    int count = -1;

    // Project some 3D points in the Color Frame
    if (!g_pProjHelper)
    {
        g_pProjHelper = new ProjectionHelper(data.stereoCameraParameters);
        g_scp = data.stereoCameraParameters;
    }
    else if (g_scp != data.stereoCameraParameters)
    {
        g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
        g_scp = data.stereoCameraParameters;
    }

    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);
    int cx = w/2;
    int cy = h/2;

    Vertex p3DPoints[1];
    Point2D p2DPoints[1];

    g_dFrames++;

    current_cloud->header.frame_id = cloud.header.frame_id;
    current_cloud->height = h;
    current_cloud->width = w;
    current_cloud->is_dense = false;
    current_cloud->points.resize(w*h);

    uchar b,g,r;
    uint32_t rgb;
    cv::Vec3b bgr;

    for(int i = 1;i < h ;i++){
        for(int j = 1;j < w ; j++){
            count++;
            current_cloud->points[count].x = -data.verticesFloatingPoint[count].x;
            current_cloud->points[count].y = data.verticesFloatingPoint[count].y;
            if(data.verticesFloatingPoint[count].z == 32001){
                current_cloud->points[count].z = 0;
            }else{
                current_cloud->points[count].z = data.verticesFloatingPoint[count].z;
            }
            p3DPoints[0] = data.vertices[count];
            g_pProjHelper->get2DCoordinates ( p3DPoints, p2DPoints, 1, CAMERA_PLANE_COLOR);
            int x_pos = (int)p2DPoints[0].x;
            int y_pos = (int)p2DPoints[0].y;
        }
    }

    //check for usage of radius filtering
    if(use_radius_filter)
    {
        //we must downsample the cloud so the filter don't take too long
        downsampleCloud(current_cloud);
        filterCloudRadiusBased(current_cloud);
    }

    //convert current_cloud to PointCloud2 and publish
    pcl::toROSMsg(*current_cloud, cloud);

    pub_cloud.publish (cloud);
    g_context.quit();
}

/*----------------------------------------------------------------------------*/
void configureAudioNode()
{
    g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

    AudioNode::Configuration config = g_anode.getConfiguration();
    config.sampleRate = 44100;

    try
    {
        g_context.requestControl(g_anode,0);

        g_anode.setConfiguration(config);

        g_anode.setInputMixerLevel(0.5f);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
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
    config.frameFormat = FRAME_FORMAT_QVGA;
    config.framerate = 25;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    g_context.requestControl(g_dnode,0);

    g_dnode.setEnableVertices(true);
    g_dnode.setEnableConfidenceMap(true);
    g_dnode.setConfidenceThreshold(confidence_threshold);
    g_dnode.setEnableVerticesFloatingPoint(true);
    g_dnode.setEnableDepthMapFloatingPoint(true);

    try
    {
        g_context.requestControl(g_dnode,0);

        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
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
    config.frameFormat = FRAME_FORMAT_WXGA_H;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = 25;

    g_cnode.setEnableColorMap(true);

    try
    {
        g_context.requestControl(g_cnode,0);

        g_cnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }

    if ((node.is<AudioNode>())&&(!g_anode.isSet()))
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

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
    //initialize ros
    ros::init (argc, argv, "softkinetic_bringup_node");
    ros::NodeHandle nh("~");

    // get frame id from parameter server
    std::string softkinetic_link;
    if(!nh.hasParam("camera_link"))
    {
        ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'camera_link' is missing.");
        ros_node_shutdown = true;
    };

    nh.param<std::string>("camera_link", softkinetic_link, "softkinetic_link");
    cloud.header.frame_id = softkinetic_link;

    // get confidence threshold from parameter server
    if(!nh.hasParam("confidence_threshold"))
    {
        ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'confidence_threshold' is not set on server.");
        ros_node_shutdown = true;
    };
    nh.param<int>("confidence_threshold", confidence_threshold, 150);

    // check for usage of radius filtering
    nh.param<bool>("use_radius_filter", use_radius_filter, false);
    if(use_radius_filter)
    {
        if(!nh.hasParam("search_radius"))
        {
            ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'search_radius' is not set on server.");
            ros_node_shutdown = true;
        };
        nh.param<double>("search_radius", search_radius, 0.5);

        if(!nh.hasParam("minNeighboursInRadius"))
        {
            ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'minNeighboursInRadius' is not set on server.");
            ros_node_shutdown = true;
        };
        nh.param<int>("minNeighboursInRadius", minNeighboursInRadius, 0);

        // downsampling cloud parameters
        nh.param<double>("voxel_grid_side", voxel_grid_side, 0.01);
    };

    offset = ros::Time::now().toSec();
    //initialize image transport object
    image_transport::ImageTransport it(nh);

    //initialize publishers
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("depth_registered/points", 1);
    pub_rgb = it.advertise ("rgb_data", 1);

    g_context = Context::create("softkinetic");

    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    // Get the list of currently connected devices
    vector<Device> da = g_context.getDevices();

    // In case there are several devices, index of camera to start ought to come as an argument
    // By default, index 0 is taken:
    int device_index = 0;
    ROS_INFO_STREAM("Number of Devices found: " << da.size());
    if(da.size() == 0)
    {
        ROS_ERROR_STREAM("No devices found!!!!!!");
    };
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

        for (int n = 0; n < (int)na.size();n++)
            configureNode(na[n]);
    }
    //loop while ros core is operational or Ctrl-C is used
    if(ros_node_shutdown){
        ros::shutdown();
    }
    while(ros::ok()){
        g_context.startNodes();
        g_context.run();
    }
    //Close out all nodes
    if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
    if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
    if (g_anode.isSet()) g_context.unregisterNode(g_anode);

    if (g_pProjHelper)
        delete g_pProjHelper;
    g_context.stopNodes();
    return 0;
}
