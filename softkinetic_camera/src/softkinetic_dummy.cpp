#include <ros/ros.h>

int main(int argc, char* argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "softkinetic_bringup_node");
  ros::NodeHandle nh;

  ros::Rate r(0.1);
  while(ros::ok())
  {
    ROS_ERROR(
      "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
      "\n++++++++++++ Could not locate the DepthSense SDK +++++++++++"
      "\nPlease install the SDK, create an softkinectic overlay"
      "\nand recompile the softkinetic_camera package, see README.txt."
      "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

    r.sleep();
  }
  return 0;
}