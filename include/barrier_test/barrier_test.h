/* *****************************************************************
*
* barrier_test_node
*
******************************************************************/

/**
* @file   %FILENAME%
* @author %USER% (%$EMAIL%)
* @date   %DATE%
*
* @brief  Filedescription
*/

#ifndef BARRIER_TEST_NODE_H
#define BARRIER_TEST_NODE_H

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>

class barrier_test_node
{
public:
  barrier_test_node(ros::NodeHandle &node_handle);

private:
  // node handle
  ros::NodeHandle *node_;

  // ros communication
  ros::Subscriber barrier_test_sub_;

  ros::Publisher barrier_test_pub_;

  ros::Publisher test_pub_;


  // parameters
  //double my_double_parameter_;
  //std::string my_string_parameter_;
  //int my_int_parameter_;
  //bool my_bool_parameter_;

  // callbacks
  //void subscriberCallback(const sensor_msgs::Joy &msg);
  void barrier_testCallback(const sensor_msgs::ImageConstPtr &msg);
  // void heightThresholder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutCloud,
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr processCloud, double zMin, double zMax);
    //bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    //void timerCallback(const ros::TimerEvent &evt);
  };

  #endif // polls3d_ming_node_H
