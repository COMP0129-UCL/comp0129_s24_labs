#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h> // For pcl::search::KdTree search used in Region Growing RGB
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud

#include "pcl/segmentation/region_growing_rgb.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h> 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "std_msgs/String.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


void get_colored_cloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud);
void cloud_callback(const  boost::shared_ptr<const sensor_msgs::PointCloud2>& msg);
