#include <pcl_demo/pcl_demo.h>

pcl::visualization::CloudViewer viewer ("Cluster viewer");

void cloud_callback(const  boost::shared_ptr<const sensor_msgs::PointCloud2>& msg){
  // ROS_INFO("PointCloud2 message received");
  // convert from ros' sensor_msg/PointCloud2 to pcl/PointCloud2
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(*msg, cloud);

  // convert pcl/PointCloud2 to PointCloud vector of PointXYZRGB 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(cloud,*temp_cloud);

  // visualize PointCloud
  viewer.showCloud(temp_cloud);
  return;
}
