/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2019-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 */

#include <style_tutorial/style_tutorial.h>

////////////////////////////////////////////////////////////////////////////////
StyleTutorial::StyleTutorial (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;

  // Define the publishers
  pub_one = nh.advertise<sensor_msgs::PointCloud2> ("test_output_one", 1, true);
  pub_two = nh.advertise<sensor_msgs::PointCloud2> ("test_output_two", 1, true);
}

////////////////////////////////////////////////////////////////////////////////
void
StyleTutorial::cloud_cb_one (const pcl::PCLPointCloud2ConstPtr& cloud_input)
{
  // Create container variables
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (cloud_input);
  vg.setLeafSize (0.1, 0.1, 0.1);
  vg.filter (cloud_filtered);
  
  // Publish the data
  ROS_INFO ("Publishing Filtered Cloud 1");
  PubFilteredPC (pub_one, cloud_filtered);
}

////////////////////////////////////////////////////////////////////////////////
void
StyleTutorial::cloud_cb_two (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr (cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_output;
  pcl_conversions::moveFromPCL (cloud_filtered, cloud_output);

  // Publish the data
  ROS_INFO ("Publishing Filtered Cloud 2");
  PubFilteredPCmsg (pub_two, cloud_output);
}

////////////////////////////////////////////////////////////////////////////////
void
StyleTutorial::PubFilteredPC (ros::Publisher &pc_pub, pcl::PCLPointCloud2 &pc)
{
  // Publish the data
  pc_pub.publish (pc);
}

////////////////////////////////////////////////////////////////////////////////
void
StyleTutorial::PubFilteredPCmsg (ros::Publisher &pc_pub,
                               sensor_msgs::PointCloud2 &pc)
{
  // Publish the data
  pc_pub.publish (pc);
}