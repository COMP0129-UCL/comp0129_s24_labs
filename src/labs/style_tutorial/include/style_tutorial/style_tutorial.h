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

#ifndef STYLE_TUTORIAL_H_
#define STYLE_TUTORIAL_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/** \brief Style Tutorial.
  *
  * \author Dimitrios Kanoulas
  */
class StyleTutorial
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    StyleTutorial (ros::NodeHandle &nh);

    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_input a PCLPointCloud2 const pointer
      */
    void
    cloud_cb_one (const pcl::PCLPointCloud2ConstPtr& cloud_input);

    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    cloud_cb_two (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    PubFilteredPC (ros::Publisher &pc_pub, pcl::PCLPointCloud2 &pc);

    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    PubFilteredPCmsg (ros::Publisher &pc_pub, sensor_msgs::PointCloud2 &pc);
    
  public:
    /** \brief Node handle. */
    ros::NodeHandle nh_;

    /** \brief ROS publishers. */
    ros::Publisher pub_one, pub_two;
  
  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif