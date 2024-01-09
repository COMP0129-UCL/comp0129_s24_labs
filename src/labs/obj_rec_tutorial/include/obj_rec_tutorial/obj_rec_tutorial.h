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

#ifndef OBJ_REC_TUTORIAL_H_
#define OBJ_REC_TUTORIAL_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>

// ROS includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

/** \brief Object Recognition Tutorial.
  *
  * \author Dimitrios Kanoulas
  */
class ObjRecTutorial
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    ObjRecTutorial (ros::NodeHandle &nh);
    
    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    cloudCallBackOne (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
    
    /** \brief Apply Voxel Grid filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyVX (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);

    /** \brief Apply Pass Through filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyPT (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);
    
    /** \brief Normal estimation.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findNormals (PointCPtr &in_cloud_ptr);
    
    /** \brief Segment Plane from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segPlane (PointCPtr &in_cloud_ptr);
    
    /** \brief Segment Cylinder from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segCylind (PointCPtr &in_cloud_ptr);
    
    
    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findCylPose (PointCPtr &in_cloud_ptr);
    
    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);
    
    /** \brief Publish the cylinder point.
      * 
      *  \input[in] cyl_pt_msg Cylinder's geometry point
      *  
      *  \output true if three numbers are added
      */
    void
    publishPose (geometry_msgs::PointStamped &cyl_pt_msg);
    
  public:
    /** \brief Node handle. */
    ros::NodeHandle g_nh;
    
    /** \brief The input point cloud frame id. */
    std::string g_input_pc_frame_id_;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_cloud;
    
    /** \brief ROS geometry message point. */
    geometry_msgs::PointStamped g_cyl_pt_msg;
    
    /** \brief ROS pose publishers. */
    ros::Publisher g_pub_pose;
    
    /** \brief Voxel Grid filter's leaf size. */
    double g_vg_leaf_sz;
    
    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered, g_cloud_filtered2;
    
    /** \brief Point Cloud (filtered) sensros_msg for publ. */
    sensor_msgs::PointCloud2 g_cloud_filtered_msg;
    
    /** \brief Point Cloud (input). */
    pcl::PCLPointCloud2 g_pcl_pc;
    
    /** \brief Voxel Grid filter. */
    pcl::VoxelGrid<PointT> g_vx;
    
    /** \brief Pass Through filter. */
    pcl::PassThrough<PointT> g_pt;
    
    /** \brief Pass Through min and max threshold sizes. */
    double g_pt_thrs_min, g_pt_thrs_max;
    
    /** \brief KDTree for nearest neighborhood search. */
    pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
    
    /** \brief Normal estimation. */
    pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    
    /** \brief Cloud of normals. */
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
    
    /** \brief Nearest neighborhooh size for normal estimation. */
    double g_k_nn;
    
    /** \brief SAC segmentation. */
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
    
    /** \brief Extract point cloud indices. */
    pcl::ExtractIndices<PointT> g_extract_pc;
  
    /** \brief Extract point cloud normal indices. */
    pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    
    /** \brief Point indices for plane. */
    pcl::PointIndices::Ptr g_inliers_plane;
      
    /** \brief Point indices for cylinder. */
    pcl::PointIndices::Ptr g_inliers_cylinder;
    
    /** \brief Model coefficients for the plane segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_plane;
    
    /** \brief Model coefficients for the culinder segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_cylinder;
    
    /** \brief Point cloud to hold plane and cylinder points. */
    PointCPtr g_cloud_plane, g_cloud_cylinder;
    
    /** \brief cw1Q1: TF listener definition. */
    tf::TransformListener g_listener_;
    
  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif
