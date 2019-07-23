#ifndef   PLANE_REMOVAL_HPP
#define   PLANE_REMOVAL_HPP
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/ModelCoefficients.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class plane_removal{
    private:
        ros::NodeHandle nh_;
        double threshould_;
        int  max_iterations_;
        double probability_;
        ros::Subscriber pcl_sub_;
        ros::Publisher pcl_pub_;
        void point_cloud_CB_(const sensor_msgs::PointCloud2ConstPtr&);
        
    public:
        plane_removal();
};
#endif//PLANE_REMOVAL_HPP