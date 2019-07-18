#include "plane_detect.hpp"
plane_detect::plane_detect(){
    ros::NodeHandle n_arg("~");
    threshould_ = 0.1;
    n_arg.getParam("threshould", threshould_);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detected_point_cloud", 1);
    pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &plane_detect::point_cloud_CB_,this);
}
void plane_detect::point_cloud_CB_(const sensor_msgs::PointCloud2ConstPtr& cb_cloud){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cb_cloud,cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;  

    seg.setOptimizeCoefficients (true);  

    seg.setModelType (pcl::SACMODEL_PLANE);  
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setDistanceThreshold (threshould_);  
    
    seg.setInputCloud (cloud.makeShared ());  
    seg.segment (*inliers, *coefficients);  
    
    for (size_t i = 0; i < inliers->indices.size (); ++i) {  
        cloud.points[inliers->indices[i]].r = 255;  
        cloud.points[inliers->indices[i]].g = 0;  
        cloud.points[inliers->indices[i]].b = 0;  
    }
    sensor_msgs::PointCloud2 pub_msg;
    pcl::toROSMsg(cloud,pub_msg);
    pcl_pub_.publish(pub_msg);
}