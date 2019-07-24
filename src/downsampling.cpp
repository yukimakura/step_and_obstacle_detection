#include "downsampling.hpp"

downsampling::downsampling():leafsize_x_(0.01f),leafsize_y_(0.01f),leafsize_z_(0.01f){
    ros::NodeHandle n_arg("~");
    n_arg.getParam("leafsize_x", leafsize_x_);
    n_arg.getParam("leafsize_y", leafsize_y_);
    n_arg.getParam("leafsize_z", leafsize_z_);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/downsampling_points", 1);
    pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &downsampling::point_cloud_CB_,this);
}


void downsampling::point_cloud_CB_(const sensor_msgs::PointCloud2ConstPtr& cb_cloud){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
    pcl::fromROSMsg(*cb_cloud,*cloud);
    
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB>  sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leafsize_x_,leafsize_y_,leafsize_z_);
    sor.filter (*cloud);

    sensor_msgs::PointCloud2 pub_msg;
    pcl::toROSMsg(*cloud,pub_msg);
    pcl_pub_.publish(pub_msg);
}