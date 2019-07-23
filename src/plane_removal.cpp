#include "plane_removal.hpp"
plane_removal::plane_removal():threshould_(0.1),max_iterations_(100),probability_(0.95){
    ros::NodeHandle n_arg("~");
    n_arg.getParam("threshould", threshould_);
    n_arg.getParam("max_iterations", max_iterations_);
    n_arg.getParam("probability", probability_);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_removed_point_cloud", 1);
    pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &plane_removal::point_cloud_CB_,this);
}
void plane_removal::point_cloud_CB_(const sensor_msgs::PointCloud2ConstPtr& cb_cloud){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
    pcl::fromROSMsg(*cb_cloud,*cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;  

    seg.setOptimizeCoefficients (true);  

    seg.setModelType (pcl::SACMODEL_PLANE);  
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setDistanceThreshold (threshould_); 
    seg.setMaxIterations(max_iterations_);
    seg.setProbability(probability_);
    
    seg.setInputCloud (cloud->makeShared ());  
    seg.segment (*inliers, *coefficients);  

    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
    extract.filter(*cloud_output);

    sensor_msgs::PointCloud2 pub_msg;
    pcl::toROSMsg(*cloud_output,pub_msg);
    pcl_pub_.publish(pub_msg);
}