#include "obstacle_detect.hpp"
obstacle_detect::obstacle_detect():cluster_tolerance_(0.02),min_cluster_size_(100),max_cluster_size_(25000){
    ros::NodeHandle n_arg("~");
    n_arg.getParam("cluster_tolerance", cluster_tolerance_);
    n_arg.getParam("min_cluster_size", min_cluster_size_);
    n_arg.getParam("max_cluster_size", max_cluster_size_);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detected_point_cloud", 1);
    pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("plane_removed_point_cloud", 1, &obstacle_detect::point_cloud_CB_,this);
}
void obstacle_detect::point_cloud_CB_(const sensor_msgs::PointCloud2ConstPtr& cb_cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mono(new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::fromROSMsg(*cb_cloud,*cloud_mono);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);  
    pcl::fromROSMsg(*cb_cloud,*cloud_cluster);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (cloud_mono);
    if (!cloud_mono->is_dense)
    {
        cloud_mono->is_dense = false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_mono,*cloud_mono, indices);
    }
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (min_cluster_size_);  
    ec.setMaxClusterSize (max_cluster_size_);  
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_mono);
    ec.extract (cluster_indices);
    
    int j = 0;  
    float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    {  
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
                cloud_cluster->points[*pit].r = colors[j%6][0];  
                cloud_cluster->points[*pit].g = colors[j%6][1];  
                cloud_cluster->points[*pit].b = colors[j%6][2];  
        }  
        j++;
    }

    sensor_msgs::PointCloud2 pub_msg;
    pcl::toROSMsg(*cloud_cluster,pub_msg);
    pcl_pub_.publish(pub_msg);
}