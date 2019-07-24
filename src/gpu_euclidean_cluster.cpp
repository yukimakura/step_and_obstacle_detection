#include "gpu_euclidean_cluster.hpp"
obstacle_detect_gpu::obstacle_detect_gpu():cluster_tolerance_(0.02),min_cluster_size_(100),max_cluster_size_(25000){
    ros::NodeHandle n_arg("~");
    n_arg.getParam("cluster_tolerance", cluster_tolerance_);
    n_arg.getParam("min_cluster_size", min_cluster_size_);
    n_arg.getParam("max_cluster_size", max_cluster_size_);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detected_point_cloud", 1);
    pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("plane_removed_point_cloud", 1, &obstacle_detect_gpu::point_cloud_CB_,this);
}
void obstacle_detect_gpu::point_cloud_CB_(const sensor_msgs::PointCloud2ConstPtr& cb_cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mono(new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::fromROSMsg(*cb_cloud,*cloud_mono);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);  
    pcl::fromROSMsg(*cb_cloud,*cloud_cluster);

    pcl::gpu::Octree::PointCloud cloud_device;

    cloud_device.upload(cloud_mono->points);
    
    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();

    std::vector<pcl::PointIndices> cluster_indices_gpu;
    pcl::gpu::EuclideanClusterExtraction gec;
    gec.setClusterTolerance (cluster_tolerance_); // 2cm
    gec.setMinClusterSize (min_cluster_size_);
    gec.setMaxClusterSize (max_cluster_size_);
    gec.setSearchMethod (octree_device);
    gec.setHostCloud(cloud_mono);
    gec.extract (cluster_indices_gpu);
    //  octree_device.clear();

    printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    std::cout << "INFO: stopped with the GPU version" << std::endl;

    j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_mono->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        j++;
    }

    sensor_msgs::PointCloud2 pub_msg;
    pcl::toROSMsg(*cloud_cluster,pub_msg);
    pcl_pub_.publish(pub_msg);
}