#ifndef   DOWNSAMPLING_HPP
#define   DOWNSAMPLING_HPP

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// #pragma GCC optimize ("O3") // 最適化レベルの変更 O0〜O3 などを指定
// #pragma GCC target ("avx")  // ターゲットの変更 sse4, avx, avx2 など

class downsampling{
    private:
        ros::NodeHandle nh_;
        float leafsize_x_,leafsize_y_,leafsize_z_;

        ros::Subscriber pcl_sub_;
        ros::Publisher pcl_pub_;
        void point_cloud_CB_(const sensor_msgs::PointCloud2ConstPtr&);
        
    public:
        downsampling();
};
#endif//DOWNSAMPLING_HPP