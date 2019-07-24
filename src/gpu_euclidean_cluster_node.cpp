#include "gpu_euclidean_cluster.hpp"

int main(int argc,char *argv[]){
    ros::init(argc, argv, "obstacle_detect_gpu_node");
    obstacle_detect_gpu o_d_gpu;
    ros::spin();
    return 0;
}