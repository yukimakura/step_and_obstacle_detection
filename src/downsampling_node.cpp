#include "downsampling.hpp"

int main(int argc,char *argv[]){
    ros::init(argc, argv, "downsampling_node");
    downsampling ds;
    ros::spin();
    return 0;
}