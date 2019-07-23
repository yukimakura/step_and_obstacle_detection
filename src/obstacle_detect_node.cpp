#include "obstacle_detect.hpp"

int main(int argc,char *argv[]){
    ros::init(argc, argv, "obstacle_detect_node");
    obstacle_detect o_d;
    ros::spin();
    return 0;
}