#include "plane_detect.hpp"

int main(int argc,char *argv[]){
    ros::init(argc, argv, "plane_detect_node");
    plane_detect p_d;
    ros::spin();
    return 0;
}