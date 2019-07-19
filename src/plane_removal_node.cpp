#include "plane_removal.hpp"

int main(int argc,char *argv[]){
    ros::init(argc, argv, "plane_removal_node");
    plane_removal p_r;
    ros::spin();
    return 0;
}