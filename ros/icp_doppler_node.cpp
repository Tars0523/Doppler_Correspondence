#include <ros/ros.h>
#include <ros/icp_doppler_.hpp>   

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_doppler_node");   
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");                    
    doppler::IcpDopplerNode node(nh, pnh);       
    ros::spin();
    return 0;
}
