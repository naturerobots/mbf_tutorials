#include <ros/ros.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_advanced/mbf_cpp_client.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbf_cpp_client_node");
    ros::NodeHandle n;

    mbf_advanced::MBFClient mbfclient(std::move(mbf_advanced::loadPoseGoals(POSE_PATH)));
    mbfclient.performCircle();

    ROS_INFO_STREAM("Success");
}