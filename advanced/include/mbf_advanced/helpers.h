#pragma once

#include <fstream>
#include <vector>
#include <string>
#include <ros/console.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_msgs/MoveBaseResult.h>

std::ostream& operator<<(std::ostream& os, geometry_msgs::Pose pose)
{
    return os << pose.position.x << " / " << pose.position.y << " / " << pose.position.z;
}

namespace mbf_advanced
{

std::vector<mbf_msgs::MoveBaseGoal> loadPoseGoals(const std::string& filepath)
{
    std::ifstream infile(filepath);
    std::vector<mbf_msgs::MoveBaseGoal> pose_goals;
    pose_goals.reserve(8);

    geometry_msgs::Pose pose;

    while (infile >> pose.position.x
                  >> pose.position.y
                  >> pose.position.z
                  >> pose.orientation.x
                  >> pose.orientation.y
                  >> pose.orientation.z
                  >> pose.orientation.w)
    {

        mbf_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = pose.position;
        goal.target_pose.pose.orientation = pose.orientation;
        pose_goals.push_back(goal);
    }

    return pose_goals;
}

mbf_msgs::MoveBaseResult::ConstPtr
move(actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> &ac, const mbf_msgs::MoveBaseGoal &goal)
{
    ac.sendGoal(goal);
    ac.waitForResult();
    return ac.getResult();
}

} // end namespace mbf_advanced