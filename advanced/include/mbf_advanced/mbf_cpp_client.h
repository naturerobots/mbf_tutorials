#pragma once

#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseResult.h>
#include <mbf_advanced/helpers.h>

namespace mbf_advanced
{

struct MBFClient
{
    explicit MBFClient(std::vector<mbf_msgs::MoveBaseGoal> pose_goals)
            : pose_goals(std::move(pose_goals))
            , ac("move_base_flex/move_base", true)
    {
        ac.waitForServer();
        ROS_INFO("Connected to MBF action server");
    }

    bool perform()
    {
        for (const auto& goal: pose_goals)
        {
            ROS_INFO_STREAM("Attempting to reach:\n" << goal.target_pose);
            auto result = *mbf_advanced::move(ac, goal);
            if (result.outcome != mbf_msgs::MoveBaseResult::SUCCESS)
            {
                ROS_ERROR_STREAM("Couldn't reach " << goal.target_pose);
                return false;
            }
        }

        return true;
    }

    std::vector<mbf_msgs::MoveBaseGoal> pose_goals;
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> ac;
};

} // end