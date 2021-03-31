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
            : pose_goals_(std::move(pose_goals))
            , it_(pose_goals_.begin())
            , home_(pose_goals_.back())
            , ac_("move_base_flex/move_base", true)
    {
        ac_.waitForServer();
        ROS_INFO("Connected to MBF action server");
    }

    bool performCircle()
    {
        for (const auto& goal: pose_goals_)
        {
            if (!log_move(goal))
            {
                return false;
            }
        }

        return true;
    }

    bool log_move(const mbf_msgs::MoveBaseGoal& goal)
    {
        ROS_INFO_STREAM("Attempting to reach " << goal.target_pose.pose.position.x << " / " << goal.target_pose.pose.position.y);
        auto result = mbf_advanced::move(ac_, goal);
        if (result->outcome != mbf_msgs::MoveBaseResult::SUCCESS)
        {
            ROS_ERROR_STREAM("Couldn't reach " << goal.target_pose.pose.position.x << " / " << goal.target_pose.pose.position.y);
            return false;
        }

        return true;
    }

    bool next_move()
    {
        auto result = log_move(*it_);
        ++it_;
        if (it_ == pose_goals_.end())
        {
            it_ = pose_goals_.begin();
        }
        return result;
    }

    bool prev_move()
    {
        if (it_ != pose_goals_.begin())
        {
            --it_;
            return log_move(*it_);
        }
        else
        {
            return driveHome();
        }
    }

    bool driveHome()
    {
        ROS_WARN_STREAM("Driving home!");
        return log_move(home_);
    }

    std::vector<mbf_msgs::MoveBaseGoal> pose_goals_;
    std::vector<mbf_msgs::MoveBaseGoal>::const_iterator it_;
    const mbf_msgs::MoveBaseGoal& home_;
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> ac_;
};

} // end