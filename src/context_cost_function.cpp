/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Wed Jul 29 2015
 */

// context-cost related parameters
#define ALPHA_MAX 2.09 // (2*M_PI/3) radians, angle between robot heading and inverse of human heading
#define D_LOW 0.7 // meters, minimum distance for compatibility measure
#define D_HIGH 10.0 // meters, maximum distance for compatibility measure
#define PREDICT_TIME 2.0 // seconds, time for predicting human and robot position, before checking compatibility
#define HUMAN_POSE_PREDICT_LOWER_SCALE 0.8 // human slow-down velocity multiplier
#define HUMAN_POSE_PREDICT_HIGHER_SCALE 1.2 // human speed-up velocity multiplier

#include <hanp_local_planner/context_cost_function.h>

namespace hanp_local_planner
{
    // empty constructor and destructor
    ContextCostFunction::ContextCostFunction() {}
    ContextCostFunction::~ContextCostFunction() {}

    bool ContextCostFunction::prepare()
    {
        // set default parameters
        std::vector<double> human_pose_predict_scales = {HUMAN_POSE_PREDICT_LOWER_SCALE, 1.0, HUMAN_POSE_PREDICT_HIGHER_SCALE};
        setParams(ALPHA_MAX, D_LOW, D_HIGH, PREDICT_TIME, human_pose_predict_scales);

        return true;
    }

    void ContextCostFunction::setParams(double alpha_max, double d_low, double d_high,
        double predict_time, std::vector<double> human_pose_predict_scales)
    {
        alpha_max_ = alpha_max;
        d_low_ = d_low;
        d_high_ = d_high;
        predict_time_ = predict_time;
        human_pose_predict_scales_ = human_pose_predict_scales;

        ROS_DEBUG_NAMED("context_cost_function", "context-cost function parameters set: "
        "alpha_max=%f, d_low=%f, d_high=:%f, predict_time=%f, human_pose_predict_scales=[%f, %f, %f]",
        alpha_max_, d_low_, d_high_, predict_time_, human_pose_predict_scales_[0],
        human_pose_predict_scales_[1], human_pose_predict_scales_[2]);
    }

    // abuse this function to give sclae with with the trajectory should be truncated
    double ContextCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {
        // copy humans for thread safety
        auto humans = humans_;

        // temporary variables for future robot pose, and compatibility
        double rx, ry, rtheta, d_p, alpha, compatibility;
        auto point_index_max = traj.getPointsSize();

        // TODO: discard humans, if information is too old

        for(auto human : humans.tracks)
        {
            // TODO: discard humans who are behind the robot

            // predict human position, for three speed possibilities
            for(auto future_human_pose : predictHumanPoses(human))
            {
                unsigned int point_index = 0;
                do
                {
                    // get the future pose of the robot
                    traj.getPoint(point_index, rx, ry, rtheta);

                    // calculate distance of robot to person
                    d_p = hypot(rx - future_human_pose[0], ry - future_human_pose[1]);
                    // ROS_DEBUG_NAMED("context_cost_function", "rx=%f, ry=%f, hx=%f, hy=%f, d_p=%f",
                    // rx, ry, future_human_pose[0], future_human_pose[1], d_p);
                    alpha = fabs(angles::shortest_angular_distance(rtheta,
                        angles::normalize_angle_positive(future_human_pose[2]) - M_PI));
                    // ROS_DEBUG_NAMED("context_cost_function", "rtheta=%f, h_inv_theta=%f, alpha=%f",
                    // rtheta, angles::normalize_angle_positive(future_human_pose[2]) - M_PI, alpha);

                    // check compatibility
                    compatibility = getCompatabilty(d_p, alpha);

                    ROS_DEBUG_NAMED("context_cost_function", "calculated compatibility %f"
                        " at d_p=%f, alpha=%f point: x=%f, y=%f (%d of %d)", compatibility,
                        d_p, alpha, rx, ry, point_index, traj.getPointsSize()-1);
                }
                // keep calculating, until we find incompatible situation or end of path
                while((++point_index < point_index_max) && (compatibility > 0.0));
                point_index_max = point_index;

                // ROS_DEBUG_NAMED("context_cost_function", "calculated maximum point index %d"
                //     " (out of %d) for human at x=%d, y=%f", point_index_max, traj.getPointsSize(),
                //     future_human_pose[0], future_human_pose[1]);

                // no need to check more when we have to stop
                if (point_index_max == 1)
                {
                    return 0.0;
                }
            }
        }

        auto scaling = (double)(point_index_max - 1) / (double)(traj.getPointsSize() - 1);
        ROS_DEBUG_NAMED("context_cost_function", "returning scale value of %f", scaling);

        return scaling;
    }

    void ContextCostFunction::updateTrackedHumans(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        humans_ = tracked_humans;
    }

    double ContextCostFunction::getCompatabilty(double d_p, double alpha)
    {
        if(d_p <= d_low_)
        {
            return 0.0;
        }
        else if(d_p >= d_high_)
        {
            return 1.0;
        }
        else if(alpha >= alpha_max_)
        {
            return 1.0;
        }
        else
        {
            return (((d_p - d_low_) / d_high_) * (alpha / alpha_max_));
        }
    }

    std::vector<human_pose> ContextCostFunction::predictHumanPoses(hanp_msgs::TrackedHuman& human)
    {
        std::vector<human_pose> future_human_poses;
        for(auto vel_scale : human_pose_predict_scales_)
        {
            future_human_poses.push_back({
                human.pose.pose.position.x + (human.twist.twist.linear.x * vel_scale) * predict_time_,
                human.pose.pose.position.y + (human.twist.twist.linear.y * vel_scale) * predict_time_,
                tf::getYaw(human.pose.pose.orientation)});

            // ROS_DEBUG_NAMED("context_cost_function", "predected human (%d) pose: x=%f, y=%f, theta=%f with vel sclae %f",
            //     human.track_id, future_human_poses.back()[0], future_human_poses.back()[1],
            //     future_human_poses.back()[2], vel_scale);
        }

        return future_human_poses;
    }
}
