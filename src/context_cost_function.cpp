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

// defining constants
#define PREDICTD_HUMAN_PUB_TOPIC "predicted_human_poses"
#define PREDICT_SERVICE_NAME "/human_pose_prediction/predict_2d_human_poses"

#define ALPHA_MAX 2.09 // (2*M_PI/3) radians, angle between robot heading and inverse of human heading
#define D_LOW 0.7 // meters, minimum distance for compatibility measure
#define D_HIGH 10.0 // meters, maximum distance for compatibility measure
#define PREDICT_TIME 2.0 // seconds, time for predicting human and robot position, before checking compatibility

#define MESSAGE_THROTTLE_PERIOD 4.0 // seconds

#include <hanp_local_planner/context_cost_function.h>

namespace hanp_local_planner
{
    // empty constructor and destructor
    ContextCostFunction::ContextCostFunction() {}
    ContextCostFunction::~ContextCostFunction() {}

    void ContextCostFunction::initialize(std::string global_frame, tf::TransformListener* tf)
    {
        ros::NodeHandle private_nh("~/");
        predict_humans_client_ = private_nh.serviceClient<hanp_prediction::HumanPosePredict>(PREDICT_SERVICE_NAME);

        // initialize variables
        global_frame_ = global_frame;
        tf_ = tf;
    }

    bool ContextCostFunction::prepare()
    {
        // set default parameters
        setParams(ALPHA_MAX, D_LOW, D_HIGH, PREDICT_TIME);

        return true;
    }

    void ContextCostFunction::setParams(double alpha_max, double d_low,
        double d_high, double predict_time)
    {
        alpha_max_ = alpha_max;
        d_low_ = d_low;
        d_high_ = d_high;
        predict_time_ = predict_time;

        ROS_DEBUG_NAMED("context_cost_function", "context-cost function parameters set: "
        "alpha_max=%f, d_low=%f, d_high=:%f, predict_time=%f",
        alpha_max_, d_low_, d_high_, predict_time_);
    }

    // abuse this function to give sclae with with the trajectory should be truncated
    double ContextCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {
        // TODO: discard humans, if information is too old
        // TODO: discard humans who are behind the robot

        hanp_prediction::HumanPosePredict predict_srv;
        predict_srv.request.predict_time = predict_time_;
        if(!predict_humans_client_.call(predict_srv))
        {
            ROS_DEBUG_THROTTLE_NAMED(MESSAGE_THROTTLE_PERIOD, "context_cost_function",
                "failed to call %s service, is prediction server running?", PREDICT_SERVICE_NAME);
            return 1.0;
        }

        ROS_DEBUG_NAMED("context_cost_function", "received %d predicted humans",
            predict_srv.response.predicted_humans.size());

        // transform humans
        std::vector<hanp_prediction::PredictedPoses> transformed_humans;
        for (auto human : predict_srv.response.predicted_humans)
        {
            transformed_humans.push_back(transformHumanPoses(human, predict_srv.response.header.frame_id));
        }
        ROS_DEBUG_NAMED("context_cost_function", "transformied %d humans to %s frame",
            transformed_humans.size(), global_frame_.c_str());

        // temporary variables for future robot pose, and compatibility
        double rx, ry, rtheta, d_p, alpha, compatibility;
        auto point_index_max = traj.getPointsSize();

        for(auto transformed_human : transformed_humans)
        {
            for(auto future_human_pose : transformed_human.poses)
            {
                unsigned int point_index = 0;
                do
                {
                    // get the future pose of the robot
                    traj.getPoint(point_index, rx, ry, rtheta);

                    // calculate distance of robot to person
                    d_p = hypot(rx - future_human_pose.x, ry - future_human_pose.y);
                    // ROS_DEBUG_NAMED("context_cost_function", "rx=%f, ry=%f, hx=%f, hy=%f, d_p=%f",
                    // rx, ry, future_human_pose.x, future_human_pose.y, d_p);
                    alpha = fabs(angles::shortest_angular_distance(rtheta,
                        angles::normalize_angle_positive(future_human_pose.theta) - M_PI));
                    // ROS_DEBUG_NAMED("context_cost_function", "rtheta=%f, h_inv_theta=%f, alpha=%f",
                    // rtheta, angles::normalize_angle_positive(future_human_pose.theta) - M_PI, alpha);

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
                //     future_human_pose.x, future_human_pose.y);

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

    hanp_prediction::PredictedPoses ContextCostFunction::transformHumanPoses(
        hanp_prediction::PredictedPoses& predicted_human, std::string frame_id)
    {
        if(global_frame_ != frame_id)
        {
            hanp_prediction::PredictedPoses transformed_human;

            //transform human pose in global frame
            int res;
            try
            {
                std::string error_msg;
                res = tf_->waitForTransform(global_frame_, frame_id,
                    ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &error_msg);
                tf::StampedTransform humans_to_global_transform;
                tf_->lookupTransform(global_frame_, frame_id, ros::Time(0), humans_to_global_transform);

                for(auto predicted_pose : predicted_human.poses)
                {
                    // transform position
                    geometry_msgs::Pose human_transformed;
                    tf::Pose human_pose(tf::Quaternion(predicted_pose.theta, 0, 0),
                        tf::Vector3(predicted_pose.x, predicted_pose.y, 0));
                    tf::poseTFToMsg(humans_to_global_transform * human_pose, human_transformed);

                    geometry_msgs::Pose2D transformed_pose;
                    transformed_pose.x = human_transformed.position.x;
                    transformed_pose.y = human_transformed.position.y;
                    transformed_pose.theta = tf::getYaw(human_transformed.orientation);
                    transformed_human.poses.push_back(transformed_pose);

                    ROS_DEBUG_NAMED("context_cost_function", "transformed human pose"
                    " to %s frame, resulting pose: x=%f, y=%f theta=%f",
                    global_frame_.c_str(), transformed_pose.x, transformed_pose.y, transformed_pose.theta);
                }
                transformed_human.track_id = predicted_human.track_id;
            }
            catch(const tf::ExtrapolationException &ex)
            {
                ROS_DEBUG("context_cost_function: cannot extrapolate transform");
            }
            catch(const tf::TransformException &ex)
            {
                ROS_ERROR("context_cost_function: transform failure (%d): %s", res, ex.what());
            }

            return transformed_human;
        }
        else
        {
            return predicted_human;
        }
    }
}
