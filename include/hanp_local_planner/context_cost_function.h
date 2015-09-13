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

#ifndef CONTEXT_COST_FUNCTION_H_
#define CONTEXT_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <hanp_prediction/HumanPosePredict.h>

#include <visualization_msgs/MarkerArray.h>

namespace hanp_local_planner {

    typedef std::array<double, 3> human_pose;

    class ContextCostFunction: public base_local_planner::TrajectoryCostFunction
    {
    public:
        ContextCostFunction();
        ~ContextCostFunction();

        void initialize(std::string global_frame, tf::TransformListener* tf);
        bool prepare();
        double scoreTrajectory(base_local_planner::Trajectory &traj);

        void setParams(double alpha_max, double d_low, double d_high, double predict_time);

    private:
        ros::ServiceClient predict_humans_client_;

        tf::TransformListener* tf_;

        double alpha_max_, d_low_, d_high_;
        double predict_time_;
        std::string global_frame_;

        double getCompatabilty(double d_p, double alpha);

        hanp_prediction::PredictedPoses transformHumanPoses(hanp_prediction::PredictedPoses&, std::string frame_id);

        bool publish_predicted_human_markers_ = true;
        visualization_msgs::MarkerArray predicted_humans_markers_;
        ros::Publisher predict_human_pub_;
    };
}

#endif // CONTEXT_COST_FUNCTION_H_
