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
#include <hanp_msgs/TrackedHumans.h>
#include <tf/transform_listener.h>

namespace hanp_local_planner {

    typedef std::array<double, 3> human_pose;

    class ContextCostFunction: public base_local_planner::TrajectoryCostFunction
    {
    public:
        ContextCostFunction();
        ~ContextCostFunction();

        bool prepare();
        double scoreTrajectory(base_local_planner::Trajectory &traj);

        void updateTrackedHumans(const hanp_msgs::TrackedHumans& tracked_humans);

        void setParams(double alpha_max, double d_low, double d_high, double predict_time);

    private:
        double alpha_max_, d_low_, d_high_;
        double predict_time_;

        hanp_msgs::TrackedHumans humans_;

        std::vector<human_pose> predictHumanPoses(hanp_msgs::TrackedHuman& human);
        double getCompatabilty(double d_p, double alpha);

        // TODO: make this configurable
        std::vector<double> human_predict_vel_steps_ = {0.7, 1.0, 1.3};
    };

}

#endif // CONTEXT_COST_FUNCTION_H_
