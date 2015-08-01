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
#define ALPHA_MAX M_PI/2 // radians, angle between robot heading and inverse of human heading
#define D_LOW 1.3// meters, minimum distance for compatibility measure
#define D_HIGH 10.0// meters, maximum distance for compatibility measure
#define PREDICT_TIME 3.0 // seconds, time for predicting human and robot position, before checking compatibility

#include <hanp_local_planner/context_cost_function.h>

namespace hanp_local_planner
{
    // empty constructor and destructor
    ContextCostFunction::ContextCostFunction() {}
    ContextCostFunction::~ContextCostFunction() {}

    bool ContextCostFunction::prepare()
    {
        // set default parameters
        setParams(ALPHA_MAX, D_LOW, D_HIGH, PREDICT_TIME);

        return true;
    }

    void ContextCostFunction::setParams(double alpha_max, double d_low, double d_high, double predict_time)
    {
        alpha_max_ = alpha_max;
        d_low_ = d_low;
        d_high_ = d_high;
        predict_time_ = predict_time;
    }

    double ContextCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {
        // copy humans for thread safety
        auto humans = humans_;

        // TODO: predict robot position

        // TODO: discard humans, if information is too old

        for(auto human : humans.tracks)
        {
            // TODO: predict human position, for three speed possibilities

            // TODO: calculate distance of robot to person
            //  hypot(rx - hx, ry - hy);

            // TODO: calculate compatibility and update overall value to lowest of all
        }

        return 0.0;
    }

    void ContextCostFunction::updateTrackedHumans(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        humans_ = tracked_humans;
    }

    double ContextCostFunction::getCompatabilty(double d_p, double alpha)
    {
        return 0.0;
    }




}
