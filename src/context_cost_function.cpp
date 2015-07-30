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
#define D_LOW 1.5// meters, minimum distance for compatibility measure
#define D_MAX 10.0// meters, maximum distance for compatibility measure

#ifndef DISTANCE2D
    #define DISTANCE2D(x1,y1,x2,y2) (sqrt(((x2)-(x1))*((x2)-(x1))+((y2)-(y1))*((y2)-(y1))))
#endif

#include <hanp_local_planner/context_cost_function.h>

namespace hanp_local_planner
{
    // empty constructor and destructor
    ContextCostFunction::ContextCostFunction() {}
    ContextCostFunction::~ContextCostFunction() {}

    bool ContextCostFunction::prepare()
    {
        return true;
    }

    double ContextCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {
        return 0.0;
    }

    void ContextCostFunction::updateTrackedHumans(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        return;
    }

    // void ContextCostFunction::checkCompatabilty(base_local_planner::Trajectory& trajectory, tf::Stamped<tf::Pose> global_pose,
    //     tf::Stamped<tf::Pose> global_vel, tf::Stamped<tf::Pose>& drive_velocities)
    // {
    //     // get tracked humans pointer locally for thread safety
    //     auto tracked_humans = tracked_humans_;
    //
    //     if(tracked_humans.tracks.size() > 0)
    //     {
    //         //transform human pose in global frame
    //         int res;
    //         try
    //         {
    //             std::string error_msg;
    //             res = tf_->waitForTransform(global_pose.frame_id_, tracked_humans.header.frame_id,
    //                 ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &error_msg);
    //             tf::StampedTransform humans_to_global_transform;
    //             tf_->lookupTransform(global_pose.frame_id_, tracked_humans.header.frame_id,
    //                 ros::Time(0), humans_to_global_transform);
    //             auto humans_to_global_translation = humans_to_global_transform.getOrigin();
    //
    //             for(auto tracked_human : tracked_humans.tracks)
    //             {
    //                 // find angle
    //
    //                 auto hx = tracked_human.pose.pose.position.x + humans_to_global_translation[0];
    //                 auto hy = tracked_human.pose.pose.position.y + humans_to_global_translation[1];
    //                 // tracked_human_transformed.pose.pose.position.z = tracked_human.pose.pose.position.z + humans_to_scan_translation[2];
    //                 // do for orientation
    //
    //                 // check distance to the humans in TODO:forward direction
    //                 //  stop if distance is below some value
    //                 double dist = DISTANCE2D(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), hx, hy);
    //                 ROS_INFO_NAMED("compatibility", "distance to human %lu is %f", tracked_human.track_id, dist);
    //                 if(dist < D_LOW)
    //                 {
    //                     // TODO: empty the trajectory
    //
    //                     // stop the robot
    //                     tf::Vector3 start(0, 0, 0);
    //                     drive_velocities.setOrigin(start);
    //                     tf::Matrix3x3 matrix;
    //                     matrix.setRotation(tf::createQuaternionFromYaw(0));
    //                     drive_velocities.setBasis(matrix);
    //                     ROS_INFO_NAMED("compatibility", "reducing robot speed to 0");
    //
    //                     // starting from 0 check with SPEED_ADAPTATION_STEPS
    //                     //  calculate conflict distance to human, for three speed possibilities
    //                     //  if it is below 0, select the speed and return
    //                 }
    //             }
    //         }
    //         catch(const tf::TransformException &ex)
    //         {
    //             ROS_ERROR("transform failure (%d): %s", res, ex.what());
    //         }
    //     }
    // }

}
