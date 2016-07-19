/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#ifndef HANP_LOCAL_PLANNER_H_
#define HANP_LOCAL_PLANNER_H_

#include <vector>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <hanp_local_planner/HANPLocalPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <pcl_ros/publisher.h>
#include <base_local_planner/map_grid_visualizer.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/prefer_forward_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <hanp_local_planner/context_cost_function.h>

namespace hanp_local_planner
{
    enum DiagnosticType { INFO, WARN, ERROR };
    enum FailureType { NOT_INITIALIZED, NO_ROBOT_POSE, NO_TRANSFORMED_PLAN, EMPTY_TRANSFORMED_PLAN,
        CANNOT_ROTATE_AT_END, CURRENTLY_IN_COLLISION, PATH_IN_COLLISION };

    class HANPLocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        HANPLocalPlanner();

        void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

        ~HANPLocalPlanner();

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        bool computeVelocityCommandsAccErrors(geometry_msgs::Twist& cmd_vel);
        bool hanpComputeVelocityCommands(tf::Stamped<tf::Pose>& global_pose, geometry_msgs::Twist& cmd_vel);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        bool isGoalReached();
        bool isInitialized()
        {
            return initialized_;
        }

        // DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
        // ~DWAPlanner() {if(traj_cloud_) delete traj_cloud_;}

    private:
        bool initialized_;
        bool setup_;
        std::stringstream calc_times_;
        bool print_calc_times_ = false;

        void reconfigureCB(HANPLocalPlannerConfig &config, uint32_t level);

        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);
        bool checkTrajectory(const Eigen::Vector3f pos, const Eigen::Vector3f vel, const Eigen::Vector3f vel_samples);
        void updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan);
        base_local_planner::Trajectory findBestPath(tf::Stamped<tf::Pose> global_pose,
            tf::Stamped<tf::Pose> global_vel, tf::Stamped<tf::Pose>& drive_velocities,
            std::vector<geometry_msgs::Point> footprint_spec);

        costmap_2d::Costmap2DROS* costmap_ros_;
        tf::TransformListener* tf_;

        dynamic_reconfigure::Server<HANPLocalPlannerConfig> *dsrv_;
        hanp_local_planner::HANPLocalPlannerConfig default_config_;

        ros::Publisher g_plan_pub_, l_plan_pub_;
        std::string odom_topic_;

        tf::Stamped<tf::Pose> current_pose_;

        double stop_time_buffer_;
        double pdist_scale_, gdist_scale_, occdist_scale_;
        Eigen::Vector3f vsamples_;
        double sim_period_, sim_time_;
        double forward_point_distance_, forward_point_distance_mul_fac_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        boost::mutex configuration_mutex_;
        pcl::PointCloud<base_local_planner::MapGridCostPoint>* traj_cloud_;
        pcl_ros::Publisher<base_local_planner::MapGridCostPoint> traj_cloud_pub_;
        bool publish_cost_grid_pc_;
        bool publish_traj_pc_;
        double cheat_factor_;
        double path_clearning_distance_squared_;
        double stop_rotate_reduce_factor_;

        base_local_planner::LatchedStopRotateController latchedStopRotateController_;
        base_local_planner::OdometryHelperRos odom_helper_;
        base_local_planner::LocalPlannerUtil planner_util_;
        base_local_planner::Trajectory result_traj_;
        base_local_planner::MapGridVisualizer map_viz_;
        base_local_planner::SimpleTrajectoryGenerator generator_;
        base_local_planner::OscillationCostFunction oscillation_costs_;
        base_local_planner::ObstacleCostFunction* obstacle_costs_;
        base_local_planner::MapGridCostFunction* path_costs_;
        base_local_planner::MapGridCostFunction* goal_costs_;
        base_local_planner::MapGridCostFunction* goal_front_costs_;
        base_local_planner::MapGridCostFunction* alignment_costs_;
        base_local_planner::PreferForwardCostFunction* prefer_forward_costs_;
        base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

        hanp_local_planner::ContextCostFunction* context_cost_function_;

        std::vector<hanp_local_planner::FailureType> failures_;
    };
};
#endif
