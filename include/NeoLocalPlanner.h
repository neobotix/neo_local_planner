/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
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
 *   * Neither the name of the Neobotix nor the names of its
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
 *********************************************************************/

#ifndef INCLUDE_NEOLOCALPLANNER_H_
#define INCLUDE_NEOLOCALPLANNER_H_

#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/local_planner_limits.h>

#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>


namespace neo_local_planner {

class NeoLocalPlanner : public nav_core::BaseLocalPlanner {
public:
	NeoLocalPlanner();

	~NeoLocalPlanner();

	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

	bool isGoalReached() override;

	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

	void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

private:
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
	tf::TransformListener* m_tf = 0;
	costmap_2d::Costmap2DROS* m_cost_map = 0;
	std::vector<geometry_msgs::PoseStamped> m_global_plan;

	boost::mutex m_odometry_mutex;
	nav_msgs::Odometry::ConstPtr m_odometry;

	ros::Subscriber m_odom_sub;
	ros::Publisher m_local_plan_pub;

	std::string m_global_frame = "map";
	std::string m_local_frame = "odom";
	std::string m_base_frame = "base_link";

	base_local_planner::LocalPlannerLimits m_limits = {};

	double m_goal_tune_time = 0;		// [s]
	double m_lookahead_time = 0;		// [s]
	double m_lookahead_dist = 0;		// [m]
	double m_start_yaw_error = 0;		// [rad]
	double m_pos_x_gain = 0;			// [1/s]
	double m_pos_y_gain = 0;			// [1/s]
	double m_pos_y_yaw_gain = 0;		// [rad/s^2]
	double m_yaw_gain = 0;				// [1/s]
	double m_static_yaw_gain = 0;		// [1/s]
	double m_cost_x_gain = 0;
	double m_cost_y_gain = 0;
	double m_cost_y_yaw_gain = 0;
	double m_cost_y_lookahead_dist = 0;	// [m]
	double m_cost_y_lookahead_time = 0;	// [s]
	double m_cost_yaw_gain = 0;
	double m_low_pass_gain = 0;
	double m_max_curve_vel = 0;			// [rad/s]
	double m_max_goal_dist = 0;			// [m]
	double m_max_backup_dist = 0;		// [m]
	double m_max_cost = 0;				// [1]
	double m_min_stop_dist = 0;			// [m]
	double m_emergency_acc_lim_x = 0;	// [m/s^2]

	bool m_enable_software_stop = true; 
	bool m_differential_drive = false;
	bool m_constrain_final = false;

	enum state_t {
		STATE_IDLE,
		STATE_TRANSLATING,
		STATE_ROTATING,
		STATE_ADJUSTING,
		STATE_TURNING,
		STATE_STUCK
	};

	state_t m_state = state_t::STATE_IDLE;

	ros::WallTime m_last_time;
	ros::WallTime m_first_goal_reached_time;

	bool m_is_goal_reached = false;
	uint64_t m_update_counter = 0;
	double m_last_control_values[3] = {};
	geometry_msgs::Twist m_last_cmd_vel;

};


} // neo_local_planner

#endif /* INCLUDE_NEOLOCALPLANNER_H_ */
