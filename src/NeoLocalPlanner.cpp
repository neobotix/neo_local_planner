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

#include "../include/NeoLocalPlanner.h"

#include <tf/transform_datatypes.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/footprint_helper.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>

// register this planner as a BaseGlobalPlanner plugin
// (see http://www.ros.org/wiki/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
PLUGINLIB_EXPORT_CLASS(neo_local_planner::NeoLocalPlanner, nav_core::BaseLocalPlanner)


namespace neo_local_planner {

std::vector<tf::Pose>::const_iterator find_closest_point(	std::vector<tf::Pose>::const_iterator begin,
															std::vector<tf::Pose>::const_iterator end,
															const tf::Vector3& pos,
															double* actual_dist = 0)
{
	auto iter_short = begin;
	double dist_short = std::numeric_limits<double>::infinity();

	for(auto iter = iter_short; iter != end; ++iter)
	{
		const double dist = (iter->getOrigin() - pos).length();
		if(dist < dist_short)
		{
			dist_short = dist;
			iter_short = iter;
		}
	}
	if(actual_dist) {
		*actual_dist = dist_short;
	}
	return iter_short;
}

std::vector<tf::Pose>::const_iterator move_along_path(	std::vector<tf::Pose>::const_iterator begin,
														std::vector<tf::Pose>::const_iterator end,
														const double dist, double* actual_dist = 0)
{
	auto iter = begin;
	auto iter_prev = iter;
	double dist_left = dist;

	while(iter != end)
	{
		const double dist = (iter->getOrigin() - iter_prev->getOrigin()).length();
		dist_left -= dist;
		if(dist_left <= 0) {
			break;
		}
		iter_prev = iter;
		iter++;
	}
	if(iter == end) {
		iter = iter_prev;		// targeting final pose
	}
	if(actual_dist) {
		*actual_dist = dist - dist_left;
	}
	return iter;
}

std::vector<base_local_planner::Position2DInt> get_line_cells(
								costmap_2d::Costmap2D* cost_map,
								const tf::Vector3& world_pos_0,
								const tf::Vector3& world_pos_1)
{
	int coords[2][2] = {};
	cost_map->worldToMapEnforceBounds(world_pos_0.x(), world_pos_0.y(), coords[0][0], coords[0][1]);
	cost_map->worldToMapEnforceBounds(world_pos_1.x(), world_pos_1.y(), coords[1][0], coords[1][1]);

	std::vector<base_local_planner::Position2DInt> cells;
	base_local_planner::FootprintHelper().getLineCells(coords[0][0], coords[1][0], coords[0][1], coords[1][1], cells);
	return cells;
}

double get_cost(costmap_2d::Costmap2DROS* cost_map_ros, const tf::Vector3& world_pos)
{
	auto cost_map = cost_map_ros->getCostmap();

	int coords[2] = {};
	cost_map->worldToMapEnforceBounds(world_pos.x(), world_pos.y(), coords[0], coords[1]);

	return cost_map->getCost(coords[0], coords[1]) / 255.;
}

double compute_avg_line_cost(	costmap_2d::Costmap2DROS* cost_map_ros,
								const tf::Vector3& world_pos_0,
								const tf::Vector3& world_pos_1)
{
	auto cost_map = cost_map_ros->getCostmap();
	const std::vector<base_local_planner::Position2DInt> cells = get_line_cells(cost_map, world_pos_0, world_pos_1);

	double avg_cost = 0;
	for(auto cell : cells) {
		avg_cost += cost_map->getCost(cell.x, cell.y) / 255.;
	}
	return avg_cost / cells.size();
}

double compute_max_line_cost(	costmap_2d::Costmap2DROS* cost_map_ros,
								const tf::Vector3& world_pos_0,
								const tf::Vector3& world_pos_1)
{
	auto cost_map = cost_map_ros->getCostmap();
	const std::vector<base_local_planner::Position2DInt> cells = get_line_cells(cost_map, world_pos_0, world_pos_1);

	int max_cost = 0;
	for(auto cell : cells) {
		max_cost = std::max(max_cost, int(cost_map->getCost(cell.x, cell.y)));
	}
	return max_cost / 255.;
}

NeoLocalPlanner::NeoLocalPlanner()
{
}

NeoLocalPlanner::~NeoLocalPlanner()
{
}

bool NeoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
	boost::mutex::scoped_lock lock(m_odometry_mutex);

	if(!m_odometry)
	{
		ROS_WARN_NAMED("NeoLocalPlanner", "Waiting for odometry ...");
		return false;
	}
	if(m_global_plan.empty())
	{
		ROS_WARN_NAMED("NeoLocalPlanner", "Global plan is empty!");
		return false;
	}

	// compute delta time
	const ros::WallTime time_now = ros::WallTime::now();
	const double dt = fmax(fmin((time_now - m_last_time).toSec(), 0.1), 0);

	// get latest global to local transform (map to odom)
	tf::StampedTransform global_to_local;
	try {
		m_tf->lookupTransform(m_local_frame, m_global_frame, ros::Time(), global_to_local);
	} catch(...) {
		ROS_WARN_NAMED("NeoLocalPlanner", "lookupTransform(m_local_frame, m_global_frame) failed");
		return false;
	}

	// transform plan to local frame (odom)
	std::vector<tf::Pose> local_plan;
	for(const auto& pose : m_global_plan)
	{
		tf::Stamped<tf::Pose> pose_;
		tf::poseStampedMsgToTF(pose, pose_);
		local_plan.push_back(global_to_local * pose_);
	}

	// get latest local pose
	tf::Pose local_pose;
	tf::poseMsgToTF(m_odometry->pose.pose, local_pose);

	const double start_yaw = tf::getYaw(local_pose.getRotation());
	const double start_vel_x = m_odometry->twist.twist.linear.x;
	const double start_vel_y = m_odometry->twist.twist.linear.y;
	const double start_yawrate = m_odometry->twist.twist.angular.z;

	// calc dynamic lookahead distances
	const double lookahead_dist = m_lookahead_dist + fmax(start_vel_x, 0) * m_lookahead_time;
	const double cost_y_lookahead_dist = m_cost_y_lookahead_dist + fmax(start_vel_x, 0) * m_cost_y_lookahead_time;

	// predict future pose (using second order midpoint method)
	tf::Vector3 actual_pos;
	double actual_yaw = 0;
	{
		const double midpoint_yaw = start_yaw + start_yawrate * m_lookahead_time / 2;
		actual_pos = local_pose.getOrigin() + tf::Matrix3x3(tf::createQuaternionFromYaw(midpoint_yaw))
												* tf::Vector3(start_vel_x, start_vel_y, 0) * m_lookahead_time;
		actual_yaw = start_yaw + start_yawrate * m_lookahead_time;
	}
	// Determining the presence of obstacles in the footprint
	geometry_msgs::Point poses1;
	poses1.x = local_pose.getOrigin().x();
	poses1.y = local_pose.getOrigin().y();
	std::vector<geometry_msgs::Point> P1;
	P1 = m_cost_map->getRobotFootprint();

	// Updating the robot footprint 
	for (int i = 0; i<P1.size(); i++)
	{
		auto pos = tf::Matrix3x3(tf::createQuaternionFromYaw(actual_yaw))* tf::Vector3(P1[i].x, P1[i].y, 0);
		P1[i].x= pos[0]+ actual_pos[0];
		P1[i].y= pos[1]+ actual_pos[1];
	}

	base_local_planner::CostmapModel world_model_(*m_cost_map->getCostmap());
	const bool obstacle_in_rot = world_model_.footprintCost(poses1, P1, 1.0,1.0);

	const tf::Pose actual_pose = tf::Pose(tf::createQuaternionFromYaw(actual_yaw), actual_pos);

	// compute cost gradients
	const double delta_x = 0.3;
	const double delta_y = 0.2;
	const double delta_yaw = 0.1;

	const double center_cost = get_cost(m_cost_map, actual_pos);

	const double delta_cost_x = (
		compute_avg_line_cost(m_cost_map, actual_pos, actual_pose * tf::Vector3(delta_x, 0, 0)) -
		compute_avg_line_cost(m_cost_map, actual_pos, actual_pose * tf::Vector3(-delta_x, 0, 0)))
		/ delta_x;

	const double delta_cost_y = (
		compute_avg_line_cost(m_cost_map, actual_pos, actual_pose * tf::Vector3(cost_y_lookahead_dist, delta_y, 0)) -
		compute_avg_line_cost(m_cost_map, actual_pos, actual_pose * tf::Vector3(cost_y_lookahead_dist, -delta_y, 0)))
		/ delta_y;

	const double delta_cost_yaw = (
		(
			compute_avg_line_cost(m_cost_map,	actual_pose * (tf::Matrix3x3(tf::createQuaternionFromYaw(delta_yaw)) * tf::Vector3(delta_x, 0, 0)),
												actual_pose * (tf::Matrix3x3(tf::createQuaternionFromYaw(delta_yaw)) * tf::Vector3(-delta_x, 0, 0)))
		) - (
			compute_avg_line_cost(m_cost_map,	actual_pose * (tf::Matrix3x3(tf::createQuaternionFromYaw(-delta_yaw)) * tf::Vector3(delta_x, 0, 0)),
												actual_pose * (tf::Matrix3x3(tf::createQuaternionFromYaw(-delta_yaw)) * tf::Vector3(-delta_x, 0, 0)))
		)) / (2 * delta_yaw);

	// fill local plan later
	nav_msgs::Path::Ptr local_path = boost::make_shared<nav_msgs::Path>();
	local_path->header.frame_id = m_local_frame;
	local_path->header.stamp = m_odometry->header.stamp;

	// compute obstacle distance
	bool have_obstacle = false;
	double obstacle_dist = 0;
	double obstacle_cost = 0;
	{
		const double delta_move = 0.05;
		const double delta_time = start_vel_x > m_limits.trans_stopped_vel ? delta_move / start_vel_x : 0;

		tf::Pose pose = actual_pose;
		tf::Pose last_pose = pose;

		while(obstacle_dist < 10)
		{
			const double cost = compute_max_line_cost(m_cost_map, last_pose.getOrigin(), pose.getOrigin());

			bool is_contained = false;
			{
				unsigned int dummy[2] = {};
				is_contained = m_cost_map->getCostmap()->worldToMap(pose.getOrigin().x(), pose.getOrigin().y(), dummy[0], dummy[1]);
			}

			have_obstacle = cost >= m_max_cost;
			obstacle_cost = fmax(obstacle_cost, cost);

			{
				geometry_msgs::PoseStamped tmp;
				tf::poseStampedTFToMsg(tf::Stamped<tf::Pose>(pose, m_odometry->header.stamp, m_local_frame), tmp);
				local_path->poses.push_back(tmp);
			}

			if(!is_contained || have_obstacle) {
				break;
			}

			last_pose = pose;
			pose = tf::Pose(tf::createQuaternionFromYaw(tf::getYaw(pose.getRotation()) + start_yawrate * delta_time),
							pose * tf::Vector3(delta_move, 0, 0));
			obstacle_dist += delta_move;
		}
	}
	obstacle_dist -= m_min_stop_dist;

	// publish local plan
	m_local_plan_pub.publish(local_path);

	// compute situational max velocities
	const double max_trans_vel = fmax(m_limits.max_trans_vel * (m_max_cost - center_cost) / m_max_cost, m_limits.min_trans_vel);
	const double max_rot_vel = fmax(m_limits.max_rot_vel * (m_max_cost - center_cost) / m_max_cost, m_limits.min_rot_vel);

	// find closest point on path to future position
	auto iter_target = find_closest_point(local_plan.cbegin(), local_plan.cend(), actual_pos);

	// check if goal target
	bool is_goal_target = false;
	{
		// check if goal is within reach
		auto iter_next = move_along_path(iter_target, local_plan.cend(), m_max_goal_dist);
		is_goal_target = iter_next + 1 >= local_plan.cend();

		if(is_goal_target)
		{
			// go straight to goal
			iter_target = iter_next;
		}
	}

	// figure out target orientation
	double target_yaw = 0;

	if(is_goal_target)
	{
		// take goal orientation
		target_yaw = tf::getYaw(iter_target->getRotation());
	}
	else
	{
		// compute path based target orientation
		auto iter_next = move_along_path(iter_target, local_plan.cend(), lookahead_dist);
		target_yaw = ::atan2(	iter_next->getOrigin().y() - iter_target->getOrigin().y(),
								iter_next->getOrigin().x() - iter_target->getOrigin().x());
	}

	// get target position
	const tf::Vector3 target_pos = iter_target->getOrigin();

	// compute errors
	const double goal_dist = (local_plan.back().getOrigin() - actual_pos).length();
	const double yaw_error = angles::shortest_angular_distance(actual_yaw, target_yaw);
	const tf::Vector3 pos_error = tf::Pose(tf::createQuaternionFromYaw(actual_yaw), actual_pos).inverse() * target_pos;

	// compute control values
	bool is_emergency_brake = false;
	double control_vel_x = 0;
	double control_vel_y = 0;
	double control_yawrate = 0;

	if(is_goal_target)
	{
		// use term for final stopping position
		control_vel_x = pos_error.x() * m_pos_x_gain;
	}
	else
	{
		control_vel_x = max_trans_vel;

		// wait to start moving
		if(m_state != state_t::STATE_TRANSLATING && fabs(yaw_error) > m_start_yaw_error)
		{
			control_vel_x = 0;
		}

		// limit curve velocity
		{
			const double max_vel_x = m_max_curve_vel * (lookahead_dist / fabs(yaw_error));
			control_vel_x = fmin(control_vel_x, max_vel_x);
		}

		// limit velocity when approaching goal position
		if(start_vel_x > 0)
		{
			const double stop_accel = 0.8 * m_limits.acc_lim_x;
			const double stop_time = sqrt(2 * fmax(goal_dist, 0) / stop_accel);
			const double max_vel_x = fmax(stop_accel * stop_time, m_limits.min_trans_vel);

			control_vel_x = fmin(control_vel_x, max_vel_x);
		}

		// limit velocity when approaching an obstacle
		if(have_obstacle && start_vel_x > 0)
		{
			const double stop_accel = 0.9 * m_limits.acc_lim_x;
			const double stop_time = sqrt(2 * fmax(obstacle_dist, 0) / stop_accel);
			const double max_vel_x = stop_accel * stop_time;

			// check if it's much lower than current velocity
			if(max_vel_x < 0.5 * start_vel_x) {
				is_emergency_brake = true;
			}

			control_vel_x = fmin(control_vel_x, max_vel_x);
		}

		// stop before hitting obstacle
		if(have_obstacle && obstacle_dist <= 0)
		{
			control_vel_x = 0;
		}

		// only allow forward velocity in this branch
		control_vel_x = fmax(control_vel_x, 0);
	}

	// limit backing up
	if(is_goal_target && m_max_backup_dist > 0
		&& pos_error.x() < (m_state == state_t::STATE_TURNING ? 0 : -1 * m_max_backup_dist))
	{
		control_vel_x = 0;
		m_state = state_t::STATE_TURNING;
	}
	else if(m_state == state_t::STATE_TURNING)
	{
		m_state = state_t::STATE_IDLE;
	}

	if(m_differential_drive)
	{
		if(fabs(start_vel_x) > (m_state == state_t::STATE_TRANSLATING ?
								m_limits.trans_stopped_vel : 2 * m_limits.trans_stopped_vel))
		{
			// we are translating, use term for lane keeping
			control_yawrate = pos_error.y() / start_vel_x * m_pos_y_yaw_gain;

			if(!is_goal_target)
			{
				// additional term for lane keeping
				control_yawrate += yaw_error * m_yaw_gain;

				// add cost terms
				control_yawrate -= delta_cost_y / start_vel_x * m_cost_y_yaw_gain;
				control_yawrate -= delta_cost_yaw * m_cost_yaw_gain;
			}

			m_state = state_t::STATE_TRANSLATING;
		}
		else if(m_state == state_t::STATE_TURNING)
		{
			// continue on current yawrate
			control_yawrate = (start_yawrate > 0 ? 1 : -1) * max_rot_vel;
		}
		else if(is_goal_target
				&& (m_state == state_t::STATE_ADJUSTING || fabs(yaw_error) < M_PI / 6)
				&& fabs(pos_error.y()) > (m_state == state_t::STATE_ADJUSTING ?
					0.25 * m_limits.xy_goal_tolerance : 0.5 * m_limits.xy_goal_tolerance))
		{
			// we are not translating, but we have too large y error
			control_yawrate = (pos_error.y() > 0 ? 1 : -1) * max_rot_vel;

			m_state = state_t::STATE_ADJUSTING;
		}
		else
		{
			// use term for static target orientation
			control_yawrate = yaw_error * m_static_yaw_gain;

			m_state = state_t::STATE_ROTATING;
		}
	}
	else
	{
		// simply correct y with holonomic drive
		control_vel_y = pos_error.y() * m_pos_y_gain;

		if(m_state == state_t::STATE_TURNING)
		{
			// continue on current yawrate
			control_yawrate = (start_yawrate > 0 ? 1 : -1) * max_rot_vel;
		}
		else
		{
			// use term for static target orientation
			control_yawrate = yaw_error * m_static_yaw_gain;

			if(fabs(start_vel_x) > m_limits.trans_stopped_vel) {
				m_state = state_t::STATE_TRANSLATING;
			} else {
				m_state = state_t::STATE_ROTATING;
			}
		}

		// apply x cost term only when rotating and not near goal orientation
		if(m_state == state_t::STATE_ROTATING && fabs(yaw_error) > M_PI / 6)
		{
			control_vel_x -= delta_cost_x * m_cost_x_gain;
		}

		// apply y cost term when not approaching goal or if we are rotating and not near goal orientation
		if(!is_goal_target || (m_state == state_t::STATE_ROTATING && fabs(yaw_error) > M_PI / 6))
		{
			control_vel_y -= delta_cost_y * m_cost_y_gain;
		}

		// apply yaw cost term when not approaching goal
		if(!is_goal_target)
		{
			control_yawrate -= delta_cost_yaw * m_cost_yaw_gain;
		}
	}

	// check if we are stuck
	if(have_obstacle && obstacle_dist <= 0 && delta_cost_x > 0
		&& m_state == state_t::STATE_ROTATING && fabs(yaw_error) < M_PI / 6)
	{
		// we are stuck
		m_state = state_t::STATE_STUCK;

		ROS_WARN_NAMED("NeoLocalPlanner", "We are stuck: yaw_error=%f, obstacle_dist=%f, obstacle_cost=%f, delta_cost_x=%f",
						yaw_error, obstacle_dist, obstacle_cost, delta_cost_x);
		return false;
	}

	// logic check
	is_emergency_brake = is_emergency_brake && control_vel_x >= 0;

	// apply low pass filter
	control_vel_x = control_vel_x * m_low_pass_gain + m_last_control_values[0] * (1 - m_low_pass_gain);
	control_vel_y = control_vel_y * m_low_pass_gain + m_last_control_values[1] * (1 - m_low_pass_gain);
	control_yawrate = control_yawrate * m_low_pass_gain + m_last_control_values[2] * (1 - m_low_pass_gain);

	// apply acceleration limits
	control_vel_x = fmax(fmin(control_vel_x, m_last_cmd_vel.linear.x + m_limits.acc_lim_x * dt),
							m_last_cmd_vel.linear.x - (is_emergency_brake ? m_emergency_acc_lim_x : m_limits.acc_lim_x) * dt);
	control_vel_y = fmax(fmin(control_vel_y, m_last_cmd_vel.linear.y + m_limits.acc_lim_y * dt),
								m_last_cmd_vel.linear.y - m_limits.acc_lim_y * dt);

	control_yawrate = fmax(fmin(control_yawrate, m_last_cmd_vel.angular.z + m_limits.acc_lim_theta * dt),
									m_last_cmd_vel.angular.z - m_limits.acc_lim_theta * dt);

	// constrain velocity after goal reached
	if(m_constrain_final && m_is_goal_reached)
	{
		tf::Vector3 direction(m_last_control_values[0], m_last_control_values[1], m_last_control_values[2]);
		if(direction.length() != 0)
		{
			direction.normalize();
			const double dist = direction.dot(tf::Vector3(control_vel_x, control_vel_y, control_yawrate));
			const auto control = direction * dist;
			control_vel_x = control[0];
			control_vel_y = control[1];
			control_yawrate = control[2];
		}
	}

	// fill return data
	cmd_vel.linear.x = fmin(fmax(control_vel_x, m_limits.min_vel_x), m_limits.max_vel_x);
	cmd_vel.linear.y = fmin(fmax(control_vel_y, m_limits.min_vel_y), m_limits.max_vel_y);
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = fmin(fmax(control_yawrate, -m_limits.max_rot_vel), m_limits.max_rot_vel);
	// Footprint based collision avoidance
	if(m_enable_software_stop == true)
	{
		if((obstacle_in_rot == -1) && (control_yawrate- start_yawrate < start_yawrate))
		{
			ROS_WARN_THROTTLE(1, "During the rotation robot predicted an obstacle on the right! Please free the robot using Joy");
			
			cmd_vel.angular.z = 0;
		}
		else if((obstacle_in_rot == -1) && (control_yawrate- start_yawrate > start_yawrate))
		{
			ROS_WARN_THROTTLE(1, "During the rotation robot predicted an obstacle on the left! Please free the robot using Joy");

			cmd_vel.angular.z = 0;
		}
	}

	if(m_update_counter % 20 == 0) {
		ROS_INFO_NAMED("NeoLocalPlanner", "dt=%f, pos_error=(%f, %f), yaw_error=%f, cost=%f, obstacle_dist=%f, obstacle_cost=%f, delta_cost=(%f, %f, %f), state=%d, cmd_vel=(%f, %f), cmd_yawrate=%f",
						dt, pos_error.x(), pos_error.y(), yaw_error, center_cost, obstacle_dist, obstacle_cost, delta_cost_x, delta_cost_y, delta_cost_yaw, m_state, control_vel_x, control_vel_y, control_yawrate);
	}

	m_last_time = time_now;
	m_last_control_values[0] = control_vel_x;
	m_last_control_values[1] = control_vel_y;
	m_last_control_values[2] = control_yawrate;
	m_last_cmd_vel = cmd_vel;

	m_update_counter++;
	return true;
}

bool NeoLocalPlanner::isGoalReached()
{
	boost::mutex::scoped_lock lock(m_odometry_mutex);

	if(!m_odometry)
	{
		ROS_WARN_NAMED("NeoLocalPlanner", "Waiting for odometry ...");
		return false;
	}
	if(m_global_plan.empty())
	{
		ROS_WARN_NAMED("NeoLocalPlanner", "Global plan is empty!");
		return true;
	}

	tf::StampedTransform global_to_local;
	try {
		m_tf->lookupTransform(m_local_frame, m_global_frame, ros::Time(), global_to_local);
	} catch(...) {
		ROS_WARN_NAMED("NeoLocalPlanner", "lookupTransform(m_local_frame, m_global_frame) failed");
		return false;
	}

	tf::Stamped<tf::Pose> goal_pose_global;
	tf::poseStampedMsgToTF(m_global_plan.back(), goal_pose_global);
	const auto goal_pose_local = global_to_local * goal_pose_global;

	const bool is_stopped = base_local_planner::stopped(*m_odometry,
														m_limits.rot_stopped_vel,
														m_limits.trans_stopped_vel);

	const double xy_error = ::hypot(m_odometry->pose.pose.position.x - goal_pose_local.getOrigin().x(),
									m_odometry->pose.pose.position.y - goal_pose_local.getOrigin().y());

	const double yaw_error = fabs(angles::shortest_angular_distance(tf::getYaw(m_odometry->pose.pose.orientation),
																	tf::getYaw(goal_pose_local.getRotation())));

	const bool is_reached = is_stopped && xy_error < m_limits.xy_goal_tolerance && yaw_error < m_limits.yaw_goal_tolerance;

	if(!m_is_goal_reached)
	{
		if(is_reached) {
			ROS_INFO_STREAM("Goal reached: xy_error=" << xy_error << " [m], yaw_error=" << yaw_error << " [rad]");
		}
		m_first_goal_reached_time = ros::WallTime::now();
	}
	m_is_goal_reached = is_reached;

	ROS_DEBUG_NAMED("NeoLocalPlanner", "is_stopped=%d, is_reached=%d, xy_error=%f [m], yaw_error=%f [rad]",
					is_stopped, is_reached, xy_error, yaw_error);

	return is_reached && (ros::WallTime::now() - m_first_goal_reached_time).toSec() >= m_goal_tune_time;
}

bool NeoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
	m_global_plan = plan;
	m_state = state_t::STATE_IDLE;
	m_last_time = ros::WallTime::now();
	m_first_goal_reached_time = ros::WallTime();
	m_is_goal_reached = false;
	m_last_control_values[0] = 0;
	m_last_control_values[1] = 0;
	m_last_control_values[2] = 0;
	m_last_cmd_vel = geometry_msgs::Twist();
	return true;
}

void NeoLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~/" + name);

	m_limits.acc_lim_x = 			private_nh.param<double>("acc_lim_x", 0.5);
	m_limits.acc_lim_y = 			private_nh.param<double>("acc_lim_y", 0.5);
	m_limits.acc_lim_theta = 		private_nh.param<double>("acc_lim_theta", 0.5);
	m_limits.acc_limit_trans = 		private_nh.param<double>("acc_limit_trans", m_limits.acc_lim_x);
	m_limits.min_vel_x = 			private_nh.param<double>("min_vel_x", -0.1);
	m_limits.max_vel_x = 			private_nh.param<double>("max_vel_x", 0.5);
	m_limits.min_vel_y = 			private_nh.param<double>("min_vel_y", -0.5);
	m_limits.max_vel_y = 			private_nh.param<double>("max_vel_y", 0.5);
	m_limits.min_rot_vel = 			private_nh.param<double>("min_rot_vel", 0.1);
	m_limits.max_rot_vel = 			private_nh.param<double>("max_rot_vel", 0.5);
	m_limits.min_trans_vel = 		private_nh.param<double>("min_trans_vel", 0.1);
	m_limits.max_trans_vel = 		private_nh.param<double>("max_trans_vel", m_limits.max_vel_x);
	m_limits.rot_stopped_vel = 		private_nh.param<double>("rot_stopped_vel", 0.5 * m_limits.min_rot_vel);
	m_limits.trans_stopped_vel = 	private_nh.param<double>("trans_stopped_vel", 0.5 * m_limits.min_trans_vel);
	m_limits.yaw_goal_tolerance = 	private_nh.param<double>("yaw_goal_tolerance", 0.02);
	m_limits.xy_goal_tolerance = 	private_nh.param<double>("xy_goal_tolerance", 0.1);

	m_differential_drive = 	private_nh.param<bool>("differential_drive", true);
	m_constrain_final = 	private_nh.param<bool>("constrain_final", false);
	m_goal_tune_time = 		private_nh.param<double>("goal_tune_time", 0.5);
	m_lookahead_time = 		private_nh.param<double>("lookahead_time", 0.2);
	m_lookahead_dist = 		private_nh.param<double>("lookahead_dist", 0.5);
	m_start_yaw_error = 	private_nh.param<double>("start_yaw_error", 0.2);
	m_pos_x_gain = 			private_nh.param<double>("pos_x_gain", 1);
	m_pos_y_gain = 			private_nh.param<double>("pos_y_gain", 1);
	m_pos_y_yaw_gain = 		private_nh.param<double>("pos_y_yaw_gain", 1);
	m_yaw_gain = 			private_nh.param<double>("yaw_gain", 1);
	m_static_yaw_gain = 	private_nh.param<double>("static_yaw_gain", 3);
	m_cost_x_gain = 		private_nh.param<double>("cost_x_gain", 0.1);
	m_cost_y_gain = 		private_nh.param<double>("cost_y_gain", 0.1);
	m_cost_y_yaw_gain = 	private_nh.param<double>("cost_y_yaw_gain", 0.1);
	m_cost_y_lookahead_dist = 	private_nh.param<double>("cost_y_lookahead_dist", 0);
	m_cost_y_lookahead_time = 	private_nh.param<double>("cost_y_lookahead_time", 1);
	m_cost_yaw_gain = 		private_nh.param<double>("cost_yaw_gain", 1);
	m_low_pass_gain = 		private_nh.param<double>("low_pass_gain", 0.5);
	m_max_cost = 			private_nh.param<double>("max_cost", 0.9);
	m_max_curve_vel = 		private_nh.param<double>("max_curve_vel", 0.2);
	m_max_goal_dist = 		private_nh.param<double>("max_goal_dist", 0.5);
	m_max_backup_dist = 	private_nh.param<double>("max_backup_dist", m_differential_drive ? 0.1 : 0.0);
	m_min_stop_dist = 		private_nh.param<double>("min_stop_dist", 0.5);
	m_emergency_acc_lim_x = private_nh.param<double>("emergency_acc_lim_x", m_limits.acc_lim_x * 4);
	m_enable_software_stop = private_nh.param<bool>("enable_software_stop", true);

	m_tf = tf;
	m_cost_map = costmap_ros;
	m_base_frame = costmap_ros->getBaseFrameID();

	m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&NeoLocalPlanner::odomCallback, this, _1));

	m_local_plan_pub = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

	ROS_INFO_NAMED("NeoLocalPlanner", "base_frame=%s, local_frame=%s, global_frame=%s",
			m_base_frame.c_str(), m_local_frame.c_str(), m_global_frame.c_str());
}

void NeoLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(m_odometry_mutex);
	m_odometry = msg;
}


} // neo_local_planner
