
#include "../include/NeoLocalPlanner.h"

#include <tf/transform_datatypes.h>
#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>

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
	m_last_time = time_now;

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

	// predict future pose (using second order midpoint method)
	tf::Vector3 actual_pos;
	double actual_yaw = 0;
	{
		const double midpoint_yaw = start_yaw + start_yawrate * m_lookahead_time / 2;
		actual_pos = local_pose.getOrigin() + tf::Matrix3x3(tf::createQuaternionFromYaw(midpoint_yaw))
												* tf::Vector3(start_vel_x * m_lookahead_time, 0, 0);
		actual_yaw = start_yaw + start_yawrate * m_lookahead_time;
	}

	// find closest point on path to future position
	auto iter_target = find_closest_point(local_plan.cbegin(), local_plan.cend(), actual_pos);

	// check if goal target
	bool is_goal_target = iter_target + 1 >= local_plan.cend();

	// figure out target orientation
	double target_yaw = 0;

	if(!is_goal_target)
	{
		// compute path based target orientation
		auto iter_next = move_along_path(iter_target, local_plan.cend(), m_lookahead_dist);
		target_yaw = ::atan2(	iter_next->getOrigin().y() - iter_target->getOrigin().y(),
								iter_next->getOrigin().x() - iter_target->getOrigin().x());

		// re-check if goal
		is_goal_target = iter_next + 1 >= local_plan.cend();

		if(is_goal_target)
		{
			// go straight to goal
			iter_target = iter_next;
		}
	}

	if(is_goal_target)
	{
		// take goal orientation
		target_yaw = tf::getYaw(iter_target->getRotation());
	}

	// get target position
	const tf::Vector3 target_pos = iter_target->getOrigin();

	// compute errors
	const double goal_dist = (local_plan.back().getOrigin() - local_pose.getOrigin()).length();
	const double yaw_error = angles::shortest_angular_distance(actual_yaw, target_yaw);
	const tf::Vector3 pos_error = tf::Pose(tf::createQuaternionFromYaw(actual_yaw), actual_pos).inverse() * target_pos;

	// compute control values
	double control_vel_x = 0;
	double control_vel_y = 0;
	double control_yawrate = 0;
	bool do_turn_around = false;

	if(is_goal_target)
	{
		// use term for final stopping position
		control_vel_x = pos_error.x() * m_pos_x_gain;

		// limit backing up
		if(m_max_backup_dist > 0 && pos_error.x() < -1 * m_max_backup_dist)
		{
			control_vel_x = 0;
			do_turn_around = true;
		}
	}
	else
	{
		// use term for lane keeping
		const double y_factor = fmax(1 - fabs(pos_error.y()) / m_max_y_error, 0);
		const double yaw_factor = fmax(1 - fabs(yaw_error) / m_max_yaw_error, 0);
		control_vel_x = m_limits.max_trans_vel * fmax(fmin(y_factor, yaw_factor), 0);
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
			}

			m_state = state_t::STATE_TRANSLATING;
		}
		else if(fabs(pos_error.y()) > (m_state == state_t::STATE_ADJUSTING ?
										0.35 * m_limits.xy_goal_tolerance : 0.7 * m_limits.xy_goal_tolerance))
		{
			// we are not translating, use term for static y error
			control_yawrate = (pos_error.y() > 0 ? 1 : -1) * m_limits.max_rot_vel;

			m_state = state_t::STATE_ADJUSTING;
		}
		else if(do_turn_around)
		{
			// continue on current yawrate
			control_yawrate = (start_yawrate > 0 ? 1 : -1) * m_limits.max_rot_vel;

			m_state = state_t::STATE_ROTATING;
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

		// use term for static target orientation
		control_yawrate = yaw_error * m_static_yaw_gain;

		if(fabs(start_vel_x) > m_limits.trans_stopped_vel) {
			m_state = state_t::STATE_TRANSLATING;
		} else {
			m_state = state_t::STATE_ROTATING;
		}
	}

	// limit curve velocity
	if(control_vel_x > 0) {
		control_vel_x = fmin(control_vel_x, m_max_curve_vel / fabs(control_yawrate));
	} else {
		control_vel_x = fmax(control_vel_x, -1 * m_max_curve_vel / fabs(control_yawrate));
	}

	// apply acceleration limits
	control_vel_x = fmin(control_vel_x, start_vel_x + m_limits.acc_lim_x * dt);

	if(control_vel_x > 0 && start_vel_x > 0)
	{
		// limit velocity when approaching goal position
		const double max_vel_x = goal_dist / start_vel_x * m_limits.acc_lim_x;
		control_vel_x = fmin(control_vel_x, max_vel_x);
	}

	// limit yaw acceleration in certain states
	switch(m_state)
	{
		case state_t::STATE_ADJUSTING:
		case state_t::STATE_ROTATING:
			if(control_yawrate > 0) {
				control_yawrate = fmin(control_yawrate, start_yawrate + m_limits.acc_lim_theta * dt);
			} else {
				control_yawrate = fmax(control_yawrate, start_yawrate - m_limits.acc_lim_theta * dt);
			}
			break;
	}

	// fill return data
	cmd_vel.linear.x = fmin(fmax(control_vel_x, m_limits.min_vel_x), m_limits.max_vel_x);
	cmd_vel.linear.y = fmin(fmax(control_vel_y, m_limits.min_vel_y), m_limits.max_vel_y);
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = fmin(fmax(control_yawrate, -m_limits.max_rot_vel), m_limits.max_rot_vel);

	ROS_INFO_NAMED("NeoLocalPlanner", "dt=%f, target_yaw=%f, pos_error=(%f, %f), yaw_error=%f, do_turn=%d, cmd_vel=%f, cmd_yawrate=%f",
					dt, target_yaw, pos_error.x(), pos_error.y(), yaw_error, do_turn_around, control_vel_x, control_yawrate);

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

	ROS_INFO_NAMED("NeoLocalPlanner", "is_stopped=%d, xy_error=%f [m], yaw_error=%f [rad]", is_stopped, xy_error, yaw_error);

	return is_stopped && xy_error < m_limits.xy_goal_tolerance && yaw_error < m_limits.yaw_goal_tolerance;
}

bool NeoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
	m_global_plan = plan;
	m_state = state_t::STATE_IDLE;
	m_last_time = ros::WallTime::now();
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
	m_limits.xy_goal_tolerance = 	private_nh.param<double>("xy_goal_tolerance", 0.05);

	m_lookahead_time = 		private_nh.param<double>("lookahead_time", 0.2);
	m_lookahead_dist = 		private_nh.param<double>("lookahead_dist", 0.5);
	m_max_y_error = 		private_nh.param<double>("max_y_error", 0.2);
	m_max_yaw_error = 		private_nh.param<double>("max_yaw_error", 0.5);
	m_pos_x_gain = 			private_nh.param<double>("pos_x_gain", 1);
	m_pos_y_gain = 			private_nh.param<double>("pos_y_gain", 1);
	m_pos_y_yaw_gain = 		private_nh.param<double>("pos_y_yaw_gain", 1);
	m_yaw_gain = 			private_nh.param<double>("yaw_gain", 1);
	m_static_yaw_gain = 	private_nh.param<double>("static_yaw_gain", 3);
	m_max_curve_vel = 		private_nh.param<double>("max_curve_vel", m_limits.min_trans_vel * m_limits.max_rot_vel);
	m_max_backup_dist = 	private_nh.param<double>("max_backup_dist", 0.1);
	m_differential_drive = 	private_nh.param<bool>("differential_drive", true);

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
