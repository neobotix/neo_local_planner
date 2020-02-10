
#include "../include/NeoLocalPlanner.h"

#include <tf/transform_datatypes.h>
#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
// (see http://www.ros.org/wiki/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
PLUGINLIB_EXPORT_CLASS(neo_local_planner::NeoLocalPlanner, nav_core::BaseLocalPlanner)


namespace neo_local_planner {

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
	m_limits.min_vel_x = -0.1;
	m_limits.max_vel_x = 0.5;
	m_limits.min_rot_vel = 0.1;
	m_limits.max_rot_vel = 0.5;
	m_limits.min_trans_vel = 0.1;
	m_limits.max_trans_vel = m_limits.max_vel_x;
	m_limits.rot_stopped_vel = 0.2 * m_limits.min_rot_vel;
	m_limits.trans_stopped_vel = 0.2 * m_limits.min_trans_vel;
	m_limits.yaw_goal_tolerance = 0.02;
	m_limits.xy_goal_tolerance = 0.05;
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
	const double start_vel = m_odometry->twist.twist.linear.x;
	const double start_yawrate = m_odometry->twist.twist.angular.z;

	// predict future pose (using second order midpoint method)
	tf::Vector3 actual_pos;
	double actual_yaw = 0;
	{
		const double midpoint_yaw = start_yaw + start_yawrate * m_lookahead_time / 2;
		actual_pos = local_pose.getOrigin() + tf::Matrix3x3(tf::createQuaternionFromYaw(midpoint_yaw))
												* tf::Vector3(start_vel * m_lookahead_time, 0, 0);
		actual_yaw = start_yaw + start_yawrate * m_lookahead_time;
	}

	// find closest point on path to future position
	auto iter_target = local_plan.cbegin();
	{
		double dist_short = std::numeric_limits<double>::infinity();

		for(auto iter = iter_target; iter != local_plan.cend(); ++iter)
		{
			const double dist = (iter->getOrigin() - actual_pos).length();
			if(dist < dist_short)
			{
				dist_short = dist;
				iter_target = iter;
			}
		}
	}

	// check if goal target
	bool is_goal_target = iter_target + 1 >= local_plan.cend();

	// if target is not goal, compute path based target orientation
	double target_yaw = 0;

	if(!is_goal_target)
	{
		auto iter_next = move_along_path(iter_target, local_plan.cend(), m_lookahead_dist);
		target_yaw = ::atan2(	iter_next->getOrigin().y() - iter_target->getOrigin().y(),
								iter_next->getOrigin().x() - iter_target->getOrigin().x());

		// reset target and re-check if goal now
		iter_target = iter_next;
		is_goal_target = iter_target + 1 >= local_plan.cend();
	}

	// compute target position and orientation
	const tf::Vector3 target_pos = iter_target->getOrigin();

	if(is_goal_target) {
		target_yaw = tf::getYaw(iter_target->getRotation());
	}

	// compute errors
	const double yaw_error = angles::shortest_angular_distance(actual_yaw, target_yaw);
	const double xy_error = (local_pose.getOrigin() - iter_target->getOrigin()).length();
	const tf::Vector3 pos_error = tf::Pose(tf::createQuaternionFromYaw(actual_yaw), actual_pos).inverse() * target_pos;

	// compute control values
	double control_vel = 0;
	double control_yawrate = 0;

	if(is_goal_target)
	{
		// use term for final stopping position
		control_vel = pos_error.x() * m_pos_x_gain;
	}
	else
	{
		// use term for lane keeping
		control_vel = fmax(m_limits.max_trans_vel * (1 - fabs(yaw_error) / m_max_yaw_error), 0);
	}

	if(fabs(start_vel) > m_limits.trans_stopped_vel)
	{
		// we are translating, use term for lane keeping
		control_yawrate = pos_error.y() / start_vel * m_pos_y_gain;
	}
	else if(is_goal_target && xy_error > m_limits.xy_goal_tolerance)
	{
		// we are not translating but goal not reached either, use term for static y error
		control_yawrate = (pos_error.y() > 0 ? 1 : -1) * m_limits.max_rot_vel;
	}
	else
	{
		// use term for target orientation
		control_yawrate = yaw_error * m_yaw_gain;
	}

	// ensure minimum translational velocity
	// if(control_vel > 0) {
	// 	control_vel = fmax(control_vel, m_limits.min_trans_vel);
	// }
	// if(control_vel < 0) {
	// 	control_vel = fmin(control_vel, -1 * m_limits.min_trans_vel);
	// }

	// ensure minimum rotational velocity
	// if(control_yawrate > 0) {
	// 	control_yawrate = fmax(control_yawrate, m_limits.min_rot_vel);
	// }
	// if(control_yawrate < 0) {
	// 	control_yawrate = fmin(control_yawrate, -1 * m_limits.min_rot_vel);
	// }

	cmd_vel.linear.x = fmin(fmax(control_vel, m_limits.min_vel_x), m_limits.max_vel_x);
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = fmin(fmax(control_yawrate, -m_limits.max_rot_vel), m_limits.max_rot_vel);

	ROS_INFO_NAMED("NeoLocalPlanner", "target_yaw=%f, pos_error=(%f, %f), yaw_error=%f, cmd_vel=%f, cmd_yawrate=%f",
					target_yaw, pos_error.x(), pos_error.y(), yaw_error, control_vel, control_yawrate);

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
}

void NeoLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~/" + name);

	m_tf = tf;
	m_cost_map = costmap_ros;
	m_local_frame = costmap_ros->getGlobalFrameID();
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
