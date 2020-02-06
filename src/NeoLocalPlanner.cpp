
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

tf::Vector3 calc_spline_pos(const double t,
							const tf::Vector3& p, const tf::Vector3& q,
							const tf::Vector3& dp, const tf::Vector3& dq,
							const tf::Vector3& ddp)
{
	return tf::Vector3(
		p.x()-( 6.0*p.x()+12.0*dp.x()+11.0*ddp.x()+-6.0*q.x()+6.0*dq.x())*(t*t*t)/36.0+ddp.x()*(t*t)/2.0+dp.x()*t,
		(t*t)*ddp.y()/2.0-(t*t*t)*( 6.0*dq.y()+6.0*p.y()+12.0*dp.y()+11.0*ddp.y()+-6.0*q.y())/36.0+p.y()+dp.y()*t,
		0
	);
}

tf::Vector3 calc_spline_dpos_dt(const double t,
								const tf::Vector3& p, const tf::Vector3& q,
								const tf::Vector3& dp, const tf::Vector3& dq,
								const tf::Vector3& ddp)
{
	return tf::Vector3(
		dp.x()-( 6.0*p.x()+12.0*dp.x()+11.0*ddp.x()+-6.0*q.x()+6.0*dq.x())*(t*t)/12.0+ddp.x()*t,
		-(t*t)*( 6.0*dq.y()+6.0*p.y()+12.0*dp.y()+11.0*ddp.y()+-6.0*q.y())/12.0+dp.y()+t*ddp.y(),
		0
	);
}

tf::Vector3 calc_spline_dpos_ddt(	const double t,
									const tf::Vector3& p, const tf::Vector3& q,
									const tf::Vector3& dp, const tf::Vector3& dq,
									const tf::Vector3& ddp)
{
	return tf::Vector3(
		-( 6.0*p.x()+12.0*dp.x()+11.0*ddp.x()+-6.0*q.x()+6.0*dq.x())*t/6.0+ddp.x(),
		ddp.y()-t*( 6.0*dq.y()+6.0*p.y()+12.0*dp.y()+11.0*ddp.y()+-6.0*q.y())/6.0,
		0
	);
}

tf::Vector3 calc_spline_1_pos(	const double t,
								const tf::Vector3& p, const tf::Vector3& q,
								const tf::Vector3& dp, const tf::Vector3& dq,
								const tf::Vector3& ddp)
{
	return tf::Vector3(
		( 12.0*p.x()+18.0*dp.x()+7.0*ddp.x()+-12.0*q.x()+18.0*dq.x())*(t*t*t)/36.0+(5.0/6.0)*p.x()+(2.0/3.0)*dp.x()+(7.0/36.0)*ddp.x()-t*( 6.0*p.x()-ddp.x()+-6.0*q.x()+6.0*dq.x())/12.0+q.x()/6.0-dq.x()/6.0-( 6.0*p.x()+12.0*dp.x()+5.0*ddp.x()+-6.0*q.x()+6.0*dq.x())*(t*t)/12.0,
		-( 6.0*dq.y()+6.0*p.y()-ddp.y()+-6.0*q.y())*t/12.0-dq.y()/6.0+(5.0/6.0)*p.y()+(2.0/3.0)*dp.y()-( 6.0*dq.y()+6.0*p.y()+12.0*dp.y()+5.0*ddp.y()+-6.0*q.y())*(t*t)/12.0+(7.0/36.0)*ddp.y()+( 18.0*dq.y()+12.0*p.y()+18.0*dp.y()+7.0*ddp.y()+-12.0*q.y())*(t*t*t)/36.0+q.y()/6.0,
		0
	);
}

tf::Vector3 calc_spline_1_dpos_dt(	const double t,
									const tf::Vector3& p, const tf::Vector3& q,
									const tf::Vector3& dp, const tf::Vector3& dq,
									const tf::Vector3& ddp)
{
	return tf::Vector3(
		-p.x()/2.0+( 12.0*p.x()+18.0*dp.x()+7.0*ddp.x()+-12.0*q.x()+18.0*dq.x())*(t*t)/12.0-( 6.0*p.x()+12.0*dp.x()+5.0*ddp.x()+-6.0*q.x()+6.0*dq.x())*t/6.0+ddp.x()/12.0+q.x()/2.0-dq.x()/2.0,
		-dq.y()/2.0-p.y()/2.0+( 18.0*dq.y()+12.0*p.y()+18.0*dp.y()+7.0*ddp.y()+-12.0*q.y())*(t*t)/12.0+ddp.y()/12.0-( 6.0*dq.y()+6.0*p.y()+12.0*dp.y()+5.0*ddp.y()+-6.0*q.y())*t/6.0+q.y()/2.0,
		0
	);
}

tf::Vector3 calc_spline_2_pos(	const double t,
								const tf::Vector3& p, const tf::Vector3& q,
								const tf::Vector3& dp, const tf::Vector3& dq,
								const tf::Vector3& ddp)
{
	return tf::Vector3(
		p.x()/6.0-(t*t*t)*( 3.0*p.x()+3.0*dp.x()+ddp.x()+-3.0*q.x()+6.0*dq.x())/18.0+dp.x()/6.0+(t*t)*( 3.0*p.x()+3.0*dp.x()+ddp.x()+-3.0*q.x()+6.0*dq.x())/6.0+ddp.x()/18.0-( 3.0*p.x()+3.0*dp.x()+ddp.x()+-3.0*q.x())*t/6.0+(5.0/6.0)*q.x()+-(2.0/3.0)*dq.x(),
		-(2.0/3.0)*dq.y()+( 6.0*dq.y()+3.0*p.y()+3.0*dp.y()+ddp.y()+-3.0*q.y())*(t*t)/6.0+p.y()/6.0-( 6.0*dq.y()+3.0*p.y()+3.0*dp.y()+ddp.y()+-3.0*q.y())*(t*t*t)/18.0-( 3.0*p.y()+3.0*dp.y()+ddp.y()+-3.0*q.y())*t/6.0+dp.y()/6.0+ddp.y()/18.0+(5.0/6.0)*q.y(),
		0
	);
}

tf::Vector3 calc_spline_2_dpos_dt(	const double t,
									const tf::Vector3& p, const tf::Vector3& q,
									const tf::Vector3& dp, const tf::Vector3& dq,
									const tf::Vector3& ddp)
{
	return tf::Vector3(
		-p.x()/2.0-dp.x()/2.0-(t*t)*( 3.0*p.x()+3.0*dp.x()+ddp.x()+-3.0*q.x()+6.0*dq.x())/6.0-ddp.x()/6.0+t*( 3.0*p.x()+3.0*dp.x()+ddp.x()+-3.0*q.x()+6.0*dq.x())/3.0+q.x()/2.0,
		( 6.0*dq.y()+3.0*p.y()+3.0*dp.y()+ddp.y()+-3.0*q.y())*t/3.0-( 6.0*dq.y()+3.0*p.y()+3.0*dp.y()+ddp.y()+-3.0*q.y())*(t*t)/6.0-p.y()/2.0-dp.y()/2.0-ddp.y()/6.0+q.y()/2.0,
		0
	);
}

NeoLocalPlanner::NeoLocalPlanner()
{
	m_limits.min_vel_x = -0.2;
	m_limits.max_vel_x = 1;
	m_limits.min_rot_vel = 0.1;
	m_limits.max_rot_vel = 1;
	m_limits.min_trans_vel = 0.1;
	m_limits.max_trans_vel = 1;
	m_limits.rot_stopped_vel = 0.01;
	m_limits.trans_stopped_vel = 0.1;
	m_limits.yaw_goal_tolerance = 0.05;
	m_limits.xy_goal_tolerance = 0.1;
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

	// get latest global to local transform
	tf::StampedTransform global_to_local;
	try {
		m_tf->lookupTransform(m_local_frame, m_global_frame, ros::Time(), global_to_local);
	} catch(...) {
		ROS_WARN_NAMED("NeoLocalPlanner", "lookupTransform(m_local_frame, m_global_frame) failed");
		return false;
	}

	// get latest local pose
	tf::Pose local_pose;
	tf::poseMsgToTF(m_odometry->pose.pose, local_pose);

	const double start_yaw = tf::getYaw(local_pose.getRotation());
	const double start_vel = m_odometry->twist.twist.linear.x;
	const double start_yawrate = m_odometry->twist.twist.angular.z;

	// transform plan
	std::vector<tf::Pose> local_plan;
	for(const auto& pose : m_global_plan)
	{
		tf::Stamped<tf::Pose> pose_;
		tf::poseStampedMsgToTF(pose, pose_);
		local_plan.push_back(global_to_local * pose_);
	}

	// find closest point
	double dist_short = std::numeric_limits<double>::infinity();
	auto iter_short = local_plan.begin();

	for(auto iter = iter_short; iter != local_plan.end(); ++iter)
	{
		const double dist = (iter->getOrigin() - local_pose.getOrigin()).length();
		if(dist < dist_short)
		{
			dist_short = dist;
			iter_short = iter;
		}
	}

	// compute look-ahead distance
	const double lookahead_dist = m_lookahead_dist + start_vel * m_lookahead_time;

	// find look-ahead position (target)
	double target_dist = 0;
	const auto iter_target = move_along_path(iter_short, local_plan.end(), lookahead_dist, &target_dist);

	// find orientation target
	const auto iter_target_2 = move_along_path(iter_target, local_plan.end(), lookahead_dist);

	// check if goal target
	const bool is_goal_target = iter_target + 1 >= local_plan.end();

	// compute target orientation
	double target_yaw = tf::getYaw(iter_target->getRotation());

	// if target is not goal, compute kinematics based target orientation
	if(!is_goal_target)
	{
		target_yaw = ::atan2(	iter_target_2->getOrigin().y() - iter_target->getOrigin().y(),
								iter_target_2->getOrigin().x() - iter_target->getOrigin().x());
	}

	// compute target velocity
	double target_vel = 0;

	if(iter_target + 1 != local_plan.end()) {
		target_vel = m_limits.max_trans_vel;
	}

	// compute spline based control values
	const double start_vel_spline = fmax(start_vel, m_limits.min_trans_vel);		// cannot have zero velocity
	const double target_vel_spline = fmax(target_vel, m_limits.min_trans_vel);		// cannot have zero velocity

	const tf::Vector3 p = local_pose.getOrigin();
	const tf::Vector3 q = iter_target->getOrigin();

	const double total_pos_dist = (q - p).length();
	const double total_yaw_dist = fabs(angles::shortest_angular_distance(tf::getYaw(local_pose.getRotation()), target_yaw));
	const double time_to_target = fmax(fmax(m_min_move_time,
											total_pos_dist / ((fabs(start_vel) + m_limits.max_trans_vel + fabs(target_vel)) / 3)),
											total_yaw_dist / ((fabs(start_yawrate) + m_limits.max_rot_vel + 0) / 3));
	const double time_factor = 2 / time_to_target;		// we have two splines to reach target

	const tf::Vector3 dp = local_pose.getBasis() * tf::Vector3(start_vel_spline * time_factor, 0, 0);
	const tf::Vector3 dq = tf::Matrix3x3(tf::createQuaternionFromYaw(target_yaw)) * tf::Vector3(target_vel_spline * time_factor, 0, 0);

	const tf::Vector3 ddp = tf::Vector3();

	const double dt = (ros::Time::now() - m_odometry->header.stamp).toSec() + m_control_delay;
	const double s = fmin(dt * time_factor, 2);

	// predict future pose (using second order midpoint method)
	tf::Vector3 actual_pos;
	double actual_yaw = 0;
	{
		const double midpoint_yaw = start_yaw + start_yawrate * dt / 2;
		actual_pos = p + tf::Matrix3x3(tf::createQuaternionFromYaw(midpoint_yaw)) * tf::Vector3(start_vel * dt, 0, 0);
		actual_yaw = start_yaw + start_yawrate * dt;
	}

	// compute desired position, velocity and orientation
	tf::Vector3 desired_pos;
	tf::Vector3 desired_dpos_dt;
	// if(s <= 1) {
	// 	desired_pos = calc_spline_pos(s, p, q, dp, dq, ddp);
	// 	desired_dpos_dt = calc_spline_dpos_dt(s, p, q, dp, dq, ddp);
	// } else if(s <= 2) {
	// 	desired_pos = calc_spline_1_pos(s - 1, p, q, dp, dq, ddp);
	// 	desired_dpos_dt = calc_spline_1_dpos_dt(s - 1, p, q, dp, dq, ddp);
	// } else {
	// 	desired_pos = calc_spline_2_pos(s - 2, p, q, dp, dq, ddp);
	// 	desired_dpos_dt = calc_spline_2_dpos_dt(s - 2, p, q, dp, dq, ddp);
	// }
	// const double desired_vel = desired_dpos_dt.length() / time_factor;
	// const double desired_yaw = ::atan2(desired_dpos_dt.y(), desired_dpos_dt.x());

	desired_pos = iter_target->getOrigin();
	const double desired_vel = target_vel;
	const double desired_yaw = target_yaw;

	// compute control values
	const tf::Vector3 pos_error = tf::Pose(tf::createQuaternionFromYaw(actual_yaw), actual_pos).inverse() * desired_pos;
	const double yaw_error = angles::shortest_angular_distance(actual_yaw, desired_yaw);
	const double control_vel = desired_vel + pos_error.x() * m_pos_gain;
	const double control_yawrate = yaw_error * m_yaw_gain + (fabs(start_vel) >= m_limits.min_trans_vel ? pos_error.y() / start_vel * m_yaw_pos_gain : 0);

	cmd_vel.linear.x = fmin(fmax(control_vel, m_limits.min_vel_x), m_limits.max_vel_x);
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = fmin(fmax(control_yawrate, -m_limits.max_rot_vel), m_limits.max_rot_vel);

	ROS_INFO_NAMED("NeoLocalPlanner", "time_to_target=%f, target_yaw=%f, dt=%f, s=%f, pos_error=(%f, %f), yaw_error=%f, cmd_vel=%f, cmd_yawrate=%f",
					time_to_target, target_yaw, dt, s, pos_error.x(), pos_error.y(), yaw_error, control_vel, control_yawrate);

	// visualize spline
	{
		nav_msgs::Path path;
		path.header.stamp = m_odometry->header.stamp;
		path.header.frame_id = m_local_frame;

		for(int i = 1; i <= 300; ++i)
		{
			const double s_i = i * 0.01;
			tf::Vector3 pos;
			tf::Vector3 dpos_dt;
			if(s_i <= 1) {
				pos = calc_spline_pos(s_i, p, q, dp, dq, ddp);
				dpos_dt = calc_spline_dpos_dt(s_i, p, q, dp, dq, ddp);
			} else if(s_i <= 2) {
				pos = calc_spline_1_pos(s_i - 1, p, q, dp, dq, ddp);
				dpos_dt = calc_spline_1_dpos_dt(s_i - 1, p, q, dp, dq, ddp);
			} else {
				pos = calc_spline_2_pos(s_i - 2, p, q, dp, dq, ddp);
				dpos_dt = calc_spline_2_dpos_dt(s_i - 2, p, q, dp, dq, ddp);
			}

			tf::Stamped<tf::Pose> pose;
			pose.stamp_ = m_odometry->header.stamp + ros::Duration(s_i / time_factor);
			pose.frame_id_ = m_local_frame;
			pose.setOrigin(pos);
			pose.setRotation(tf::createQuaternionFromYaw(::atan2(dpos_dt.y(), dpos_dt.x())));

			geometry_msgs::PoseStamped pose_;
			tf::poseStampedTFToMsg(pose, pose_);
			path.poses.push_back(pose_);
		}
		m_local_plan_pub.publish(path);
	}

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
	m_global_frame = costmap_ros->getGlobalFrameID();
	m_base_frame = costmap_ros->getBaseFrameID();

	m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&NeoLocalPlanner::odomCallback, this, _1));

	m_local_plan_pub = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

	ROS_INFO_NAMED("NeoLocalPlanner", "base_frame=%s, global_frame=%s", m_base_frame.c_str(), m_local_frame.c_str());
}

void NeoLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(m_odometry_mutex);
	m_odometry = msg;
}


} // neo_local_planner
