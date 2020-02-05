
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
		-( dq.x()+3.0*p.x()+5.0*dp.x()+4.0*ddp.x()+-3.0*q.x())*(t*t*t)/12.0+p.x()+dp.x()*t+ddp.x()*(t*t)/2.0,
		(t*t)*ddp.y()/2.0+( 3.0*q.y()-dq.y()+-3.0*p.y()+-5.0*dp.y()+-4.0*ddp.y())*(t*t*t)/12.0+p.y()+dp.y()*t,
		0
	);
}

tf::Vector3 calc_spline_dpos_dt(const double t,
								const tf::Vector3& p, const tf::Vector3& q,
								const tf::Vector3& dp, const tf::Vector3& dq,
								const tf::Vector3& ddp)
{
	return tf::Vector3(
		-( dq.x()+3.0*p.x()+5.0*dp.x()+4.0*ddp.x()+-3.0*q.x())*(t*t)/4.0+dp.x()+ddp.x()*t,
		( 3.0*q.y()-dq.y()+-3.0*p.y()+-5.0*dp.y()+-4.0*ddp.y())*(t*t)/4.0+dp.y()+t*ddp.y(),
		0
	);
}

tf::Vector3 calc_spline_dpos_ddt(	const double t,
									const tf::Vector3& p, const tf::Vector3& q,
									const tf::Vector3& dp, const tf::Vector3& dq,
									const tf::Vector3& ddp)
{
	return tf::Vector3(
		-( dq.x()+3.0*p.x()+5.0*dp.x()+4.0*ddp.x()+-3.0*q.x())*t/2.0+ddp.x(),
		( 3.0*q.y()-dq.y()+-3.0*p.y()+-5.0*dp.y()+-4.0*ddp.y())*t/2.0+ddp.y(),
		0
	);
}

tf::Vector3 calc_spline_2_pos(	const double t,
								const tf::Vector3& p, const tf::Vector3& q,
								const tf::Vector3& dp, const tf::Vector3& dq,
								const tf::Vector3& ddp)
{
	return tf::Vector3(
		(t*t*t)*( 4.0*ddp.x()+-9.0*q.x()+7.0*dq.x()+9.0*p.x()+11.0*dp.x())/12.0+ddp.x()/6.0+q.x()/4.0+t*( 3.0*q.x()-dq.x()+-3.0*p.x()-dp.x())/4.0-(t*t)*( 2.0*ddp.x()+-3.0*q.x()+dq.x()+3.0*p.x()+5.0*dp.x())/4.0-dq.x()/12.0+(3.0/4.0)*p.x()+(7.0/12.0)*dp.x(),
		(3.0/4.0)*p.y()-( 3.0*p.y()+dp.y()+-3.0*q.y()+dq.y())*t/4.0+(7.0/12.0)*dp.y()+(t*t*t)*( 9.0*p.y()+11.0*dp.y()+4.0*ddp.y()+-9.0*q.y()+7.0*dq.y())/12.0+ddp.y()/6.0+q.y()/4.0-( 3.0*p.y()+5.0*dp.y()+2.0*ddp.y()+-3.0*q.y()+dq.y())*(t*t)/4.0-dq.y()/12.0,
		0
	);
}

tf::Vector3 calc_spline_2_dpos_dt(	const double t,
									const tf::Vector3& p, const tf::Vector3& q,
									const tf::Vector3& dp, const tf::Vector3& dq,
									const tf::Vector3& ddp)
{
	return tf::Vector3(
		(3.0/4.0)*q.x()+(t*t)*( 4.0*ddp.x()+-9.0*q.x()+7.0*dq.x()+9.0*p.x()+11.0*dp.x())/4.0-t*( 2.0*ddp.x()+-3.0*q.x()+dq.x()+3.0*p.x()+5.0*dp.x())/2.0-dq.x()/4.0+-(3.0/4.0)*p.x()-dp.x()/4.0,
		-(3.0/4.0)*p.y()-t*( 3.0*p.y()+5.0*dp.y()+2.0*ddp.y()+-3.0*q.y()+dq.y())/2.0-dp.y()/4.0+(3.0/4.0)*q.y()+( 9.0*p.y()+11.0*dp.y()+4.0*ddp.y()+-9.0*q.y()+7.0*dq.y())*(t*t)/4.0-dq.y()/4.0,
		0
	);
}

NeoLocalPlanner::NeoLocalPlanner()
{
	m_limits.max_vel_x = 1;
	m_limits.min_trans_vel = 0.1;
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
		// TODO: global path doesnt have orientation
	}

	// compute target velocity
	double target_vel = 0;

	if(iter_target + 1 != local_plan.end()) {
		target_vel = m_limits.max_vel_x;
	}

	// compute spline based control values
	const double start_vel_spline = fmax(start_vel, m_limits.min_trans_vel);		// cannot have zero velocity
	const double target_vel_spline = fmax(target_vel, m_limits.min_trans_vel);		// cannot have zero velocity

	const tf::Vector3 p = local_pose.getOrigin();
	const tf::Vector3 q = iter_target->getOrigin();

	const double total_dist = (q - p).length();
	const double time_to_target = total_dist / ((start_vel_spline + target_vel_spline) / 2);
	const double time_factor = 2 / time_to_target;		// we have two splines to reach target

	const tf::Vector3 dp = local_pose.getBasis() * tf::Vector3(start_vel_spline * time_factor, 0, 0);
	const tf::Vector3 dq = tf::Matrix3x3(tf::createQuaternionFromYaw(target_yaw)) * tf::Vector3(target_vel_spline * time_factor, 0, 0);

	const tf::Vector3 ddp = tf::Vector3();	// TODO

	const double dt = (ros::Time::now() - m_odometry->header.stamp).toSec() + m_control_delay;
	const double ds = start_vel_spline	* dt;

	// find control point on spline
	double t = 0;
	{
		double dist = 0;
		tf::Vector3 last_pos = p;

		for(int i = 1; i <= 1000; ++i)
		{
			t = i * 0.001;
			const tf::Vector3 pos = calc_spline_pos(t, p, q, dp, dq, ddp);
			dist += (pos - last_pos).length();
			last_pos = pos;

			if(dist >= ds) {
				break;
			}
		}
	}

	// compute control velocity
	const tf::Vector3 dpos_dt = calc_spline_dpos_dt(t, p, q, dp, dq, ddp);
	const double control_vel = dpos_dt.length() / time_factor;

	// compute control yawrate
	// const tf::Vector3 dpos_ddt = calc_spline_dpos_ddt(t, p, q, dp, dq, ddp);
	// const double control_yawrate = ::atan2(dpos_ddt.y(), dpos_ddt.x()) / time_factor;
	const tf::Vector3 dpos_dt_2 = calc_spline_dpos_dt(t * 2, p, q, dp, dq, ddp);
	const double control_yaw = ::atan2(dpos_dt_2.y(), dpos_dt_2.x());
	const double delta_yaw = angles::shortest_angular_distance(start_yaw, control_yaw);
	const double control_yawrate = (delta_yaw / (2 * dt)) * m_yaw_gain;
	// const tf::Vector3 dpos_dt_2 = calc_spline_dpos_dt(t + 0.001, p, q, dp, dq, ddp);
	// const double control_yawrate = angles::shortest_angular_distance(::atan2(dpos_dt.y(), dpos_dt.x()), ::atan2(dpos_dt_2.y(), dpos_dt_2.x()))
	// 								/ 0.001 / time_factor;

	cmd_vel.linear.x = control_vel;
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = control_yawrate;

	// debug output
	ROS_INFO_NAMED("NeoLocalPlanner", "dist_short=%f, total_dist=%f, time_to_target=%f, dt=%f, ds=%f, t=%f, cmd_vel=%f, cmd_yawrate=%f, target_yaw=%f",
					dist_short, total_dist, time_to_target, dt, ds, t, control_vel, control_yawrate, target_yaw);

	// visualize first spline
	{
		nav_msgs::Path path;
		path.header.stamp = m_odometry->header.stamp;
		path.header.frame_id = m_local_frame;

		for(int i = 1; i <= 200; ++i)
		{
			const double t_i = i * 0.01;
			tf::Vector3 pos;
			tf::Vector3 dpos_dt;
			if(t_i <= 1) {
				pos = calc_spline_pos(t_i, p, q, dp, dq, ddp);
				dpos_dt = calc_spline_dpos_dt(t_i, p, q, dp, dq, ddp);
			} else {
				pos = calc_spline_2_pos(t_i - 1, p, q, dp, dq, ddp);
				dpos_dt = calc_spline_2_dpos_dt(t_i - 1, p, q, dp, dq, ddp);
			}

			tf::Stamped<tf::Pose> pose;
			pose.stamp_ = m_odometry->header.stamp + ros::Duration(t_i / time_factor);
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
