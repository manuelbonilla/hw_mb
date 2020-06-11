#include "hw_mb/robot_definition.h"
#include "utils/pseudo_inversion.h"
#include "utils/skew_symmetric.h"
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>


bool UR10::init(ros::NodeHandle &n){

	nh_ = n;
	string robot_description, root_name, tip_name;

	if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
	{
		ROS_ERROR_STREAM("No robot description (URDF) found on parameter server ("<<nh_.getNamespace()<<"/robot_description)");
		return false;
	}

	if (!nh_.getParam("root_name", root_name))
	{
		ROS_ERROR_STREAM("No root name found on parameter server ("<<nh_.getNamespace()<<"/root_name)");
		return false;
	}

	if (!nh_.getParam("tip_name", tip_name))
	{
		ROS_ERROR_STREAM("No tip name found on parameter server ("<<nh_.getNamespace()<<"/tip_name)");
		return false;
	}


// Construct an URDF model from the xml string
	std::string xml_string;

	if (nh_.hasParam(robot_description))
		nh_.getParam(robot_description.c_str(), xml_string);
	else
	{
		ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
		nh_.shutdown();
		return false;
	}

	if (xml_string.size() == 0)
	{
		ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
		nh_.shutdown();
		return false;
	}

	ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

// Get urdf model out of robot_description
	urdf::Model model;
	if (!model.initString(xml_string))
	{
		ROS_ERROR("Failed to parse urdf file");
		nh_.shutdown();
		return false;
	}
	ROS_INFO("Successfully parsed urdf file");

	KDL::Tree kdl_tree_;
	if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
	{
		ROS_ERROR("Failed to construct kdl tree");
		nh_.shutdown();
		return false;
	}

// Populate the KDL chain
	if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
	{
		ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
		ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
		ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
		ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
		ROS_ERROR_STREAM("  The segments are:");

		KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		KDL::SegmentMap::iterator it;

		for( it=segment_map.begin(); it != segment_map.end(); it++ )
			ROS_ERROR_STREAM( "    "<<(*it).first);

		return false;
	}

	ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());
	ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

// Parsing joint limits from urdf model along kdl chain
	urdf::LinkConstSharedPtr link_ = model.getLink(tip_name);
	urdf::JointConstSharedPtr joint_;
	joint_limits_.pos_min.resize(kdl_chain_.getNrOfJoints());
	joint_limits_.pos_max.resize(kdl_chain_.getNrOfJoints());
	joint_limits_.vel_max.resize(kdl_chain_.getNrOfJoints());
	joint_limits_.acc_max.resize(kdl_chain_.getNrOfJoints());
	init_states_.resize(kdl_chain_.getNrOfJoints());

	joint_curr_states_.resize(kdl_chain_.getNrOfJoints());
	joint_des_states_.resize(kdl_chain_.getNrOfJoints());


	int index = 0;

	joint_state.name.resize(kdl_chain_.getNrOfJoints());
    joint_state.position.resize(kdl_chain_.getNrOfJoints());

    //load parameters from rosparam

    nh_.param<double>("/error_tolerance", tol_, 1e-5);

	for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
	{
		if ( it->getJoint().getType() != KDL::Joint::None )
		{
			joint_names_.push_back(it->getJoint().getName());
			joint_ = model.getJoint(it->getJoint().getName()); 
			double d_dato;
			nh_.param<double>(joint_->name.c_str()+string("/pos_max"),joint_limits_.pos_max(index)  , 360*M_PI/180.0 );
			ROS_DEBUG("Max position limit for joint: %s \t %f", joint_->name.c_str(),joint_limits_.pos_max(index) );
			nh_.param<double>(joint_->name.c_str()+string("/pos_min"),joint_limits_.pos_min(index) ,  -360*M_PI/180.0 );
			ROS_DEBUG("Min position limit for joint: %s \t %f", joint_->name.c_str(),joint_limits_.pos_min(index) );
			nh_.param<double>(joint_->name.c_str()+string("/vel_max"),joint_limits_.vel_max(index),  180*M_PI/180.0 );
			ROS_DEBUG("Velocity limit for joint: %s \t %f", joint_->name.c_str(),joint_limits_.vel_max(index) );
			nh_.param<double>(joint_->name.c_str()+string("/acc_max"), joint_limits_.acc_max(index), 100*M_PI/180.0);
			ROS_DEBUG("Acceleration limit for joint: %s \t %f", joint_->name.c_str(),joint_limits_.acc_max(index) );
			nh_.param<double>(joint_->name.c_str()+string("/init_pos"), init_states_.q(index), 100*M_PI/180.0);
			ROS_DEBUG("Initial position for joint: %s \t %f", joint_->name.c_str(),init_states_.q(index) );
			joint_curr_states_.q(index)= init_states_.q(index);
			joint_curr_states_.qdot(index)= 0.0;
			joint_curr_states_.qdotdot(index)= 0.0;
			joint_des_states_.q(index)= init_states_.q(index);
			joint_des_states_.qdot(index)= 0.0;
			joint_des_states_.qdotdot(index)= 0.0;
			joint_state.name[index] = it->getJoint().getName();

			index++;
		}
	}

	trajs_generators_ = new std::vector<trajectory::TrajectoryGenerator*>(kdl_chain_.getNrOfJoints());
	trajs_in_ = new std::vector<trajectory_msgs::JointTrajectory*>(kdl_chain_.getNrOfJoints());
	trajs_out_ = new std::vector<trajectory_msgs::JointTrajectory*>(kdl_chain_.getNrOfJoints());

	for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
	{   
		trajs_generators_->at(i) = new trajectory::TrajectoryGenerator(joint_limits_.vel_max(i),joint_limits_.acc_max(i),1);
	    trajectory_msgs::JointTrajectory tmp_traj;

	}


	robot_in_target_ = true;
	//initialize kinematic utils
	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
	ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
	ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.pos_min,joint_limits_.pos_max,*fk_pos_solver_,*ik_vel_solver_));

	J_.resize(kdl_chain_.getNrOfJoints());
	fk_pos_solver_->JntToCart(joint_curr_states_.q, x_initial_);
	x_curr_ = x_initial_;

	//initialize ros topics

	sub_move_to_joint_ref_ = nh_.subscribe("/moveToJointRef", 250, &UR10::moveToJointRef, this);
	sub_move_to_cart_ref = nh_.subscribe("/moveToCartRef", 250, &UR10::moveToCartRef, this);

	sub_sphere = nh_.subscribe("/visualization_marker", 250, &UR10::spherePoseCb, this);
	pub_cart_pose_ = nh_.advertise<geometry_msgs::Pose>("/robot_cart_pose", 250);;
	pub_traj_sucess_ = nh_.advertise<std_msgs::Bool>("/movement_completed", 250);
	pub_cart_dist_sphere = nh_.advertise<std_msgs::Float64>("/sphere_dist", 250);
	pub_joint_states_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 250);
	pub_cart_ball_pose_in_tcp = nh_.advertise<geometry_msgs::Pose>("/sphere_pose_in_tcp", 250);

	geometry_msgs::Pose cart_pose;
	tf::poseKDLToMsg(x_curr_, cart_pose);
	pub_cart_pose_.publish(cart_pose);
	//ros::spinOnce();

	return true;

};

void UR10::spinOnce(const ros::Time& time, const ros::Duration& period){

	nh_.param<double>("/error_tolerance", tol_, 1e-5);
	period_ = period;
	// prepare message to update model states
    for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
	{
	    joint_state.position[i] = joint_curr_states_.q(i);
	}
	joint_state.header.stamp = ros::Time::now();
	pub_joint_states_.publish(joint_state);

	//Decide if doing Joint Space or Cartesian Space motion
	if(last_reference_in_joint_space_)
		performJointStepFromJointRef(time, period);
	else
		performJointStepFromCartRef(time, period);
	
	convertJointStatesToCartPose();
	ros::spinOnce();
};

void UR10::convertJointStatesToCartPose()
{
	//stream cartesian position
	fk_pos_solver_->JntToCart(joint_curr_states_.q, x_curr_);
	geometry_msgs::Pose cart_pose;
	tf::poseKDLToMsg(x_curr_, cart_pose);
	pub_cart_pose_.publish(cart_pose);

}

void UR10::moveToJointRef(const hw_mb::JointPoseRef::ConstPtr &msg)
{

	//read Joint position references and prepare for trajectory computation and execution
	if(msg->time_exec < 0.01){
		ROS_INFO ("Execution Time should be grater than 0.01. Ignoring command");
		return;
	}
	last_reference_in_joint_space_ = true;
	joint_des_states_.q(0) = msg->q1;
	joint_des_states_.q(1) = msg->q2;
	joint_des_states_.q(2) = msg->q3;
	joint_des_states_.q(3) = msg->q4;
	joint_des_states_.q(4) = msg->q5;
	joint_des_states_.q(5) = msg->q6;


	//double check joint limits

	for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
	{
		if(joint_des_states_.q(i) > joint_limits_.pos_max(i))
		{
			joint_des_states_.q(i) = joint_limits_.pos_max(i);
			ROS_WARN("Desired Reference do Joint %d is out of limit. Reference will be limited automatically to %f",i,joint_des_states_.q(i));
		}
		if(joint_des_states_.q(i) < joint_limits_.pos_min(i))
		{
			joint_des_states_.q(i) = joint_limits_.pos_min(i);
			ROS_WARN("Desired Reference do Joint %d is out of limit. Reference will be limited automatically to %f",i,joint_des_states_.q(i));
		}
	}

	joint_des_states_.qdot(0) = 0.0;
	joint_des_states_.qdot(1) = 0.0;
	joint_des_states_.qdot(2) = 0.0;
	joint_des_states_.qdot(3) = 0.0;
	joint_des_states_.qdot(4) = 0.0;
	joint_des_states_.qdot(5) = 0.0;

	joint_des_states_.qdotdot(0) = 0.0;
	joint_des_states_.qdotdot(1) = 0.0;
	joint_des_states_.qdotdot(2) = 0.0;
	joint_des_states_.qdotdot(3) = 0.0;
	joint_des_states_.qdotdot(4) = 0.0;
	joint_des_states_.qdotdot(5) = 0.0;

	time_desired_to_reach_target_ = msg->time_exec;


	//compute Joint trajectory
	computeJointTrajFromJointRef();

	

};

trajectory_msgs::JointTrajectory UR10::getTrajectory()
{
	trajectory_is_available_ = false;
	return traj_;
};


void UR10::spherePoseCb(const visualization_msgs::Marker::ConstPtr &msg )
{
	//stream distance to the sphere
	KDL::Vector p_marker(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

	KDL::Twist x_err;
	KDL::Frame x_sphere(p_marker); // pose of the ball in world frame
	//x_curr is  the pose of the TCP in world frame
	x_err.vel = x_sphere.p - x_curr_.p;
	//geometry_msgs::Vector3 sphere_dist;
	std_msgs::Float64 dist;
	dist.data = sqrt(x_err.vel(0) *x_err.vel(0) +x_err.vel(1) *x_err.vel(1) +x_err.vel(2) *x_err.vel(2) ) ;
	//sphere_dist.x = x_err.vel(0) ;
	//sphere_dist.y = x_err.vel(1);
	//sphere_dist.z = x_err.vel(2);

	KDL::Frame sphere_pose_in_tcp(x_curr_.Inverse()*x_sphere); //sphere pose with respect to  robot Tcp
	pub_cart_dist_sphere.publish(dist);
	geometry_msgs::Pose sphere_pose_in_tcp_msg;
	tf::poseKDLToMsg(sphere_pose_in_tcp, sphere_pose_in_tcp_msg);
	pub_cart_ball_pose_in_tcp.publish(sphere_pose_in_tcp_msg);

};

void UR10::moveToCartRef(const geometry_msgs::Point::ConstPtr &msg )
{
	//read Cartesian position references and prepare for trajectory computation and execution
	trajectory_is_available_ = false;
	fk_pos_solver_->JntToCart(joint_curr_states_.q, x_curr_);
	x_des_ = KDL::Frame(x_curr_.M, KDL::Vector(msg->x, msg->y, msg->z) );

	last_reference_in_joint_space_ = false;
		trajectory_is_available_ = true;
};

void UR10::performJointStepFromJointRef(const ros::Time& time, const ros::Duration& period)
{

	//Execute Joins space trajectory	
	if(trajectory_is_available_)
	{
		double joint_space_error =0.0;
		for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
			joint_space_error += abs(joint_curr_states_.q(i)-joint_des_states_.q(i));

		std_msgs::Bool b;

		if(joint_space_error <= tol_)
		{
			trajectory_is_available_ = false;
			b.data = true;
			pub_traj_sucess_.publish(b);
			return;
		}
		b.data = false;
		pub_traj_sucess_.publish(b);

		//update states an trajectory
		for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
		{
			if (trajs_out_->at(i)->points.size()>0)
			{
				joint_curr_states_.q(i) = trajs_out_->at(i)->points[0].positions[0];
				trajs_out_->at(i)->points.erase(trajs_out_->at(i)->points.begin());
			}
		}

	}
};
void UR10::performJointStepFromCartRef(const ros::Time& time, const ros::Duration& period)
{


	// perform resolved motion step
	fk_pos_solver_->JntToCart(joint_curr_states_.q, x_curr_);
	jnt_to_jac_solver_->JntToJac(joint_curr_states_.q, J_);

	//here to avoid singularities
	pseudo_inverse_rank_update(J_.data, J_pinv_, true);

// end-effector position error
	x_err_.vel = x_des_.p - x_curr_.p;


	x_curr_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);   //.M is referred to the rotation of x_. The function GetQuaternion obtains quaternion from the rotation matrix
	x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

	skew_symmetric(quat_des_.v, skew_); // Gets skew-matrix from the quaternion of desired frame
	KDL::Vector v_temp_;
	for (int i = 0; i < skew_.rows(); i++)
	{
		v_temp_(i) = 0.0;                               //Initialization of the i_element of vector v_temp
		for (int k = 0; k < skew_.cols(); k++)      
	   	 v_temp_(i) += skew_(i,k)*(quat_curr_.v(k)); //Substitution of the the i_element of vector v_temp with the product between skew_matrix and quaternion
	}

	// end-effector orientation error
	x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

	double alpha1;
	nh_.param<double>("/alpha_cart_to_joint_projection", alpha1, 1.0); // This param is to define the length of the step over the gradient

	//Project Cartesian errors to joint speed. This is a P controller. At least D controller should be implemented to improve performance 
	for (int i = 0; i < J_pinv_.rows(); i++)
	{
		joint_curr_states_.qdot(i) = 0.0;
		for (int k = 0; k < J_pinv_.cols(); k++)
			joint_curr_states_.qdot(i) += alpha1 * J_pinv_(i,k) * x_err_(k);

	}

	//double check limits
	for (int i =0;  i < joint_names_.size(); i++)
	{
		if (joint_curr_states_.qdot(i) < -joint_limits_.vel_max(i))
			joint_curr_states_.qdot(i) = -joint_limits_.vel_max(i);
		if (joint_curr_states_.qdot(i) > joint_limits_.vel_max(i))
			joint_curr_states_.qdot(i) = joint_limits_.vel_max(i);
	}

	for (int i = 0; i < joint_names_.size(); i++)
		joint_curr_states_.q(i) += 0.1*joint_curr_states_.qdot(i);
	

	for (int i =0;  i < joint_names_.size(); i++)
	{
		if (joint_curr_states_.q(i) < joint_limits_.pos_min(i))
			joint_curr_states_.q(i) = joint_limits_.pos_min(i);
		if (joint_curr_states_.q(i) > joint_limits_.pos_max(i))
			joint_curr_states_.q(i) = joint_limits_.pos_max(i);
	}

	double sqr_err = sqrt(x_err_.vel(0)*x_err_.vel(0) + x_err_.vel(1)*x_err_.vel(1)+ x_err_.vel(2)*x_err_.vel(2));

	std_msgs::Bool b;
	if(sqr_err < tol_)
	{
		b.data = true;
		trajectory_is_available_ = false;
	}
	else
		b.data = false;
	pub_traj_sucess_.publish(b);


};


void UR10::computeJointTrajFromJointRef()
{
	trajectory_is_available_ = false;
	double joint_space_error =0;
	for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
		joint_space_error += abs(joint_curr_states_.q(i)-joint_des_states_.q(i));

	if (joint_space_error <= tol_)
	{
		ROS_INFO("Joint space error is already lower that the tolerance" );
		return;
	}

	for (int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
	{       
		trajectory_msgs::JointTrajectory traj_in, traj_out;
		traj_in.header.stamp = ros::Time::now();

		traj_in.joint_names.resize(1);
		traj_in.points.resize(2);
		traj_in.points[0].positions.resize(1);
		traj_in.points[1].positions.resize(1);
		traj_in.points[0].time_from_start = ros::Duration(0.0);
		traj_in.points[1].time_from_start = ros::Duration(time_desired_to_reach_target_);
		traj_in.joint_names[0] = joint_names_[i];
		traj_in.points[0].positions[0] = joint_curr_states_.q(i);
		traj_in.points[1].positions[0] = joint_des_states_.q(i);
		trajs_generators_->at(i)->generate(traj_in, traj_out);
		trajs_out_->at(i) = new trajectory_msgs::JointTrajectory(traj_out);
		
	}

	trajectory_is_available_ = true;

};
