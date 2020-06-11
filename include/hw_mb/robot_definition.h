
#ifndef ROBOT_DEFINITION_H
#define ROBOT_DEFINITION_H

#include <urdf/model.h>
#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <kdl_conversions/kdl_msg.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_broadcaster.h>
#include <joint_trajectory_generator/trajectory_generation.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <hw_mb/JointPoseRef.h>



using namespace std;

class UR10 
{
public:
  UR10(){
    last_reference_in_joint_space_ = true;
    time_desired_to_reach_target_ = 0.0;
    robot_in_target_ = true;
    trajectory_is_available_ = false;
  }
  ~UR10(){}
  bool init(ros::NodeHandle &n);
  void spinOnce(const ros::Time& time, const ros::Duration& period);
  int getNumJoints(){return kdl_chain_.getNrOfJoints();};
  std::vector<string> getJointNames(){return joint_names_;};
  KDL::JntArrayAcc getInitStates(){return init_states_;};
  void moveToJointRef(const hw_mb::JointPoseRef::ConstPtr &msg);
  bool isTrajectoryAvailable(){return trajectory_is_available_;};
  trajectory_msgs::JointTrajectory getTrajectory();
  void moveToCartRef(const geometry_msgs::Point::ConstPtr &msg );

private:
  std::vector<string> joint_names_;
  void convertJointStatesToCartPose();
  void spherePoseCb(const visualization_msgs::Marker::ConstPtr &msg);
  void performJointStepFromJointRef(const ros::Time& time, const ros::Duration& period);
  void computeJointTrajFromJointRef();
  void performJointStepFromCartRef(const ros::Time& time, const ros::Duration& period);
  bool trajectory_is_available_;
  ros::Duration period_;
  bool last_reference_in_joint_space_;
  double time_desired_to_reach_target_;
  bool robot_in_target_;
  double tol_;

  // trajectory utils
  std::vector<trajectory_msgs::JointTrajectory*> *trajs_in_;
  std::vector<trajectory_msgs::JointTrajectory*> *trajs_out_;
  std::vector<trajectory::TrajectoryGenerator*> *trajs_generators_;

  //Ros utils
  sensor_msgs::JointState joint_state;

  ros::Subscriber sub_move_to_joint_ref_, sub_sphere, sub_move_to_cart_ref;
  ros::Publisher pub_cart_pose_, pub_cart_dist_sphere;
  ros::Publisher pub_traj_sucess_, pub_joint_states_, pub_cart_ball_pose_in_tcp;
  

protected:
    ros::NodeHandle nh_;
    KDL::Chain kdl_chain_;
    KDL::JntArrayAcc init_states_, joint_curr_states_, joint_des_states_;
    int n_joints;
    struct limits_
    {
      KDL::JntArray pos_min;
      KDL::JntArray pos_max;
      KDL::JntArray vel_max;
      KDL::JntArray acc_max;
    } joint_limits_;

    
    trajectory_msgs::JointTrajectory traj_;
    int index_traj_;


    /* Here all things for  ik*/;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;   // Class to compute the jacobian of a general KDL::Chain, it is used by other solvers. It should not be used outside of KDL.
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;  // Implementation of a recursive forward position kinematics algorithm to calculate the position transformation from joint space to Cartesian space of a general kinematic chain (KDL::Chain)
    boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;   // Implementation of a inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the velocity transformation from Cartesian to joint space of a general KDL::Chain. It uses a svd-calculation based on householders rotations
    boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;    // Implementation of a general inverse position kinematics algorithm based on Newton-Raphson iterations to calculate the position transformation from Cartesian to joint space of a general KDL::Chain. Takes joint limits into account.
    KDL::Frame x_curr_;    //current pose
    KDL::Frame x_initial_;   //current pose
    KDL::Frame x_des_;  //desired pose
    KDL::Twist x_err_;  //error position
    KDL::Jacobian J_; //Jacobian
    Eigen::MatrixXd J_pinv_;  //Pseudoinverse jacobian of dynamic dimension (double)
    Eigen::Matrix<double,3,3> skew_;  //skew-matrix (3x3) of double

    struct quaternion_
    {
      KDL::Vector v;
      double a;
    } quat_curr_, quat_des_;


        
};


#endif