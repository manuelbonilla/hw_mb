
/* Author: Manuel Bonilla

This code implements a simulator for a robot UR10. However it is parameterized to be used in all mUR models

*/

#include "hw_mb/robot_definition.h"

float curr_time;
float desired_time_for_traj;
hw_mb::JointPoseRef des_refs;
std::vector<float> position;

void cbJointStates(const sensor_msgs::JointState::ConstPtr &msg)
{

    for (int i = 0; i < 6; ++i)
    {
        position[i] = msg->position[i];
    }
}


void cbMovementCompleted(const std_msgs::Bool::ConstPtr &msg)
{
    //compare time
    float thresh_time =0.2;
    if(curr_time <  desired_time_for_traj - thresh_time) 
    {
        ROS_INFO("Time running: %f", curr_time) ;
        return;
    }



    if(msg->data )
        {
            //check for time and positions

            //check if successful in the desired time;
            
            if((curr_time > (desired_time_for_traj - thresh_time)) && (curr_time < (desired_time_for_traj + thresh_time)))
            {
                float err = 0.0;
                for (int i = 0; i < 6; ++i)
                {
                    err+= position[i]-1.0;
                }

                if(err < 0.4)
                    ROS_INFO("Test successfull :)");
                return;
            }
            
        }

          ROS_INFO("Test unsuccessfull :( Time was: %f", curr_time) ;
// decide if test is ok or not;

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "test");
    ros::NodeHandle n;

    int rate = 10;
    ros::Rate loop_rate(rate);
    
    ros::Subscriber sub_robot_on_target;
    ros::Publisher pub_des_joint_states;

    desired_time_for_traj = 5.0;

    //position = std::vector<float>(6,0.0);
    position.resize(6);

    des_refs.q1 = 1.0;
    des_refs.q2 = 1.0;
    des_refs.q3 = 1.0;
    des_refs.q4 = 1.0;
    des_refs.q5 = 1.0;
    des_refs.q6 = 1.0;
    des_refs.time_exec = desired_time_for_traj;
    
    pub_des_joint_states = n.advertise<hw_mb::JointPoseRef>("/moveToJointRef", 250);
    sub_robot_on_target = n.subscribe("/movement_completed", 250, cbMovementCompleted);
    
    ros::spinOnce();
    //take ros time
    ros::Duration(1.0).sleep();

    pub_des_joint_states.publish(des_refs);
    curr_time =0.0;

    while (ros::ok()) 
    {

        ros::spinOnce();
        curr_time+=1.0/rate;
        loop_rate.sleep();
    }

    return 0;
}