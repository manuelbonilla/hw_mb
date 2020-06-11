
/* Author: Manuel Bonilla

This code implements a simulator for a robot UR10. However it is parameterized to be used in all mUR models

*/

#include "hw_mb/robot_definition.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "homework");
    ros::NodeHandle n;

    int rate = 10;
    ros::Rate loop_rate(rate);
    ros::Duration ct (1/rate);

    UR10 Robot; // initialize class
    if(!Robot.init(n)) return 0; //double check errors


    while (ros::ok()) {
        
        Robot.spinOnce(ros::Time::now(),ct); 

        loop_rate.sleep();
    }

    return 0;
}