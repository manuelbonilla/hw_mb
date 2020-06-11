
/* Author: Manuel Bonilla

 this node publish a moving ball. In this case the balls moves in world frame Z axis 

*/
#include <ros/ros.h>


#include <rviz_visual_tools/rviz_visual_tools.h>


#include <visualization_msgs/Marker.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "ball_sim");
  ros::NodeHandle n;

  int rate = 10;
  ros::Rate loop_rate(rate);
  ros::Duration ct (1/rate);

  // create topic for the marker
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  
  marker.ns = "basic_shapes";
  marker.id = 0;


  marker.type = shape;


  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration();

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  double freq = .2;
  double time = 0.0;

  double x_amplitude = 0.0;
  double y_amplitude = 0.0;
  double z_amplitude = 0.0;

  while (ros::ok()) {
    //Inside the loop to be dynamic
    n.param<double>("ball_motion_amplitude/x",x_amplitude,  0.0);
    n.param<double>("ball_motion_amplitude/y",y_amplitude,  0.0);
    n.param<double>("ball_motion_amplitude/z",z_amplitude ,  0.0);

    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = x_amplitude*sin(2.0*M_PI*freq*time);
    marker.pose.position.y = y_amplitude*cos(2.0*M_PI*freq*time);
    marker.pose.position.z = z_amplitude*sin(2.0*M_PI*freq*time);
    time += 1.0/rate;
    marker_pub.publish(marker);
    ros::spinOnce(); 
    loop_rate.sleep();
  }
}
