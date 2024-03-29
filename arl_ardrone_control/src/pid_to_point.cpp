/*
 *  Copyright (c) 2012, Parker Conroy
 *	ARLab @ University of Utah
 *  All rights reserved.
 *
 *
 *	The purpose of this software is to take in the tf of a trackee, and output the desired tf of a robot tracking said trackee
 *
 *
 *
 *	This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
#include "ros/ros.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/time.h"
#include <geometry_msgs/Twist.h>

#define yaw 0
#define pitch 1
#define roll 2
#define thrust 3
#define msg_time 4

double new_data[5];
double old_data[5];
double integration[5];
double derivative[5];
double controls[4];
float min_control[4];
float max_control[4];
double pid[4];
float min_pid;
float max_pid;
float K_p[] = {1,1,1,1}; //yaw, roll, pitch, thrust
float K_d[] = {0,0,0,0};
float K_i[] = {0,0,0,0};

float map(float value, float in_min, float in_max, float out_min, float out_max) {
  return (float)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);}


int main(int argc, char **argv)
{
	int rate= 50;
	float inv_rate=1/rate;

	ros::init(argc, argv, "ARdrone_PID_to_Point");
	ros::NodeHandle n;
	ros::Rate loop_rate(rate);
	
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster br;

	tf::StampedTransform desired_pos;
	tf::StampedTransform ardrone;
	tf::StampedTransform trackee;
	tf::StampedTransform desired_in_ardrone_coords;

	ros::Publisher pub_twist;
	geometry_msgs::Twist twist_msg;
	tf::Quaternion ardrone_yawed;
	
	memset(controls, 0, sizeof(double)*4);
	memset(old_data, 0, sizeof(double)*5);
	memset(new_data, 0, sizeof(double)*5);
	memset(integration, 0, sizeof(double)*5);
	memset(derivative, 0, sizeof(double)*5);
	memset(pid, 0, sizeof(double)*4);
	
	//PID params
	min_control[yaw] =-1.0;
	min_control[roll] =-1.0;
	min_control[pitch]=-1.0;
	min_control[thrust]=-1.0;

	max_control[yaw]=1.0;
	max_control[roll]=1.0;
	max_control[pitch]=1.0;
	max_control[thrust]=1.0;

	min_pid =-5.0;
	max_pid =5.0;

	while (ros::ok())
	{
		  try {
		//Get desired position transform
		tf_listener.waitForTransform("/optitrak", "/desired_position", ros::Time(0), ros::Duration(inv_rate));
		tf_listener.lookupTransform("/optitrak", "/desired_position",  ros::Time(0), desired_pos); 
		// Get the quad rotor transform
		tf_listener.waitForTransform("/optitrak", "/ardrone", ros::Time(0), ros::Duration(inv_rate));
		tf_listener.lookupTransform("/optitrak", "/ardrone",  ros::Time(0), ardrone);

		} catch (...) {
		  	ROS_ERROR("Failed on initial transform: Check VRPN server");}
	
		  double y1, p1, r1;
		  btMatrix3x3(ardrone.getRotation()).getRPY(r1, p1, y1);
		  
		  ardrone_yawed.setRPY(0.0,0.0,y1);
		  
		  /* //Dep code 
		  btQuaternion ardrone_yawed(y1, 0.0, 0.0);
		  */
		  
		  ardrone.setRotation(ardrone_yawed);

		  //set up twist publisher
		  pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
		
		  // Register the ardrone without roll and pitch with the transform system
		  br.sendTransform( tf::StampedTransform(ardrone, ros::Time::now(), "/optitrak", "ardrone_wo_rp") );
		  
		  try {
		  // Get the vector between quad without roll and pitch and the desired point
		  tf_listener.waitForTransform("/ardrone_wo_rp", "/desired", ros::Time(0), ros::Duration(inv_rate));
		  tf_listener.lookupTransform("/ardrone_wo_rp", "/desired", ros::Time(0), desired_in_ardrone_coords);
		  }
		  catch (...) {
		  	ROS_ERROR("Failed on w/o roll and pitch transform");}
		  	
		  // Extract the yaw, x, y, z components
		  btMatrix3x3(desired_in_ardrone_coords.getRotation()).getRPY(r1, p1, y1);
		  new_data[yaw]=y1;
		  new_data[pitch] = desired_in_ardrone_coords.getOrigin().getX();
		  new_data[roll] = desired_in_ardrone_coords.getOrigin().getY();
		  new_data[thrust] = desired_in_ardrone_coords.getOrigin().getZ();
 		  new_data[msg_time] = (double)ros::Time::now().toSec();
		  ROS_DEBUG("Error: [x: %f y:  %f z: %f]", new_data[pitch], new_data[roll], new_data[thrust]);

		  // Integrate/Derivate the data
		  double deltaT = (new_data[msg_time] - old_data[msg_time]);
		  integration[yaw] += new_data[yaw] * deltaT;
		  integration[pitch] += new_data[pitch] * deltaT;
		  integration[roll] += new_data[roll] * deltaT;
		  integration[thrust] += new_data[thrust] * deltaT;

		  ROS_DEBUG("Integration: [deltaT: %f x: %f y: %f z: %f]", deltaT, integration[pitch], integration[roll], integration[thrust]);

		  derivative[yaw] = (new_data[yaw] - old_data[yaw])/deltaT;
		  derivative[pitch] = (new_data[pitch] - old_data[pitch])/deltaT;
		  derivative[roll] = (new_data[roll] - old_data[roll])/deltaT;
		  derivative[thrust] = (new_data[thrust] - old_data[thrust])/deltaT;

		  ROS_DEBUG("Derivative: [deltaT: %f x: %f y: %f z: %f]", deltaT, derivative[pitch], derivative[roll], derivative[thrust]);

		  // Calculate the PID values
		  pid[yaw] = K_p[yaw] * new_data[yaw] + K_d[yaw] * derivative[yaw] + K_i[yaw] * integration[yaw];
		  pid[pitch] = K_p[pitch] * new_data[pitch] + K_d[pitch] * derivative[pitch] + K_i[pitch] * integration[pitch];
		  pid[roll] = K_p[roll] * new_data[roll] + K_d[roll] * derivative[roll] + K_i[roll] * integration[roll];
		  pid[thrust] = K_p[thrust] * new_data[thrust] + K_d[thrust] * derivative[thrust] + K_i[thrust] * integration[thrust];

		  ROS_DEBUG("PID: [x: %f y:  %f z: %f]", pid[pitch], pid[roll], pid[thrust]);

		  memcpy(old_data, new_data, sizeof(double)*5);
			
		  pid[yaw]=0.0; //YAW IS DISABLED!
		  controls[yaw] =    map(pid[yaw], 	 min_pid, max_pid, min_control[yaw], max_control[yaw]);
		  controls[pitch] =   map(pid[pitch], min_pid, max_pid, min_control[roll], max_control[roll]);
		  controls[roll] =  map(pid[roll], min_pid, max_pid, min_control[pitch], max_control[pitch]);
		  controls[thrust] = map(pid[thrust], min_pid, max_pid, min_control[thrust], max_control[thrust]);
		  ROS_DEBUG("Controls: [yaw: %f roll: %f pitch: %f thrust: %f]", controls[yaw], controls[roll], controls[pitch], controls[thrust]);

	      //change the ref frame to inverted x-y-z coords. by modifying the directional control
		  twist_msg.linear.x=-controls[roll]; 
		  twist_msg.linear.y=-controls[pitch];	
		  twist_msg.linear.z=-controls[thrust];
		  twist_msg.angular.z=controls[yaw];
          pub_twist.publish(twist_msg);
		  
			ros::spinOnce();
			loop_rate.sleep();
	
}//while ros ok      
	ROS_ERROR("ROS::ok failed- Node Closing");
		  
}//main 
