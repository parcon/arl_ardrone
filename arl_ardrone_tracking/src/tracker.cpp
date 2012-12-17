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
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/time.h"


 // Variables!!!
tf::Transformer transformer;
tf::TransformListener tf_listener;
tf::TransformBroadcaster br;
/*
desiredtf.setOrigin( Trackee_position );
desiredtf.setRotation( tf::Quaternion(0.0, 0.0, 0.0) );
tf::StampedTransform desired_in_quad;
btVector3 diff;
btQuaternion rotation;
btVector3 axis(0.0, 0.0, 0.0);
float angle = 0.0;
double new_data[4];
double pid[4];
int64_t controls[4];
*/
tf::StampedTransform tracker;
tf::StampedTransform trackee;
//tf::StampedTransform desiredtf;
btVector3 Trackee_position(0.0, 0.0, 0.0);
double yaw, pitch, roll;
double new_x,new_y,new_z;
double deg_to_rad= 180/M_PI;
double follow_radius = 0.5; //[m]
double follow_angle = 90.0*deg_to_rad; //[rads]
double follow_height = 1.5; //[m]


int main(int argc, char **argv)
{

	ros::init(argc, argv, "Tracker");
	ros::NodeHandle n;
	ros::Rate r(200); //update @ 200hz
	
	
while (ros::ok())
{
	  try {
//	tf_listener.waitForTransform("/optitrak", "/trackee", ros::Time(0), ros::Duration(.1)); //wait .1 seconds 																									for new tf
	tf_listener.lookupTransform("/optitrak", "/trackee",  ros::Time(0), trackee); // new tf is called trackee
	
	Trackee_position = trackee.getOrigin(); //vector of position (x,y,z)
	btMatrix3x3(trackee.getRotation()).getRPY(roll, pitch, yaw); // vector of orientation (roll, pitch, yaw)
//	Trackee_position.setZ(2.0);
	  
      
      
      } catch (...) {
      	ROS_ERROR("Failed on Trackee TF");
      }
      
      new_x =Trackee_position.getX()+follow_radius*sin(follow_angle);
      new_y =Trackee_position.getY()+follow_radius*cos(follow_angle);
      new_z =follow_height;
      Trackee_position.setX(new_x);
      Trackee_position.setX(new_y);
      Trackee_position.setX(new_z);
      yaw= atan((new_y-Trackee_position.getY())/(Trackee_position.getX()-new_y));
      btQuaternion des_ori(yaw, 0.0, 0.0);
	//send position and orientation to tracker tf
	tracker.setRotation(des_ori);
	tracker.setOrigin(Trackee_position);
      //publish tracker tf in optitrack world
     br.sendTransform( tf::StampedTransform(tracker, ros::Time::now(), "/optitrak", "Tracker_des_pos") ); 
      
}//while ros ok      
      
}//main 
