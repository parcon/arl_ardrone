/*
Parker Conroy
ARLab

Visual Servoing of ARdrone from Point feature.


*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <ardrone_autonomy/Navdata.h>

//Coordinate definitions for vectors
const int x =0;
const int y =1;
const int z =2;
const int t =3;

const int yaw =0;
const int roll =1;
const int pitch =2;
const int thrust =3;

//parameters
const int desired_z= 26; //may want to take this from the navdata, since we are gonna need the angles


const float error_scale=.005;
const float error_scale_z=.001;
int had_message =0;
double controls[4];
double old_data[4];
double msg_time;
double new_data[4];
float integration[4];
float derivative[4];
float pid[4];
float trackee_position[4];
float min_control[4];
float max_control[4];
float min_pid; 
float max_pid;
float K_p[4];
float K_d[4];
float K_i[4];
float quad_roll;
float quad_pitch;
float quad_height;
float temp_quad_roll;
float temp_quad_pitch;
float temp_quad_height;
float temp_trackee_position[4];
float deg2rad= 180/3.14159265359;
geometry_msgs::Twist twist_msg;
geometry_msgs::Point point_msg;

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
		//Take in state of ardrone	
	temp_quad_roll=msg_in.rotY;	
	temp_quad_pitch=msg_in.rotY;
	temp_quad_height=msg_in.altd;	
}

	void point_callback(const geometry_msgs::Point& point_in)
{
	had_message=1;
	//Take in point
	temp_trackee_position[x]=point_in.y;
	temp_trackee_position[y]=point_in.x;
	temp_trackee_position[z]=point_in.z;
}

float map(float value, float in_min, float in_max, float out_min, float out_max) {
  return (float)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int main(int argc, char** argv)
{
	//using namespace std;
	ros::init(argc, argv,"ARDrone_hover_over_Haar");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
	
	memset(controls, 0, sizeof(int64_t)*4);
	memset(old_data, 0, sizeof(double)*4);
	memset(new_data, 0, sizeof(double)*4);
	memset(integration, 0, sizeof(float)*4);
	memset(derivative, 0, sizeof(float)*4);
	memset(pid, 0, sizeof(float)*4);

	msg_time =0.0;

	//PID params
	min_control[yaw] =-1.0;
	min_control[roll] =-1.0;
	min_control[pitch]=-1.0;
	min_control[thrust]=-1.0;

	max_control[yaw]=1.0;
	max_control[roll]=1.0;
	max_control[pitch]=1.0;
	max_control[thrust]=1.0;

	min_pid =-75.0;
	max_pid =75.0;

	K_p[x] = K_p[y] = K_p[z] = 1;
	K_d[x] = K_d[y] = K_d[z] = 0;
	K_i[x] = K_i[y] = K_i[z] = 0;

	ros::Publisher pub;
	ros::Subscriber point_sub;
	ros::Subscriber nav_sub;

	//cam_sub = n.subscribe("/ardrone/camera_info", 1, cam_callback);

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //que of 1 
	nav_sub = n.subscribe("/ardrone/navdata", 1, nav_callback);
	point_sub = n.subscribe("ardrone_tracker/found_point", 10, point_callback);

  

 	while (ros::ok()) 
			{

			if (had_message)
				{			
				//read variable incase it changes during error calculation
				trackee_position[x]=temp_trackee_position[x]-50;
				trackee_position[y]=50-temp_trackee_position[y];
				trackee_position[z]=temp_trackee_position[z];
				quad_roll=temp_quad_roll*deg2rad;	
				quad_pitch=temp_quad_pitch*deg2rad;
				quad_height=temp_quad_height/1000;
				ROS_INFO("Target: [x: %f y:  %f z: %f]", trackee_position[x], trackee_position[y], trackee_position[z]);

/*
				  // Extract the x, y, & z components, now dealing with orientatiton 
				  new_data[x] = (trackee_position[x] - quad_height*sin(quad_pitch));
				  new_data[y] = (trackee_position[y] - quad_height*sin(quad_roll));
				  new_data[z] = (trackee_position[z]-desired_z);
				  new_data[t] = (double)ros::Time::now().toSec();
*/

				  // Extract the x, y, & z components, now dealing with orientatiton 
				  new_data[x] = (trackee_position[x]);
				  new_data[y] = (trackee_position[y]);
				  new_data[z] = (trackee_position[z]-desired_z);
				  new_data[t] = (double)ros::Time::now().toSec();


				 ROS_INFO("Error: [x: %f y:  %f z: %f subtract: %f]", new_data[x], new_data[y], new_data[z], quad_height*sin(quad_pitch));


				  // Integrate the data
				  double deltaT = (new_data[t] - old_data[t]);
				  integration[x] += new_data[x] * deltaT;
				  integration[y] += new_data[y] * deltaT;
				  integration[z] += new_data[z] * deltaT;

				  ROS_DEBUG("Integration: [deltaT: %f x: %f y: %f z: %f]", deltaT, integration[x], integration[y], integration[z]);

				  derivative[x] = (new_data[x] - old_data[x])/deltaT;
				  derivative[y] = (new_data[y] - old_data[y])/deltaT;
				  derivative[z] = (new_data[z] - old_data[z])/deltaT;

				  ROS_DEBUG("Derivative: [deltaT: %f x: %f y: %f z: %f]", deltaT, derivative[x], derivative[y], derivative[z]);

				//set terms for next cycle, error and time
				memcpy(old_data, new_data, sizeof(double)*4);
				msg_time = (double)ros::Time::now().toSec();
				
				//set controls w/ PID
				pid[x]= K_p[x]*new_data[x]+K_d[x]*derivative[x]+K_i[x]*integration[x]; 
				pid[y]= K_p[y]*new_data[y]+K_d[y]*derivative[y]+K_i[y]*integration[y];
				pid[z]= K_p[z]*new_data[z]+K_d[z]*derivative[z]+K_i[z]*integration[z];
				ROS_INFO("PID: x: %f y: %f z: %f]", pid[x], pid[y], pid[z]);

				controls[yaw] = map(0.0, min_pid, max_pid, min_control[yaw], max_control[yaw]); //no yaw controls are implimented yet
      			controls[roll] = map(pid[x], min_pid, max_pid, min_control[roll], max_control[roll]);
      			controls[pitch] = map(pid[y], min_pid, max_pid, min_control[pitch], max_control[pitch]);
			    controls[thrust] = map(0.0, min_pid, max_pid, min_control[thrust], max_control[thrust]); //not height controls are implimented
	ROS_INFO("Controls: x: %f y: %f z: %f z_rot: %f]", controls[roll], controls[pitch], controls[thrust],controls[yaw]);
				//Create command message
				twist_msg.linear.x=controls[roll]; 
				twist_msg.linear.y=controls[pitch];	
				twist_msg.linear.z=controls[thrust];
				twist_msg.angular.z=controls[yaw];
	
		/*
				//Create command message
				twist_msg.linear.x=0.0; 
				twist_msg.linear.y=0.0;
				twist_msg.linear.z=0.0;
				twist_msg.angular.z=0.0;
				
*/
				pub.publish(twist_msg);
				}//if had message
			ros::spinOnce();
			loop_rate.sleep();
														
			}//ros::ok

}//main
