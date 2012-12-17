/*
Parker Conroy
ARLab

This program processes tag data from the ardrone.


#Tags in Vision Detectoion
uint32 tags_count
uint32[] tags_type
uint32[] tags_xc
uint32[] tags_yc
uint32[] tags_width
uint32[] tags_height
float32[] tags_orientation
float32[] tags_distance

#time stamp
float32 tm


*/
#include <ros/ros.h>
#include <vector>
#include <std_msgs/Empty.h>
#include <cstdlib>
#include <ardrone_autonomy/Navdata.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//#define uint32 int
#define float32 float
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

	std_msgs::Empty emp_msg;
    ardrone_autonomy::Navdata msg_in;
    
    int x=0;
    int y=1;
    int z=2;
    
    int had_message =0;
    float deg2rad= 0.0174532925;
	int drone_state;
	float vision_angle[2];
	float tag_position[3];
	int cam_height=360;
	int cam_width=640;
	float cam_width_degree=92*deg2rad;
	float cam_height_degree=51*deg2rad; //cam_width_degree*(cam_height/cam_width) //NOT MEASURED
	
	uint32_t tags_count;
	/*
	std::vector<uint32_t> tags_type;
	std::vector<uint32_t> tags_xc;
	std::vector<uint32_t> tags_yc;
	std::vector<uint32_t> tags_width;
	std::vector<uint32_t> tags_height;
	std::vector<float> tags_orientation;
	std::vector<float> tags_distance;
	*/
	
	
	uint32_t tags_type[10];
	uint32_t tags_xc[10];
	uint32_t tags_yc[10];
	uint32_t tags_width[10];
	uint32_t tags_height[10];
	float tags_orientation[10];
	float tags_distance[10];
	
	
	double time_stamp;
/*
void change_vector_size(int size)
{
	 tags_type.resize(size);
	 tags_xc.resize(size);
	tags_yc.resize(size);
	tags_width.resize(size);
	 tags_height.resize(size);
	tags_orientation.resize(size);
	 tags_distance.resize(size);
}
*/
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
		//Take in state of ardrone	
	//	if (tags_count<msg_in.tags_count){
//	change_vector_size(msg_in.tags_count);
	tags_count=msg_in.tags_count;
//	}
	
	time_stamp=msg_in.tm;	
		
	for (uint32_t i=0; i <tags_count; i++)
	{
    tags_distance[i]=msg_in.tags_distance[i];
    tags_xc[i]=msg_in.tags_xc[i];
    tags_yc[i]=msg_in.tags_yc[i];
    tags_width[i]=msg_in.tags_width[i];
    tags_height[i]=msg_in.tags_height[i];
    tags_orientation[i]=msg_in.tags_orientation[i];
    }//for
}	//call back

int main(int argc, char** argv)
{
	//using namespace std;
	ROS_INFO("Starting Tag Tracker");
	ros::init(argc, argv,"ARDrone_tag_track");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
	ros::Subscriber nav_sub;
	ros::Publisher cloud_pub;

	nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
	cloud_pub = node.advertise<PointCloud> ("point_cloud", 1);

	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id = "/mydrone/ardrone_base_frontcam";
	//msg->height = cam_height;
	//msg->width = cam_width;
  
 	while (ros::ok()) {
 		int tag_id =0;
	 	vision_angle[x]=( (((float)(tags_xc[tag_id]-500.0)) /500.0) *cam_width_degree/2.0);
	 	vision_angle[y]=( (((float)(tags_yc[tag_id]-500.0)) /500.0) *cam_height_degree/2.0);
	 	tag_position[x]=tags_distance[tag_id]*sin(vision_angle[x]);
	 	tag_position[y]=tags_distance[tag_id]*sin(vision_angle[y]);
	 	tag_position[z]=tag_position[x]/tan(vision_angle[x]);
	 	
//		  (pcl::PointXYZ(tag_position[x], tag_position[y], tag_position[z]));
	//	 msg->points.x=tag_position[x];
	//	 msg->points.y=tag_position[y];
	//	 msg->points.z=tag_position[z];
	// msg->points.push_back (pcl::PointXYZ(tag_position[x], tag_position[y], tag_position[z]));
	 msg->points.assign(1,pcl::PointXYZ(tag_position[x], tag_position[y], tag_position[z]));
	 //	msg->points.push_back (pcl::PointXYZ(1.0,2.0,3.0));
	 	msg->header.stamp = ros::Time::now ();
		cloud_pub.publish (msg);
		
		ROS_INFO("Tag position~ x: %f y: %f z: %f",tag_position[x],tag_position[y],tag_position[z]);
		ROS_INFO("count: %i, xc: %i, yc: %i, distance: %f",tags_count,tags_xc[tag_id],tags_yc[tag_id],tags_distance[tag_id]); //width: %i, height: %i, orientation: %i
		
		ros::spinOnce();
		loop_rate.sleep();
								
			}//ros::ok
ROS_ERROR("ROS not ok (tag_tracker)");
exit(0);
}//main
