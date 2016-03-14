#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include<vn_100/ins_data.h>

  
  ros::Publisher marker_pub;


void imuCallBack(const vn_100::ins_data::ConstPtr& msg)
{


		
uint32_t shape = visualization_msgs::Marker::CUBE;



    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "pub_rviz";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = msg->quat_data[0];
    marker.pose.orientation.y = msg->quat_data[1];
    marker.pose.orientation.z = msg->quat_data[2];
    marker.pose.orientation.w = msg->quat_data[3];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 3.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

		marker_pub.publish(marker);

		



}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "pub_rviz");

	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_imu", 1);

  
	ros::Subscriber imu_sub = n.subscribe("/vn_100/ins_data", 1, imuCallBack);

	ros::spin();

	return 0;
}
