#include <iostream>
#include <ros/ros.h>
#include "vectornav.h"
#include<std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
//#include <vn_100/send_arduino.h>
#include <vn_100/ins_data.h>
class SubAndPub
{
	public:
		SubAndPub();
		void callback(const vn_100::ins_data::ConstPtr& sensors);


                ros::NodeHandle n_;
                ros::Subscriber sub_;
                ros::Publisher pub_;

};

SubAndPub::SubAndPub()
{
	  pub_ = n_.advertise<std_msgs::Float64>("/ypr_data",1);

	 sub_ = n_.subscribe<vn_100::ins_data>("/vn_100/ins_data", 1 , &SubAndPub::callback , this);
         //                 pub_ = n_.advertise<geometry_msgs::Vector3>("/ypr_data",1);

}

void SubAndPub::callback(const vn_100::ins_data::ConstPtr& sensors)
{
	std_msgs::Float64 key;
//	send_value::sensor_data msg_data;
//	key = r_data;
	//  key.x = sensors->YPR.x;
	  key.data = sensors->YPR.y;
	//  key.z = sensors->YPR.z;
           pub_.publish(key);
}

int main(int argc ,  char **argv)
{
	ros::init(argc,argv,"send_to_arduino");
	SubAndPub sub_pub;
	ros::spin();
	return 0;
}
