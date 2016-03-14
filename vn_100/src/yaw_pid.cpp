#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <vn_100/ins_data.h>
#include <dynamic_reconfigure/server.h>
#include <vn_100/pidConfig.h>
#include <vn_100/ins_data.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <string>

using namespace std;

float kp_yaw,ki_yaw,kd_yaw,setpoint_yaw,error,previous_error,total_error,integral,derivative;
float kp_depth,ki_depth,kd_depth,setpoint_depth,error_depth,previous_error_1,total_error_1,integral_1,derivative_1,dt_1=10;
float ITerm,last_yawvalue,ckpoint;
long int present_time = 0.00,past_time =0.00;
float dt;
float last_ckpoint;

bool i;
int x;

 	 std_msgs::UInt16 value;
	 std_msgs::UInt16 value2;
	 std_msgs::UInt16 value1;
	 std_msgs::UInt16  add_value;
	 std_msgs::UInt16  add_value_1;
	 std_msgs::Float64 ui_value;
	 std_msgs::Float64 yaw_value;
	 ros::Time str1;
	 ros::Time str2;
		 	 	
void callback_values(vn_100::pidConfig &config,uint32_t level)
{
	if(config.bool_param == true)
	{
		 kp_yaw = config.kp_yaw ;
	     ki_yaw = config.ki_yaw ;
		 kd_yaw = config.kd_yaw;
		 kp_depth = config.kp_depth ;
		 ki_depth = config.ki_depth ;
		 kd_depth = config.kd_depth;
		 setpoint_depth = config.setpoint_depth;
		 setpoint_yaw = config.setpoint_yaw;
	}
		
}


void callback(const vn_100::ins_data::ConstPtr& yaw_value)
{
	
	ckpoint = yaw_value->YPR.x;
	present_time = (yaw_value->header.stamp.sec);			
}

int main(int argc , char **argv)
{
	ros::init(argc,argv,"yaw_pid");
	ros::NodeHandle n;
	ros::Publisher leftpub=n.advertise<std_msgs::UInt16>("sideleftspeed",1);
	ros::Publisher rightpub=n.advertise<std_msgs::UInt16>("sideripghtspeed",1);
	ros::Publisher frontpub=n.advertise<std_msgs::UInt16>("frontpitchspeed",1);
	ros::Publisher backpub=n.advertise<std_msgs::UInt16>("backpitchspeed",1);
	ros::Subscriber depth_data = n.subscribe<vn_100::ins_data>("/vn_100/ins_data", 1 ,callback);
	dynamic_reconfigure::Server<vn_100::pidConfig> server;
	dynamic_reconfigure::Server<vn_100::pidConfig>::CallbackType f;
	
	f = boost::bind(&callback_values, _1,_2);
	server.setCallback(f);
	
	
	  
	  //	ros::ServiceClient client = n.serviceClient<vn_100::depth_srv>("depth_srv");
	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		if(past_time == 0.00)
			past_time = present_time;
		
			dt = (present_time - past_time);
		
			error = abs(setpoint_yaw - ckpoint);
			
			cout << "present_time :"<<present_time << endl;
			cout<<"dt : "<<dt <<endl;
			cout<<"error : "<<error << endl;
				integral = integral + error * dt;
				
				derivative = (error - previous_error)/dt;
				total_error = kp_yaw*error + ki_yaw*integral + kd_yaw*derivative ;
		
			    cout<<"integral  : "<<integral<<endl;
				cout<<"proportional :"<<kp_yaw*error<<endl;
						
				add_value.data  = total_error/100;
				cout << "add_value :" << total_error/100 << endl;
				value.data = 1500;
				
				if(error >0 )
				{
					value1.data = 1500 + add_value.data;
					value2.data = 1500 -add_value.data;
					if(value1.data > 2000 )
					{
						value1.data = 2000;
						value2.data = 1000;
					}
					leftpub.publish(value1);
					rightpub.publish(value2);
				}
				else
				{
					value1.data =  1500- add_value.data;
					value2.data = 1500 +add_value.data;
					if(value2.data > 2000 )
					{
						value2.data = 2000;
						value1.data = 1000;
					}
				    leftpub.publish(value2);
					rightpub.publish(value1);
				}
					past_time = present_time;
				previous_error = error;
				loop_rate.sleep();
			ros::spinOnce();
		
	}

	
	return 0;
}