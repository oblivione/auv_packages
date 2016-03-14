#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <vn_100/ins_data.h>
#include <dynamic_reconfigure/server.h>
#include <vn_100/pidConfig.h>
#include <vn_100/depth_srv.h>

#include <iostream>
#include <cstdlib>
using namespace std;

float kp_yaw,ki_yaw,kd_yaw,setpoint_yaw,error,previous_error,total_error,integral,derivative,dt=10;
float kp_depth,ki_depth,kd_depth,setpoint_depth,error_depth,previous_error_1,total_error_1,integral_1,derivative_1,dt_1=10;
float ITerm,last_yawvalue,ckpoint;
bool i;
int x;

 std_msgs::UInt16 value;
std_msgs::UInt16 value2;
	 std_msgs::UInt16 value1;
	 std_msgs::UInt16  add_value;
	 std_msgs::UInt16  add_value_1;
	 std_msgs::Float64 ui_value;
	 std_msgs::Float64 yaw_value;
	 

	 	
void callback_values(vn_100::pidConfig &config,uint32_t level)
{
	kp_yaw = config.kp_yaw ;
	
	
		ki_yaw = config.ki_yaw ;
		 kd_yaw=config.kd_yaw;
		kp_depth = config.kp_depth ;
		 ki_depth = config.ki_depth ;
		 kd_depth = config.kd_depth;
		 setpoint_depth = config.setpoint_depth;
		 setpoint_yaw = config.setpoint_yaw;
		 	i = config.bool_param;
		
}

/*
class Depth
yaw_pub_4
	public:
	
	Depth();
	//void callback(const std_msgs::Float64::ConstPtr& sensors);
	//void callback_1(const std_msgs::Float64::ConstPtr& sensors_1);
	//void callback(const vn_100::ins_data::ConstPtr& yaw);
	         
	            ros::NodeHandle n_;
                ros::Subscriber sub_;
                ros::Subscriber sub_1;
                ros::Subscriber sub_2;
                ros::Publisher yaw_pub_1;
                ros::Publisher yaw_pub_2;
                ros::Publisher yaw_pub_3;
                ros::Publisher yaw_pub_4;
	};
	
	/*void Depth::callback_1(const std_msgs::Float64::ConstPtr& sensors_1)
{ 
	ui_value.data = sensors_1->data;
	value.data = 42;
	
}
void Depth::callback_2(const vn_100::ins_data::ConstPtr& yaw)
{   
	yaw_value.data = yaw->YPR.x;
		
}*/	
/*
Depth::Depth()
{
	yaw_pub_1 = n_.advertise<std_msgs::UInt16>("frontpitchspeed",1);
	yaw_pub_2 = n_.advertise<std_msgs::UInt16>("backpitchspeed",1);
	yaw_pub_3 = n_.advertise<std_msgs::UInt16>("sideleftspeed",1);
	yaw_pub_4 = n_.advertise<std_msgs::UInt16>("siderightspeed",1);
  //sub_ = n_.subscribe<std_msgs::Float64>("/depth_value", 1 , &Depth::callback , this);
  //sub_2 = n_.subscribe<vn_100::ins_data>("/vn_100/ins_data",1,&Depth::callback, this);
  //sub_1 = n_.subscribe<std_msgs::Float64>("/ui_depth",1 , &Depth::callback_1, this);
}


void Depth::callback(const std_msgs::Float64::ConstPtr& sensors)
{
		cout << "i am in callback"<<endl;
	value.data = 42;
	error = setpoint_1 - sensors->data;
	
	
		integral = integral + error * dt;
		derivative = (error - previous_error)/dt;
		total_error = kp_1*error + ki_1*integral + kd_1*derivative ;
		
		add_value.data  = total_error/1000;
		
		value1.data = value.data + add_value.data;
		yaw_pub_1.publish(value1);
		yaw_pub_2.publish(value1);
		previous_error = error;
	//	value1.data = value.data;
	}
	*/

void callback(const std_msgs::Float64::ConstPtr& depth_value)
{
	
	ckpoint = depth_value->data;
}
int main(int argc , char **argv)
{
	ros::init(argc,argv,"depth_pid");
	ros::NodeHandle n;
	ros::Publisher leftpub=n.advertise<std_msgs::UInt16>("sideleftspeed",1);
	ros::Publisher rightpub=n.advertise<std_msgs::UInt16>("sideripghtspeed",1);
	ros::Publisher frontpub=n.advertise<std_msgs::UInt16>("frontpitchspeed",1);
	ros::Publisher backpub=n.advertise<std_msgs::UInt16>("backpitchspeed",1);
	ros::Subscriber depth_data = n.subscribe<std_msgs::Float64>("/depth_value", 1 ,callback);
	dynamic_reconfigure::Server<vn_100::pidConfig> server;
	dynamic_reconfigure::Server<vn_100::pidConfig>::CallbackType f;
	
	f = boost::bind(&callback_values, _1,_2);
	server.setCallback(f);
	
	  
	  //	ros::ServiceClient client = n.serviceClient<vn_100::depth_srv>("depth_srv");
	ros::Rate loop_rate(5);
	while(ros::ok())
	{
		
	 //	vn_100::depth_srv req;
	 // 	req.request.input = "request" ;
		  
		  
	  //  if (client.call(req))
	//	{
			cout << "i am in callback"<<endl;
			cout << endl;
				value.data = 44;
			error = setpoint_depth - ckpoint;
	
			cout<<"error : "<<error << endl;
			integral = integral + error * dt;
			integral = integral * i;
			derivative = (error - previous_error)/dt;
			total_error = kp_depth*error + ki_depth*integral + kd_depth*derivative ;
		
		    cout<<"integral  : "<<integral<<endl;
			
			add_value.data  = total_error/100;
		
			value1.data = 42 + add_value.data;
			value2.data = 42 +add_value.data;
			if(value1.data  <=  142 && error > 0)
			{
				frontpub.publish(value2);
				backpub.publish(value1);
			}
			else 
 			{
				value1.data = 42;
				value2.data = 42;
				frontpub.publish(value2);
				backpub.publish(value1);
			}
			previous_error = error;
	//	}	
			ros::spinOnce();
			loop_rate.sleep();
	}

	
	return 0;
}