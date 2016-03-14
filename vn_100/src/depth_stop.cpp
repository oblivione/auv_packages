#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <vn_100/ins_data.h>
#include <dynamic_reconfigure/server.h>
#include <vn_100/pidConfig.h>

float kp,ki,kd,setpoint,error,previous_error,total_error,integral,derivative,dt=10;
float kp_1,ki_1,kd_1,setpoint_1,error_1,previous_error_1,total_error_1,integral_1,derivative_1,dt_1=10;
float ITerm,last_pitchvalue;

	 std_msgs::UInt16 value;
	 std_msgs::UInt16 value1;
	 std_msgs::UInt16  add_value;
	 std_msgs::UInt16  add_value_1;
	 std_msgs::Float64 ui_value;
	 std_msgs::Float64 pitch_value;
	
void callback_values(vn_100::pidConfig &config,uint32_t level)
{
	kp = config.kp ;
		ki = config.ki ;
		 kd=config.kd;
		kp_1 = config.kp_1 ;
		 ki_1 = config.ki_1 ;
		 kd_1 = config.kd_1;
		
}
class Depth
{
	public:
	
	Depth();
	void callback(const std_msgs::Float64::ConstPtr& sensors);
	void callback_1(const std_msgs::Float64::ConstPtr& sensors_1);
	void callback_2(const vn_100::ins_data::ConstPtr& pitch);
	         
	            ros::NodeHandle n_;
                ros::Subscriber sub_;
                ros::Subscriber sub_1;
                ros::Subscriber sub_2;
                ros::Publisher pub_1;
                ros::Publisher pub_2;
                ros::Publisher pub_3;
                ros::Publisher pub_4;
	};
	
Depth::Depth()
{
	pub_1 = n_.advertise<std_msgs::UInt16>("frontpitchspeed",1);
	pub_2 = n_.advertise<std_msgs::UInt16>("backpitchspeed",1);
	pub_3 = n_.advertise<std_msgs::UInt16>("sideleftspeed",1);
	pub_4 = n_.advertise<std_msgs::UInt16>("siderightspeed",1);
	sub_ = n_.subscribe<std_msgs::Float64>("depth_val", 1 , &Depth::callback , this);
	sub_2 = n_.subscribe<vn_100::ins_data>("/ins_data",1,&Depth::callback_2, this);
	sub_1 = n_.subscribe<std_msgs::Float64>("/ui_depth",1 , &Depth::callback_1, this);
}

void Depth::callback_1(const std_msgs::Float64::ConstPtr& sensors_1)
{ 
	ui_value.data = sensors_1->data;
	value.data = 42;
	
}
void Depth::callback_2(const vn_100::ins_data::ConstPtr& pitch)
{   
	pitch_value.data = pitch->YPR.y;
		
}
void Depth::callback(const std_msgs::Float64::ConstPtr& sensors)
{
	error = pitch_value.data;
	error_1 = ui_value.data - sensors->data; 
	if(sensors->data <= ui_value.data &&  pitch_value.data > -10.00 && pitch_value.data < 10.00)
	{
		integral_1 = integral_1 + error_1 * dt_1;
		derivative_1 = (error_1 - previous_error_1)/dt_1;
		total_error_1 = kp_1*error_1 + ki_1*integral_1 + kd_1*derivative_1;
		
		add_value_1.data = total_error_1;
		value1.data = value.data + add_value_1.data;
		pub_1.publish(value1);
		pub_2.publish(value1);
	}
	else if(sensors->data < ui_value.data)
	{
		integral = integral + error * dt;
		derivative = (error - previous_error)/dt;
		total_error = kp*error + ki*integral + kd*derivative ;
		
		add_value.data  = total_error;
		if(add_value.data <0)
		{
				value1.data = value.data + add_value.data;
				pub_1.publish(value1);
				pub_2.publish(value);
		}
		else 
		{
				value1.data = value.data - add_value.data;
				pub_1.publish(value);
				pub_2.publish(value1);
		}
		
		previous_error = error;
	//	value1.data = value.data;
	}
}
int main(int argc , char **argv)
{
	ros::init(argc,argv,"depth_stop");
	dynamic_reconfigure::Server<vn_100::pidConfig> server;
	dynamic_reconfigure::Server<vn_100::pidConfig>::CallbackType f;
	
	f = boost::bind(&callback_values, _1,_2);
	server.setCallback(f);
	
	Depth sub_pub;
	ros::spin();
	return 0;
	}
