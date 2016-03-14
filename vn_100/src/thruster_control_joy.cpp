#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
using namespace std;

class Thruster
{
	public:
		Thruster();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		
		ros::NodeHandle nh_;
	
		//int linear_,angular_;
		int thruster_on,thruster_off,thruster_init;
		int runmode,f;
		//double l_scale_, a_scale_;
		ros::Publisher thruster_but_pub;
		ros::Publisher frontpitchspeedPub;
		ros::Publisher backpitchspeedPub;
		ros::Publisher sideleftspeedPub;
		ros::Publisher siderightspeedPub;
		ros::Subscriber joy_sub_;
};

Thruster::Thruster():
	//linear_(1),
	//angular_(2),
	thruster_off(1),
	thruster_on(2),
	thruster_init(3),
	runmode(0),
	f(0)
{
	//nh_.param("axis_linear",linear_,linear_);
	//nh_.param("axis_angular",angular_,angular_);
	//nh_.param("scale_angular",a_scale_,a_scale_);
	//nh_.param("scale_linear",l_scale_,l_scale_);

	//twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	thruster_but_pub = nh_.advertise<std_msgs::UInt16>("thrusterstate",1 ); //on,off,init
	frontpitchspeedPub = nh_.advertise<std_msgs::UInt16>("frontpitchspeed",1 );
	backpitchspeedPub = nh_.advertise<std_msgs::UInt16>("backpitchspeed",1 );
	sideleftspeedPub = nh_.advertise<std_msgs::UInt16>("sideleftspeed",1 );
	siderightspeedPub = nh_.advertise<std_msgs::UInt16>("siderightspeed",1);
	
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",10,&Thruster::joyCallback,this);
}

void Thruster::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//geometry_msgs::Twist twist;
	std_msgs::UInt16 msg;
	//twist.angular.z = a_scale_*joy->axes[angular_];
	//twist.linear.x = l_scale_*joy->axes[linear_];
	cout<<"button value(off)="<<joy->buttons[thruster_off]<<endl;
	cout<<"button value(on)="<<joy->buttons[thruster_on]<<endl;
	cout<<"button value(init)="<<joy->buttons[thruster_init]<<endl;

	if(joy->buttons[runmode] == 1)
	{
		f++;
		if(f == 2)
		{
		 	f = 0;
		}
	}
		cout<<"f="<<f<<endl;
		if(joy->buttons[thruster_off] == 1)
		{
			msg.data = 3;
		}
		else if(joy->buttons[thruster_on] == 1)
		{
			msg.data = 2;
		}
		else if(joy->buttons[thruster_init] == 1)
		{
			msg.data = 1;
		}
		thruster_but_pub.publish(msg);

		std_msgs::UInt16 msgf; //front
		std_msgs::UInt16 msgb; //back
		std_msgs::UInt16 msgl; //left
		std_msgs::UInt16 msgr; //right
		
		/*Default values:*/
		msgf.data = 1000;  //axes[1] = 1
		msgb.data = 1000;  // axes[1] = -1
		msgl.data = 1500; //axes[2] = 1
		msgr.data = 1500; //axes[2] = -1

		cout<<"axes[1]="<<joy->axes[1]<<endl;
		cout<<"axes[2]="<<joy->axes[2]<<endl;
		cout<<"axes[3]="<<joy->axes[3]<<endl;
		cout<<"axes[4]="<<joy->axes[4]<<endl;


	switch(f)
	{
		case 1:

		if( joy->axes[1] <= 1 && joy->axes[1] >=0)
		{
			msgf.data +=  (int) (joy->axes[1] * 1000);  //map it
			msgb.data = msgf.data;
		}
		else if( joy->axes[1] >= -1 && joy->axes[1] <=0 )
		{
			msgb.data -=  (int) (joy->axes[1] * 1000); //map it
			msgf.data = msgb.data;
		}
		if( joy->axes[2] <=1 && joy->axes[2] >=0)
		{
			cout<<"loop left entered"<<endl;
			msgl.data += (int)(joy->axes[2] * 500); //map it
			msgr.data = msgl.data;
		}
		if( joy->axes[2] >=-1 && joy->axes[2] <=0)
		{
			cout<<"right loop entered"<<endl;
			msgr.data += (int)(joy->axes[2] * 500);//map it
			msgl.data = msgr.data;
		}
	
		cout<<"msgf="<<msgf.data<<endl;
        	cout<<"msgb="<<msgb.data<<endl;
        	cout<<"msgl="<<msgl.data<<endl;
        	cout<<"msgr="<<msgr.data<<endl;

		frontpitchspeedPub.publish(msgf);
        	backpitchspeedPub.publish(msgb);
        	sideleftspeedPub.publish(msgl);
        	siderightspeedPub.publish(msgr);

		break;
		
		case 0:
		
		if( joy->axes[1] <= 1 && joy->axes[1] >=0)
		{
			msgf.data +=  (int) (joy->axes[1] * 1000);  //map it
			//msgb.data = msgf.data;
		}
		else if( joy->axes[1] >= -1 && joy->axes[1] <=0 )
		{
			msgb.data -=  (int) (joy->axes[1] * 1000); //map it
			//msgf.data = msgb.data;
		}
		if( joy->axes[2] <=1 && joy->axes[2] >=0)
		{
			cout<<"loop left entered"<<endl;
			msgl.data += (int)(joy->axes[2] * 500); //map it
			//msgr.data = msgl.data;
		}
		if( joy->axes[2] >=-1 && joy->axes[2] <=0)
		{
			cout<<"right loop entered"<<endl;
			msgr.data += (int)(joy->axes[2] * 500);//map it
			//msgl.data = msgr.data;
		}
	
		cout<<"msgf="<<msgf.data<<endl;
        	cout<<"msgb="<<msgb.data<<endl;
        	cout<<"msgl="<<msgl.data<<endl;
        	cout<<"msgr="<<msgr.data<<endl;

		frontpitchspeedPub.publish(msgf);
        	backpitchspeedPub.publish(msgb);
        	sideleftspeedPub.publish(msgl);
        	siderightspeedPub.publish(msgr);

		break;

		default:
		cout<<"wrong choice"<<endl;
	}
	//thruster on: data = 2, off: data=3,initialize : data=1
	//twist_pub_.publish(twist);	
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"thruster_control_joy");
	Thruster thruster;

	ros::spin();
}
