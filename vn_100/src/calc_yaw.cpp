#include<iostream>
#include<ctime>
#include<cmath>
#include<ros/ros.h>
#include "std_msgs/Float64.h"
#include <vn_100/ins_data.h>

//variables for kalman filter
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
//function definations
	double getAngle(double,double,double);
	//void setAngle(double); // Used to set angle, this should be set as the starting angle
    //double getRate(); // Return the unbiased rate
/*
    /* These are used to tune the Kalman filter 
    */

    double Q_angle= 0.001;
    double    Q_bias = 0.003;
    double    R_measure = 0.03;
    
	
	 double acx=0.0;
	double acly=0.0;
	double aclz=0.0;
	double gyroX=0.0;
	double gyroY=0.0;
	double gyroZ=0.0;
	double yaw=0.0,dt=0.0,gyroZrate=0.0,kalmanYaw=0.0;	



    double getAngle(double newAngle, double newRate, double dt) {
        
        /* Step 1 */
        rate = newRate - bias;
        angle += dt * rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
 		 /* Step 3 */
        y = newAngle - angle;
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P[0][0] + R_measure;
        /* Step 5 */
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
       
        /* Step 6 */
        angle += K[0] * y;
        bias += K[1] * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return angle;
    }
	//void setAngle(double newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
   // double getRate() { return rate; }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    /*void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
    void setRmeasure(double newR_measure) { R_measure = newR_measure; };

    double getQangle() { return Q_angle; };
    double getQbias() { return Q_bias; };
    double getRmeasure() { return R_measure; };
*/
void callback(const vn_100::ins_data::ConstPtr& ypr_values)
{
     gyroX=ypr_values->YPR.x;
	 gyroY=ypr_values->YPR.y;
	 gyroZ=ypr_values->YPR.z;
}
int main(int argc,char **argv)
{
   ros::init(argc, argv, "yaw_finder");
   ros::NodeHandle yf;
   ros::Publisher conf_code=yf.advertise<std_msgs::Float64>("/code", 1);

	ros::Publisher pub=n.advertise<std_msgs::Float64>("Yaw value",1);
    ros::Subscriber value = yf.subscribe<vn_100::ins_data>("/vn_100/sensor_data", 1 , callback);
	std_msgs::Float64 msg1;
	double timer=clock();//
	 
	while(true)
	{
		if(/*IMU data node is giving value*/)   //give the condition such that if we are subscribing the IMU code then only it will print the yaw otherwise it will print zero
		{
			while(1)
			{
			  acx=;    //subscribe the values accleration and gyro value accordingly accordingly(see the co-ordinates)
			  acly=;
			  aclz=;
			//  gyroX=;
			 // gyroY=;
			  gyroZ=;	
			 
			 
			 yaw = 180 * atan (aclZ/sqrt(aclX*aclX + aclZ*aclZ))/M_PI;
			 dt = (double)(clock() - timer) / (double) CLOCKS_PER_SEC; // Calculate delta time
             timer = clock();
			 gyroZrate = gyroZ / 131.0; // Convert to deg/s
			 kalmanYaw=getAngle(yaw,gyroZrate,dt);
			 msg1.data=kalmanYaw;
			 pub.publish(msg1);
			 
			
			 	
			}
		}
		else
		{
		  msg1.data=0.0;
		  pub.publish(msg1);
		  
			
		}
	}
	return 0;
	
	
}