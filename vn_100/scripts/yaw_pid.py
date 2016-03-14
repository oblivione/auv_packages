#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16
from vn_100.msg import ins_data
from dynamic_reconfigure.server import Server
from vn_100.cfg import pidConfig
kp_yaw,ki_yaw,kd_yaw,setpoint_yaw,ckpoint,presenttime,pasttime,error,integral,derivative=0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00
max_error,min_error
sideleftspeedpub=rospy.Publisher("/sideleftspeed",UInt16,queue_size=1)
siderightspeedpub=rospy.Publisher("/siderightspeed",UInt16,queue_size=1)

def pidcallback(msg):
	global ckpoint,presenttime,pasttime,setpoint_yaw,error,integral,derivative,frontpitchpub,max_error,min_error
	ckpoint=msg.YPR.x
	presenttime=float(str(msg.header.stamp))
	if pasttime == 0.00 :
		pasttime=presenttime
	error=setpoint_yaw - ckpoint 
	max_error = setpoint_yaw + 90
	min_error = setpoint_yaw - 90
	print "error" , error
	errorcomp=kp_yaw*error
	timediff=((presenttime-pasttime)/1000000000)
	#print timediff
	integral=integral+(error*timediff)
	integralcomp=integral*ki_yaw
	if timediff!=0 :
		derivative=error/timediff
	derivativecomp=kd_yaw*derivative
	totalerror=errorcomp+integralcomp+derivativecomp
	#if error<0 :
	#	totalerror=totalerror-errorcomp
	#finalval=1500+totalerror
	#if finalval<42:
		#finalval=42
	#elif finalval>142:
		#finalval=142
	#finalval1=finalval-1000
	print "error value :-",error
	if(error > 0):
		finalval = 1500 - totalerror
		finalval1 = 1500 + totalerror
		print "finalval : ",finalval
		print "finalval1 :",finalval1
		if(finalval1 >2000):
				finalval1=2000
		sideleftspeedpub.publish(finalval)
 		siderightspeedpub.publish(finalval1)
	else:
		finalval = 1500 + totalerror
		finalval1 = 1500 - totalerror
		print "finalval : ",finalval
		print "finalval1 :",finalval1
		if(finalval > 2000):
			finalval = 2000
		sideleftspeedpub.publish(finalval)
		siderightspeedpub.publish(finalval1)
		
		
	pasttime=presenttime
	#print finalval
def callback(config,level):
	global kp_yaw,ki_yaw,kd_yaw,setpoint_yaw,integral,ckpoint
	if config.bool_param==True :
		kp_yaw=config.kp_yaw
		ki_yaw=config.ki_yaw
		kd_yaw=config.kd_yaw
		setpoint_yaw=config.setpoint_yaw
		if(setpoint_yaw == 0):
			setpoint_yaw = ckpoint
		integral=0
		#print kp_pitch,ki_pitch,kd_pitch
	return config	
yawdatasub=rospy.Subscriber("/vn_100/ins_data",ins_data,pidcallback)
if __name__ == "__main__":
	rospy.init_node("yaw_pid",anonymous=True)
	srv=Server(pidConfig,callback)
	rospy.spin()