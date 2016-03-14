#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16
from vn_100.msg import ins_data
from dynamic_reconfigure.server import Server
from vn_100.cfg import pitchparamsConfig
kp_pitch,ki_pitch,kd_pitch,setpoint,ckpoint,presenttime,pasttime,error,integral,derivative=0,0,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00
frontpitchpub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
backpitchpub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
depthhelppub=rospy.Publisher("depthhelp",UInt16,queue_size=1)
def pidcallback(msg):
	global ckpoint,presenttime,pasttime,setpoint,error,integral,derivative,frontpitchpub
	ckpoint=msg.YPR.y
	presenttime=float(str(msg.header.stamp))
	if pasttime == 0.00 :
		pasttime=presenttime
	error=setpoint-ckpoint
	errorcomp=kp_pitch*error
	#print errorcomp
	timediff=((presenttime-pasttime)/1000000000)
	print "timediff",timediff
	integral=integral+(error*timediff)
	integralcomp=integral*ki_pitch
	print "integral",integral
	print integralcomp
	if timediff!=0 :
		derivative=error/timediff
	derivativecomp=kd_pitch*derivative
	totalerror=errorcomp+integralcomp+derivativecomp
	if error<0 :
		totalerror=totalerror-errorcomp
	finalval=1000+totalerror
	#if finalval<42:
		#finalval=42
	#elif finalval>142:
		#finalval=142
	finalval1=finalval-1000
	#frontpitchpub.publish(finalval)
	backpitchpub.publish(finalval)
	print "finalval",finalval1
	depthhelppub.publish(finalval1)
	pasttime=presenttime
	#print finalval
def callback(config,level):
	global kp_pitch,ki_pitch,kd_pitch,setpoint,integral
	if config.bool_param==True :
		kp_pitch=config.kp_pitch
		ki_pitch=config.ki_pitch
		kd_pitch=config.kd_pitch
		setpoint=config.setpoint
		integral=0
		#print kp_pitch,ki_pitch,kd_pitch
	return config	
pitchdatasub=rospy.Subscriber("/vn_100/ins_data",ins_data,pidcallback)
if __name__ == "__main__":
	rospy.init_node("pitch_pid",anonymous=True)
	srv=Server(pitchparamsConfig,callback)
	rospy.spin()