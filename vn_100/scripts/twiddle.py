#!/usr/bin/env python
import rospy
from std_msgs.msg import String,UInt16
from vn_100.msg import ins_data
from dynamic_reconfigure.server import Server
from vn_100.cfg import pitchparamsConfig
kp_pitch,ki_pitch,kd_pitch,setpoint,ckpoint,presenttime,pasttime,error,integral,derivative,best_error=0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00
frontpitchpub=rospy.Publisher("frontpitchspeed",UInt16,queue_size=1)
backpitchpub=rospy.Publisher("backpitchspeed",UInt16,queue_size=1)
depthhelppub=rospy.Publisher("depthhelp",UInt16,queue_size=1)
best_error = 0.00
p = [kp_pitch,ki_pitch,kd_pitch]
dp= [1,0.5,1]


def run(p):

	global ckpoint,presenttime,pasttime,setpoint,error,integral,best_error,derivative,frontpitchpub
	
	p[1] = kp_pitch
	p[2] = ki_pitch
	p[3] = kd_pitch
	
	if pasttime == 0.00 :
		pasttime=presenttime
	error=ckpoint-setpoint
	errorcomp=kp_pitch*error
	timediff=((presenttime-pasttime)/1000000000)
	#print timediff
	integral=integral+(error*timediff)
	integralcomp=integral*ki_pitch
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
	frontpitchpub.publish(finalval)
	depthhelppub.publish(finalval1)
	pasttime=presenttime
	
	return error,p
	
def twiddle(total = 0.0005):
	global best_error,error,dp,p,kp_pitch,ki_pitch,kd_pitch
	
	best_error = run(p)
	print p 
	while sum(dp) > total:
		for i in range (3):
			p[i] += dp[i]
			error = run(p)
			#print p
			if error < best_error:
				best_error = error
				dp[i] *=1.1
			else:
				p[i] -= 2.0*dp[i]
				error = run(p)
				#print p
				if error < best_error:
					best_error = error
					dp[i] *=1.1
				else:
				 	p[i] += dp[i]
					dp[i] *= 0.9
		print p	 		
		return p

		
def pidcallback(msg):
	global ckpoint,presenttime,pasttime,setpoint,error,integral,best_error,derivative,frontpitchpub
	ckpoint=msg.YPR.y
	presenttime=float(str(msg.header.stamp))
	#print finalval
def callback(config,level):
	global kp_pitch,ki_pitch,kd_pitch,setpoint,integral,best_error
	if config.bool_param==True :
		kp_pitch=config.kp_pitch
		ki_pitch=config.ki_pitch
		kd_pitch=config.kd_pitch
		setpoint=config.setpoint
		integral=0
	if config.bool_twiddle==True:
		best_error=config.best_error;
		print kp_pitch,ki_pitch,kd_pitch
	return config	
pitchdatasub=rospy.Subscriber("/vn_100/ins_data",ins_data,pidcallback)
if __name__ == "__main__":
	rospy.init_node("pitch_pid",anonymous=True)
	srv=Server(pitchparamsConfig,callback)
	rospy.spin()