#!/usr/bin/env python
import matplotlib.pyplot as plt
from drawnow import *
import rospy
#from vn_100.msg import ins_data
from sensor_msgs.msg import Imu
yaw=[]
time=[]
plt.ion()
def makefig():
	plt.grid(True)
	plt.plot(time,yaw)
class plotty():
	def __init__(self,parent=None):
		self.pitchsub=rospy.Subscriber("/ardrone/imu",Imu,self.callback)
		self.yaw=0.00
	def callback(self,msg):
		self.yaw=msg.linear_acceleration.x
def main():
	global yaw, time
	rospy.init_node("yaw_plot",anonymous=True)
	match=plotty()
	rate=rospy.Rate(10)
	count=0
	count1=0
	while not rospy.is_shutdown():
		yaw.append(float(match.yaw))
		time.append(count)
		try:
			drawnow(makefig)
		except:
			pass
			
		plt.pause(0.00001)
		count+=0.1
		count1+=1
		if count1>300 :
			yaw.pop(0)
			time.pop(0)
		print count1
		rate.sleep()
if __name__=="__main__":
	main()