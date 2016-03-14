#!/usr/bin/env python
import matplotlib.pyplot as plt
from drawnow import *
import rospy
from vn_100.msg import ins_data
pitch=[]
time=[]
plt.ion()
def makefig():
	plt.grid(True)
	plt.plot(time,pitch)
class plotty():
	def __init__(self,parent=None):
		self.pitchsub=rospy.Subscriber("/vn_100/ins_data",ins_data,self.callback)
		self.pitch=0.00
	def callback(self,msg):
		self.pitch=msg.YPR.y
def main():
	global pitch, time
	rospy.init_node("pitch_plot",anonymous=True)
	match=plotty()
	rate=rospy.Rate(10)
	count=0
	count1=0
	while not rospy.is_shutdown():
		pitch.append(float(match.pitch))
		time.append(count)
		try:
			drawnow(makefig)
		except:
			pass
			
		plt.pause(0.00001)
		count+=0.1
		count1+=1
		if count1>300 :
			pitch.pop(0)
			time.pop(0)
		print count1
		rate.sleep()
if __name__=="__main__":
	main()