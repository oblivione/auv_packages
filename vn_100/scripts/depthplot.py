#!/usr/bin/env python
import matplotlib.pyplot as plt
from drawnow import *
import rospy
from std_msgs.msg import Float32,Float64,UInt16

depth=[]
time=[]
plt.ion()
prev_val=994
depth.append(994)
time.append(0)
def makefig():
	plt.grid(True)
	plt.plot(time,depth)
class plotty():
	def __init__(self,parent=None):
		self.depthsub=rospy.Subscriber("/depth_value",Float64,self.callback)
		self.depth=994.00
	def callback(self,msg):
		self.depth=msg.data
def main():
	global depth,time,presentime,pasttime,prev_val
	rospy.init_node("depth_plot",anonymous=True)
	match=plotty()
	rate=rospy.Rate(100)
	count=0
	count1=0
	while not rospy.is_shutdown():
		#print match.depth
		if match.depth >1300 or match.depth<960:
			depth.append(prev_val)
		else:
			depth.append(float(match.depth))
			prev_val=match.depth
		time.append(count)
		try:
			drawnow(makefig)
		except:
			pass
			
		plt.pause(0.00001)
		count+=0.01
		count1+=1
		if count1>300 :
			depth.pop(0)
			time.pop(0)
		#print count1
		#rate.sleep()
if __name__=="__main__" :
	main()