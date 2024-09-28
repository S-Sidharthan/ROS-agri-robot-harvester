#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import math

i = 1

regions = {
		'bright': 0.0,
		'fright': 0.0,
		'front':  0.0,
		'fleft':  0.0,
		'bleft':  0.0
	}
roll = pitch = yaw = 0.0

east = 0.0  * math.pi/180
west = 180.0 * math.pi/180
north = 90 * math.pi/180
south = -90 * math.pi/180


def odom_callback(data):
	global roll , pitch , yaw 
	orientations = data.pose.pose.orientation
	orientations_ls = [orientations.x,orientations.y,orientations.z,orientations.w] 


	(roll , pitch , yaw) = euler_from_quaternion (orientations_ls)

def laser_callback(msg):
	global regions
	regions = {
		'bright': min(min(msg.ranges[0:143]), 8)  , #143
		'fright': min(min(msg.ranges[144:287]), 8)  , #287
		'front':  min(min(msg.ranges[288:432]), 8)  , #432 #359 centre af
		'fleft':  min(min(msg.ranges[433:576]), 8)  , #576
		'bleft':  min(min(msg.ranges[577:719]), 8)  , #719
	}



def control_loop():
	global i
	rospy.init_node('ebot_contoller')

	pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
	rospy.Subscriber('/ebot/laser/scan',LaserScan,laser_callback)
	rospy.Subscriber('/odom',Odometry,odom_callback)


	rate = rospy.Rate(.5)

	i = i + 1
	velocity_msg = Twist()
	velocity_msg.linear.x = 0
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)


	while not rospy.is_shutdown():
		
		#target = south
		#velocity_msg.angular.z = (target - yaw )  
	
		leftcheck = regions['bleft'] 
		rightcheck  = regions['bright']
		
		
		#pub.publish(velocity_msg)
		#print("target={}     	Current = {}".format(target,yaw))
		 

		#print("Controller message pushed at {}".format(rospy_time()))
		 

		#print("bright = {} \nfright = {}\nfront = {}\nfleft = {}\nbleft = {}".format(regions['bright'],regions['fright'],regions['front'],regions['fleft'],regions['bleft']))
		#print("leftcheck = {}   rightcheck = {}   z ={}".format(leftcheck,rightcheck,velocity_msg.angular.z))
		print(regions) 
		rate.sleep()


	
if __name__ == '__main__':
	try:
		control_loop()

	except rospy.ROSInterruptException:
		pass  	
