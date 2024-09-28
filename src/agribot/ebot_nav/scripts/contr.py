#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import math


pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

roll = pitch = yaw = 0.0

east = 0.0  * math.pi/180
west = 180.0 * math.pi/180
north = 90 * math.pi/180
south = -90 * math.pi/180
direction = 0.0


def turn_ebot():
	global direction
	global pub 
	target = direction
	msg = Twist()
	while(target != yaw):
		msg.angular.z = target - yaw
		pub.publish(msg)
	return msg


def odom_callback(data):
	global roll , pitch , yaw 
	orientations = data.pose.pose.orientation
	orientations_ls = [orientations.x,orientations.y,orientations.z,orientations.w] 


	(roll , pitch , yaw) = euler_from_quaternion (orientations_ls)

def laser_callback(msg):
	global regions
	#regions = {
	#   'fright':   ,
	#   'front':    ,
	#   'bleft':    ,
	#}



def control_loop():
	global direction, pub
	rospy.init_node('ebot_contoller')

	rospy.Subscriber('/ebot/laser/scan',LaserScan,laser_callback)
	rospy.Subscriber('/odom',Odometry,odom_callback)


	rate = rospy.Rate(1)
	i=0

	velocity_msg = Twist()
	velocity_msg.linear.x = 0
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)


	while not rospy.is_shutdown():
		
		
		velocity_msg.angular.z =  - 1
	

		#velocity_msg.linear.x = 
		#velocity_msg.angular.z =
		#pub.publish(velocity_msg)
		print("target={}       z = {}".format(direction,velocity_msg.angular.z))
		print("Controller message pushed at {}".format(rospy.get_time()))
		pub.publish(velocity_msg)
		rate.sleep()


	
if __name__ == '__main__':
	try:
		control_loop()

	except rospy.ROSInterruptException:
		pass    
