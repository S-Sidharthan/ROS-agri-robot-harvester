#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time

from tf.transformations import euler_from_quaternion
import math

regions = {
		'bright': 0.0,
		'fright': 0.0,
		'front':  0.0,
		'fleft':  0.0,
		'bleft':  0.0
	}

roll = pitch = yaw = 0.0

odom_dist = 0

#the arena is given cardinal directions which is used by odom to turn to particular direction 

east = 0.0  * math.pi/180              #converted to radians 
west = 180.0 * math.pi/180
north = 90 * math.pi/180
south = -90 * math.pi/180

direction = 0.0                        # to set the directions

checkpoint = 0                         # To store the checkpoint values 

state = 0                       # To store current state 

state_dict = {
	0: 	'doing nothing',
	1:	'going straight',
	2:	'changing direction',
	3:  'stop'                #Dictionary to define states
	
}


#function to delay 
def delay(time_s):
	time.sleep(time_s)


def odom_callback(data):
	global roll , pitch , yaw , odom_dist
	orientations = data.pose.pose.orientation
	orientations_q = [orientations.x,orientations.y,orientations.z,orientations.w] 




	(roll , pitch , yaw) = euler_from_quaternion (orientations_q)
	


def laser_callback(msg):
	global regions
	regions = {
		'bright': min(min(msg.ranges[0:143]), 8)  , #143
		'fright': min(min(msg.ranges[144:287]), 8)  , #287
		'front':  min(min(msg.ranges[288:432]), 8)  , #432 #359 centre af
		'fleft':  min(min(msg.ranges[433:576]), 8)  , #576
		'bleft':  min(min(msg.ranges[577:719]), 8)  , #719
	}

	


#function to turn the bot
def turn_ebot():
	print("in turn_ebot")
	global direction,checkpoint,yaw
	target = round(direction,1)
	msg = Twist()
	yaw = round(yaw,1)
	msg.linear.x=0
	
	#To rotate accordingly

	if (target==yaw):             
		msg.angular.z = 0
		print("orientation reached")
		delay(0.01)
		checkpoint = checkpoint+1
		change_state(1)

	elif ((target - yaw)>0):
		msg.angular.z = 0.4

	elif ((target - yaw) < 0):
		msg.angular.z = -0.4


	print("target = {}  yaw = {} checkpoint ={}".format(target,yaw,checkpoint))
	return msg

##function to make bot go straight
def go_straight():
	print("in go_straight")
	global direction , yaw , regions
	regions_ = regions
	target = direction
	msg = Twist()
	msg.linear.x = 0.8

	if  (regions_['bright'] < 0.5):
		msg.angular.z = 0.3 
	elif (regions_['bleft'] < 0.5):
		msg.angular.z = -0.3
	else :
		msg.angular.z = (target - yaw )# avoid any deviations or error (P controller)	

	return msg

##function which detects when wall ends (wall following)
def define_state():
	global regions
	regions_ = regions
	msg = Twist()
	
	if  (checkpoint == 1 or checkpoint == 2 ):                           # these checkpoints only require to check wall on right
		if  (regions_['fright'] > 1.4  and regions_['bright'] > 1.4) : 
			delay(1.7)
			change_state(2)

	elif (checkpoint == 3 ):											# this checkpoint front it has a wall but still has to take turn so only bright is checked		
		if (regions_['bright'] > 1.4):
			delay(1.7)
			change_state(2)

	elif (checkpoint == 4 or checkpoint == 5 or checkpoint == 6 ):        # these checkpoints only require to check wall on left 
		if  (regions_['fleft'] > 1.5 and regions_['bleft'] > 1.5) : 
			delay(1.7)
			change_state(2)

	
		
	print(regions_)
		

##function to change 7state
def change_state(state_):
	global state, state_dict
	if state_ is not state:
		state = state_

def control_loop():
	global state,direction,checkpoint

	delay(8)  

	route = {                             #dictionary to store the turns based on checkpoints

		0:west,
		1:north,
		2:east,
		3:south,
		4:east,
		5:north,
		6:west	
	}

   
	checkpoint = 0
	msg = Twist()
	rospy.init_node('ebot_contoller')

	pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
	rospy.Subscriber('/ebot/laser/scan',LaserScan,laser_callback)
	rospy.Subscriber('/odom',Odometry,odom_callback)

	rate = rospy.Rate(10)

	state = 2                         #state is 2 (changing direction) because we have to take turn in the begining


	while not rospy.is_shutdown() :
		
		define_state() 

		            

		if state == 0 :
			pass

		elif state == 1 :       
			msg = go_straight()
			
		elif state == 2 :                    # as different turns has to be taken based on the checkpoints
			if (checkpoint == 0) :
				direction = route[0]
				msg = turn_ebot()
			elif (checkpoint == 1) :
				direction = route[1]
				msg = turn_ebot()
			elif (checkpoint == 2) :
				direction = route[2]
				msg = turn_ebot()
			elif (checkpoint == 3) :
				direction = route[3]
				msg = turn_ebot()
			elif (checkpoint == 4) :
				direction = route[4]
				msg = turn_ebot()
			elif (checkpoint == 5) :
				direction = route[5]
				msg = turn_ebot()
			elif (checkpoint == 6) :
				change_state(3)
				#delay(1)
				#rospy.signal_shutdown('OVER :)') 	
			else :
				rospy.logerr('checkpoint defination over ! ABORT!!')
				change_state(3) 
		
		elif state == 3 :
			msg.angular.z = 0
			msg.linear.x = 0					
	
		else:
			rospy.logerr('Unknown state!')
		
		pub.publish(msg)
		print("x = {} z = {} State = {} checkpoint = {}".format(msg.linear.x,msg.angular.z,state,checkpoint))
		print("Controller message pushed at {}".format(rospy.get_time()))
		
		rate.sleep()

if __name__ == '__main__':
	try:
		control_loop()

	except rospy.ROSInterruptException:
		pass  	
