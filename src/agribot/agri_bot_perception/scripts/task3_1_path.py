#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time


from tf.transformations import euler_from_quaternion
import math

import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import time


regions = {
		'bright': 0.0,
		'fright': 0.0,
		'front':  0.0,
		'fleft':  0.0,
		'bleft':  0.0
	}

stop = False



#the arena is given cardinal directions which is used by odom to turn to particular direction 

east = 0.0  * math.pi/180              #converted to radians 
west = 180.0 * math.pi/180
north = 90 * math.pi/180
south = -90 * math.pi/180

roll = pitch = 0.0
yaw = north

direction = 0.0                        # to set the directions

checkpoint = 0                         # To store the checkpoint values 

state = 0                       # To store current state 

state_dict = {
	0:  'doing nothing',
	1:  'going straight',
	2:  'changing direction',
	3:  'stop'                #Dictionary to define states
	
}

cv_image = None
arucos_visited = [False] * 20

def callback(data):
	# Initializing variables
	global cv_image,stop
	focal_length = 476.70308
	center_x = 400.5
	center_y = 400.5
	aruco_dimension = 0.1
	try:
		bridge = CvBridge()
		frame = bridge.imgmsg_to_cv2(data, "bgr8")
		frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

		# load the dictionary that was used to generate the markers
		dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_1000)

		# initializing the detector parameters with default values
		parameters =  cv.aruco.DetectorParameters_create()

		# detect the markers in the frame
		corners, ids, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

		if len(corners) > 0:
			# Flatten the ArUco IDs list
			ids = ids.flatten()
			# loop over the detected ArUCo corners
			for (markerCorner, markerID) in zip(corners, ids):
				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners
				# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				# draw the bounding box of the ArUCo detection
				cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
				cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
				cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
				cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
				# compute and draw the center (x, y)-coordinates of the ArUco
				# marker
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
				
				pixel_width = topLeft[1] - bottomRight[1]

				# draw the ArUco marker ID on the frame
				cv.putText(frame, str(markerID),
					(topLeft[0], topLeft[1] - 15), cv.FONT_HERSHEY_SIMPLEX,
					0.5, (0, 255, 0), 2)
				
				'''uncomment to view aruco ID and verify the working of the c'''
				print("[INFO] ArUco marker ID: {}".format(markerID))
			   

				# obtain depth for each ArUco marker
				distance = (focal_length*aruco_dimension)/pixel_width
				if(distance < 0.5 and arucos_visited[markerID] == False):

					arucos_visited[markerID] = True
					
					stop = True
					print(stop)

				# transforming pixel coordinates to world coordinates
				world_x = (cX - center_x)/focal_length*distance
				world_y = (cY - center_y)/focal_length*distance
				world_z = distance

				# broadcasting TF for each aruco marker
				br = tf2_ros.TransformBroadcaster()
				t = geometry_msgs.msg.TransformStamped()
				t.header.stamp = rospy.Time.now()
				t.header.frame_id = "camera_link2"
				t.child_frame_id = "aruco"+str(markerID)

				# putting world coordinates coordinates as viewed for sjcam frame
				t.transform.translation.x = world_z
				t.transform.translation.y = -world_x
				t.transform.translation.z = world_y
				# not extracting any orientation thus orientation is (0, 0, 0)
				q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
				t.transform.rotation.x = q[0]
				t.transform.rotation.y = q[1]
				t.transform.rotation.z = q[2]
				t.transform.rotation.w = q[3]

				br.sendTransform(t)

		'''uncoment to view the visual of detection'''
		cv.imshow("frame", frame)
		print(arucos_visited)
		cv.waitKey(1)
	except CvBridgeError as e:
		print(e)


#function to delay 
def delay(time_s):
	time.sleep(time_s)


def odom_callback(data):
	global roll , pitch , yaw 
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
		msg.angular.z = 0.8

	elif ((target - yaw) < 0):
		msg.angular.z = -0.8


	print("target = {}  yaw = {} checkpoint ={}".format(target,yaw,checkpoint))
	return msg

##function to make bot go straight
def go_straight():
	print("in go_straight")
	global direction , yaw , regions
	regions_ = regions
	target = direction
	msg = Twist()
	msg.linear.x = 1

	if  (regions_['bright'] < 0.5):
		msg.angular.z = 0.5 
	elif (regions_['bleft'] < 0.5):
		msg.angular.z = -0.5
	else :
		msg.angular.z = (target - yaw )# avoid any deviations or error (P controller)   

	return msg

##function which detects when wall ends (wall following)
def define_state():
	global regions
	regions_ = regions
	msg = Twist()

	if (checkpoint == 3 ):                                          # this checkpoint front it has a wall but still has to take turn so only bright is checked      
		if (regions_['bleft'] > 1.4):
			delay(2)
			change_state(2)

	elif (checkpoint == 1 or checkpoint == 2 or checkpoint == 4 ):        # these checkpoints only require to check wall on left 
		if  (regions_['fleft'] > 1.5 and regions_['bleft'] > 1.5) : 
			delay(2)
			change_state(2)

	
		
	print(regions_)
		

##function to change state
def change_state(state_):
	global state, state_dict
	if state_ is not state:
		state = state_

def control_loop():
	global state,direction,checkpoint,stop

  

	route = {                             #dictionary to store the turns based on checkpoints

		0:east,
		1:north,
		2:west,
		3:west,
		4:south,
			
	}

   
	checkpoint = 0
	msg = Twist()
	rospy.init_node('ebot_contoller')

	pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
	rospy.Subscriber('/ebot/laser/scan',LaserScan,laser_callback)
	rospy.Subscriber('/odom',Odometry,odom_callback)

	
	#subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
	image_sub = rospy.Subscriber("/ebot/camera1/image_raw", Image, callback)

	rate = rospy.Rate(10)

	state = 2                         #state is 2 (changing direction) because we have to take turn in the begining


	while not rospy.is_shutdown() :
		
		define_state() 
		print(stop)
		if(stop):
			print("config stop")
			change_state(3)
			delay(2)
			change_state(1)
			stop = False            

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
