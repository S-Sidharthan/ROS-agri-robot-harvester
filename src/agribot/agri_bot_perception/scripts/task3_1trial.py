#!/usr/bin/env python3

"""
Description: Obtaining TF of a ArUco marker using single sjcam camera
Algorithm: 
    Input: ROS topic for the RGB image
    Process: 
        - Subscribe to the image topic
        - Convert ROS format to OpenCV format
        - Detecting ArUco marker along with its ID
        - Applying perspective projection to calculate focal length
        - Extracting depth for each aurco marker
        - Broadcasting TF of each ArUco marker with naming convention as aruco1 for ID1, aruco2 for ID2 and so on.
    Output: TF of ArUco marker with respect to ebot_base
"""

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

cv_image = None
centers = []




def caldepth(ros_image):
    global centers
    try:
        focal_length = 554.387
        center_x = 320.5 
        center_y = 240.5



        cv_bridge = CvBridge()

     
        depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
       
        depth_array = np.array(depth_image, dtype=np.float32)

        #rospy.loginfo(depth_array)
        if centers :
            for i in centers:
                cX = i[0]
                cY = i[1]
                
                distance = (depth_array[cY,cX])
                print('distance {}'.format(distance))

                #transforming pixel coordinates to world coordinates
                world_x = (cX - center_x)/focal_length*distance
                world_y = (cY - center_y)/focal_length*distance
                world_z = distance
                print("x={} y={} z={}".format(world_x,world_y,world_z))
                # broadcasting TF for each aruco marker
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera_link2"
            

                # # putting world coordinates coordinates as viewed for sjcam frame
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
       

                

        

    except CvBridgeError as e:
        print(e)    





def detect(data):
    # # Initializing variables  
    global centers 
    focal_length = 554.387
    center_x = 320.5 
    center_y = 240.5 
    tomato_dimension = 0.1
    
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        
        imgHSV = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        lower = np.array([0,60,0])
        upper = np.array([0,255,255])

        mask = cv.inRange(imgHSV,lower,upper)

        #imgResult = cv.bitwise_and(frame,frame,mask=mask)

        
        #cv.imshow("imageResult",imgResult)
        #Change image to greyscale
        ret,thresh = cv.threshold(mask,127,255,0)

        

        #Find contours in binary image
        contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        i = 1
        for c in contours:
            # M = cv.moments(c)
            # if M["m00"] == 0:
            #     continue
            # # calculate moments for each contour
    
            # cX = int(M["m10"] / M["m00"])
            # cY = int(M["m01"] / M["m00"])
            (cX,cY),radius = cv.minEnclosingCircle(c)
            cX = int(cX)
            cY = int(cY)
            center = (cX,cY)
            radius = int(radius)
            

            if radius < 3:
                continue 

            if (cX,cY) not in centers :
                centers.append((cX, cY))
                i += 1
            else:
                i = 1
                centers = []

            
            
            cv.circle(img,center,radius,(255,0,0),2)


            #cv.circle(img, (cX, cY), 2, (255, 255, 255), -1)
            #cv.circle(img,(cX,cY),30,(255,0,0),2)

            name = "obj" + str(i)
            cv.putText(img, name, center,cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # pixel_width = 2*radius

            # # # obtain depth for each ArUco marker
            # distance = (focal_length*tomato_dimension)/pixel_width

            # # #transforming pixel coordinates to world coordinates
            # world_x = (cX - center_x)/focal_length*distance
            # world_y = (cY - center_y)/focal_length*distance
            # world_z = distance
            # print("{} x={} y={} z={}".format(name,world_x,world_y,world_z))
            # # # broadcasting TF for each aruco marker
            # br = tf2_ros.TransformBroadcaster()
            # t = geometry_msgs.msg.TransformStamped()
            # t.header.stamp = rospy.Time.now()
            # t.header.frame_id = "camera_link2"
            

            # # # putting world coordinates coordinates as viewed for sjcam frame
            # t.transform.translation.x = world_z
            # t.transform.translation.y = -world_x
            # t.transform.translation.z = world_y
            # # not extracting any orientation thus orientation is (0, 0, 0)
            # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            # t.transform.rotation.x = q[0]
            # t.transform.rotation.y = q[1]
            # t.transform.rotation.z = q[2]
            # t.transform.rotation.w = q[3]

            # br.sendTransform(t)

           

         # display the image
        cv.imshow("Image", img)
        cv.imshow("frame",frame)
        cv.imshow("mask",mask)
        cv.waitKey(1)   
    
        

      
    except CvBridgeError as e:
        print(e)



def main(args):
    rospy.init_node('camera_tf', anonymous=True)
    # subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, detect)
    rospy.Subscriber("/camera/depth/image_raw2", Image, caldepth)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)