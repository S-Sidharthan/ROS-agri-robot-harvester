#!/usr/bin/env python3


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
                if distance > 1 :
                    continue

                print('distance {}'.format(distance))

                #transforming pixel coordinates to world coordinates
                world_x = (cX - center_x)/focal_length*distance
                world_y = (cY - center_y)/focal_length*distance
                world_z = distance
                #print("x={} y={} z={}".format(world_x,world_y,world_z))
                # broadcasting TF for each aruco marker
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera_link2"
                t.child_frame_id = "tom"+str(i)
            

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
    centers = []

    focal_length = 554.387
    center_x = 320.5 
    center_y = 240.5 
    tomato_dimension = 0.1
    
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        img = frame
        imgHSV = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        lower = np.array([0,60,0])
        upper = np.array([0,255,255])

        mask = cv.inRange(imgHSV,lower,upper)

        ret,thresh = cv.threshold(mask,127,255,0)

        

        #Find contours in binary image
        contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        i = 1
        loop = 1
        
        for c in contours:
            
            (cX,cY),radius = cv.minEnclosingCircle(c)
            cX = int(cX)
            cY = int(cY)
            center = (cX,cY)
            radius = int(radius)
            flag = 0
            
            
            if radius < 3:
                continue 

            if (cX,cY) not in centers :
                for j in centers:
                    if(abs(cX - j[0]) < 5 and abs(cY - j[1] < 5)):
                        flag = 1
                if(not flag):  
                    centers.append((cX, cY,0))
            
            #print("flag{} loop{} centrs {} \n".format(flag,loop,centers))
            loop +=1
            
            
            cv.circle(img,center,radius,(255,0,0),2)


            name = "obj" + str(i)
            cv.putText(img, name, center,cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            i += 1
        cv.imshow("Image", img)
        #cv.imshow("frame",frame)
        #cv.imshow("mask",mask)
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