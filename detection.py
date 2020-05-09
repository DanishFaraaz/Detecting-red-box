#!/usr/bin/env python

import sys
import rospy
import cv2
import time
import imutils

from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Pose
from std_msgs.msg           import Bool
from std_msgs.msg           import Float64
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg      import Twist

class BoxDetector:

    def __init__(self):
    
        rospy.init_node("Box_location")

        self.colorThresh = 220

        print (">> Publisher box box found or not found")
        self.box_bool = rospy.Publisher("/box_bool",Bool,queue_size=10)
    
        print (">> Publishing box location to topic /box_location")
        self.pubBoxLoc = rospy.Publisher("/box_location",Pose,queue_size=10)

        print (">> Publishing turtlebot commands to topic /cmd_vel")
        self.turtle_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /camera/rgb/image_raw")
        
        self.cv_image = None
        
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    
    def location(self):

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.cv_image is not None:
                img_rgb = self.cv_image
                
                img_b = img_rgb[:,:,0]
                img_g = img_rgb[:,:,1]
                img_r = img_rgb[:,:,2]


                img_r_binary = np.zeros_like(img_r)
                img_b_binary = np.zeros_like(img_r)
                img_g_binary = np.zeros_like(img_r)

                img_r_binary = (img_r>100) & (img_r<=200)
                img_b_binary = (img_b>100) & (img_b<=200)
                img_g_binary[(img_g>100) & (img_g<=200)] = 1

                img_marker = np.zeros_like(img_r)
                img_marker[ (img_r_binary) ^ (img_b_binary)] = 255

                xsum = 0
                ysum = 0
                
                temp = np.argwhere(img_marker>self.colorThresh)
                
                if (len(temp) > 800):
                    b_bool = True
                    self.box_bool.publish(b_bool)
                else:
                    b_bool = False
                    self.box_bool.publish(b_bool)

                if(len(temp)==0):
                    xloc=999
                    yloc=999
                else:
                    for t in temp:
                        xsum += t[1]
                        ysum += t[0]  
                    xloc = xsum/len(temp)
                    yloc = ysum/len(temp)

                msgOut= Pose()
                msgOut.position.x = xloc
                msgOut.position.y = yloc
                self.pubBoxLoc.publish(msgOut)

                if (len(temp)>20000):
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0

                    self.turtle_vel.publish(twist)
                    
                rate.sleep()




if __name__ == '__main__':
    try:
        b = BoxDetector()
        b.location()
    except rospy.ROSInterruptException:
        pass
