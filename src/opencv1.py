#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from baxter_core_msgs.msg import EndpointState


bridge = CvBridge() #Creates obj for sending poop btwn ros and opencv

def imagecb(data):
    global bridge

    #Converts Image message to CV image with blue-green-red color order (bgr8)
    try: #if no image, then throw out error but continue
        img_original = bridge.imgmsg_to_cv2(data, "bgr8")


#COLOR TRACKING
        #Converts bgr8 to hsv
        hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

        #Def range of red color in HSV
        color_dict = {
            'blue':{
                'lower':np.array([105,135,20]),
                'upper':np.array([115,160,60])
            },
            'red':{
                'lower':np.array([-50,100,100]),
                'upper':np.array([10,255,255])
            },
            'yellow': {
                'lower':np.array([0,0,0]),
                'upper':np.array([0,0,0])
            },
            'green': {
                'lower':np.array([0,0,0]),
                'upper':np.array([0,0,0])
            },
            'purple': {
                'lower':np.array([0,0,0]),
                'upper':np.array([0,0,0])
            }
        }
        # lower_yellow = np.array([22, 0, 0])
        # upper_yellow = np.array([38, 255, 255])
        
        # lower_green = np.array([38, 0, 0])
        # upper_green = np.array([75, 255, 255])
        
        
        
        # lower_purple = np.array([130, 0, 0])
        # upper_purple = np.array([160, 255, 255])
        
        # lower_green = np.array([160, 0, 0])
        # upper_green = np.array([179, 255, 255])
        


        # lower_blue = np.array([120, 50, 50])
        # upper_blue = np.array([180, 255, 255])


        # lower_red = np.array([-50, 100, 100])
        # upper_red = np.array([10, 255, 255])

        #Def mask using set hsv range
        color = 'red'
        mask = cv2.inRange(hsv,color_dict[color]['lower'] ,color_dict[color]['upper'])
        mask = cv2.erode(mask,None,iterations=5)
        mask = cv2.dilate(mask,None,iterations=5)

        #Mask and original image overlay
        res = cv2.bitwise_and(img_original,img_original, mask= mask)

        #Creating b w image from res  (outputs binary matrices)
        ret, thresh = cv2.threshold(res[:,:,2], 100, 255, cv2.THRESH_BINARY)


        #Find and creat the circle
        cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c=max(cnts,key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print center
            height,width,depth=img_original.shape
            if radius > 10:

                cv2.circle(img_original,(int(x), int(y)), int(radius), (0,255,255),2)
                cv2.circle(img_original,center,5,(0,0,255),-1)

            #Have to convert pixel points to 3D points for baxter as follows
            calibration = 0.0023
            cam_disp_x = 0.02
            cam_disp_y = -0.02
            z_disp = rospy.get_param('/zoff')
            cx = center[0]
            cy = center[1]
            print 'Bpx=',Bpx
            print 'Bpy=',Bpy
            xb = (cy - 0.5*height)*calibration*z_disp + Bpx + cam_disp_x
            yb = (cx - 0.5*width)*calibration*z_disp + Bpy + cam_disp_y
            print 'xb=',xb
            print 'yb=',yb

#DISPLAY WHAT CAMERA IS PUBLISHING TO opencv2
        cv2.imshow("colorout", res)
        #cv2.imshow("contout", thresh)
        cv2.imshow("original",img_original)

        cv2.waitKey(20) #Updates with next cached img every 0.01sec

    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)


def get_body_stuff(data):
    global Bpx, Bpy
    Bpx = data.pose.position.x
    Bpy = data.pose.position.y

def listener():
    rospy.init_node('listener',anonymous=True)
    #Initializes node
    #create a publisher to publish xb and yb
    #subscribe to color topic
    

    

    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,get_body_stuff)
    rospy.Subscriber("/cameras/left_hand_camera/image",Image,imagecb)
    #Def node as subscriber with rostopic
    rospy.spin()
    #Loops python until node stopped
    #Let's create a service that takes in color and returns the x and y

if __name__ == '__main__':
    listener()



