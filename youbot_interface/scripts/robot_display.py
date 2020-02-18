#! /usr/bin/env python

import signal
import sys
import time
import roslib
import rospy
import tf
import numpy as np
import rospy
from std_msgs.msg import String
import roslaunch
import subprocess# as child
import os
import argparse
import cv2
import cv_bridge
from sensor_msgs.msg import Image as Image_Sensor_Msg

import Image
import ImageDraw
import ImageFont
import cStringIO
from PIL import Image as ImagePIL


packageName = 'pitt_object_table_segmentation'
LauncherName = 'table_segmentation.launch'
newCondition=False
runnerFlag=False
# child

def signal_handler(signal, frame):
    print('Bye!')
    sys.exit(0)
    
def callback(data):
    rospy.loginfo("robot display: I heard %s",data.data)
    DataList = [str(x) for x in data.data.split(' ')][0:]
    
    font2=cv2.FONT_HERSHEY_PLAIN
    img2=np.copy( img)
    
    print(len(DataList[0]),": ", DataList)
    if(DataList[1]=="Human"):
        cv2.putText(img2,DataList[1], (400,400), font2, 4,(0,255,0),4)
        cv2.putText(img2,DataList[0], (100,500), font2, 2,(0,255,0),4)
        
    elif(DataList[1]=="LeftArm" or DataList[1]=="RightArm" or DataList[1]=="LeftArm+RightArm" or DataList[1]=="RightArm+LeftArm"):
        cv2.putText(img2,DataList[1], (400,400), font2, 4,(0,0,255),4)

        
        if(len(DataList[0])>50):
            cv2.putText(img2,DataList[0], (10,500), font2, 2,(0,0,255),4)
        else:
            cv2.putText(img2,DataList[0], (100,500), font2, 2,(0,0,255),2)

            
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img2, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.1)


def main():
    rospy.init_node('robot_display')
    global pub
    pub=rospy.Publisher('/robot/xdisplay', Image_Sensor_Msg, latch=True , queue_size=10)
    Sub = rospy.Subscriber('robotDisplayText', String, callback)
    
    path='/home/nasa/catkin_ws/src/ROBOT_INTERFACE/robot_interface/files/black_Background.png'
    
    global img
    img = cv2.imread(path)    
    global font
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,'HUMAN',            (100,150), font, 3,(0,255,0),10)
    cv2.putText(img,'ROBOT',            (550,150), font, 3,(0,0,250),10)
    cv2.putText(img,'+',                (450,150), font, 3,(250,250,250),10)

    cv2.putText(img,'Please Perform:',  (100,250), font, 1.5,(255,255,255),5)
    cv2.putText(img,'Performs:',        (550,250), font, 1.5,(255,255,255),5)

    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    
    print type(img )
    rospy.sleep(0.1)


    rospy.spin()
    signal.signal(signal.SIGINT, signal_handler)

    print('Press Ctrl+C')
#     signal.pause()

if __name__ == '__main__':
    main()
