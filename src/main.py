#!/usr/bin/env python3
import numpy as np
import rospy
# import roslib; roslib.load_manifest('gazebo')
import time
from cv_bridge import CvBridge
import cv2

from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState 
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

from math import pi

import sys
import os

class RobotTaxi(object):

    def __init__(self):
        self.loop_rate = rospy.Rate(1)
        self.br = CvBridge()

        ## Subscribers
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageSub)

        ## Publishers
        self.vel_pub=rospy.Publisher("/cmd_vel", Twist,queue_size=1)

        # self.boxes=np.arange(-0.5,0.6,0.05)
        self.boxes_cte=np.linspace(-0.5,0.5,4)
        self.boxes_he=np.linspace(-np.pi/4,np.pi/4,4)
        
        reg_speed=0.13
        max_speed=0.2

        reg_ang=0.13
        max_ang=0.2

        no_segments_cte=len(self.boxes_cte)-1
        no_segments_he=len(self.boxes_he)-1

        self.boxes_speed_cte=np.zeros([no_segments_cte,2])
        self.boxes_ang_he=np.zeros([no_segments_he,2])

        val_speed=(max_speed-reg_speed)/(np.ceil(no_segments_cte/2.0))
        val_ang=(max_ang-reg_ang)/(np.ceil(no_segments_he/2.0))

        ## Setting up for CTE
        for i in (np.arange(int(no_segments_cte/2))):
            vel_arr=np.array([reg_speed, max_speed-i*val_speed])
            self.boxes_speed_cte[i]=vel_arr
            self.boxes_speed_cte[no_segments_cte-i-1]=np.flip(vel_arr,0)
        self.boxes_speed_cte[int(no_segments_cte/2)]=np.array([reg_speed,reg_speed])

        ## Setting up for HE
        for i in (np.arange(int(no_segments_he/2))):
            vel_arr=np.array([reg_ang, max_ang-i*val_ang])
            self.boxes_ang_he[i]=vel_arr
            self.boxes_ang_he[no_segments_he-i-1]=np.flip(vel_arr,0)
        self.boxes_ang_he[int(no_segments_he/2)]=np.array([reg_ang,reg_ang])
        print(self.boxes_ang_he)
        

    def imageSub(self, data):
        pos=self.gms_client('robot','').pose
        pos_y=pos.position.y

        ml=0.0
        mr=0.0

        ml_speed=0.0
        mr_speed=0.0

        ml_ang=0.0
        mr_ang=0.0

        for i in np.arange(len(self.boxes_cte)):
            # print(pos_y, self.boxes_cte[i])
            if pos_y<self.boxes_cte[i]:
                ml_speed=self.boxes_speed_cte[i-1][0]
                mr_speed=self.boxes_speed_cte[i-1][1]
                break

        angle=2*np.arcsin(pos.orientation.z)
        for i in np.arange(len(self.boxes_he)):
            # print(angle, self.boxes_he[i])
            if angle<self.boxes_he[i]:
                ml_ang=self.boxes_ang_he[i-1][0]
                mr_ang=self.boxes_ang_he[i-1][1]
                break
        # print(2*np.arcsin(angle))
        print(ml_speed, mr_speed)
        print(ml_ang, mr_ang)
        print('\n\n')

        ## Without heading error
        # ml=ml_speed
        # mr=mr_speed

        ## With heading error
        ml=(ml_speed+ml_ang)/2
        mr=(mr_speed+mr_ang)/2

        vel=Twist()
        vel.linear.x=(ml+mr)/2.0
        # ml_actual=ml+noise
        # vel.linear.x=(ml_actual+mr)/2.0
        vel.angular.z=(2*(mr-vel.linear.x))/0.306
        self.vel_pub.publish(vel)


    ## To get robot's position
    def gms_client(self,model_name,relative_entity_name):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms(model_name,relative_entity_name)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def start(self):
        while not rospy.is_shutdown():
            self.loop_rate.sleep()

if __name__=='__main__':
    rospy.init_node("taxinet", anonymous=True)
    robot=RobotTaxi()
    robot.start()