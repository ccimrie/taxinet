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
from tempfile import TemporaryFile

from math import pi

import sys
import os
import time

class RobotTaxi(object):

    def __init__(self):
        print("done")
        self.loop_rate = rospy.Rate(1)
        self.br = CvBridge()
        self.path=('/media/cci502/Extreme SSD/dataset_taxinet/')
        # self.data=open('/media/cci502/Extreme SSD/dataset_3/data.txt','w')
        # self.label=open(self.path+'label.txt','w')
        self.length=1000
        self.count=0
        self.count_file=0
        self.data=np.zeros([self.length,480,640,3])
        self.data_resized=np.zeros([self.length,64,48,3])
        self.label=np.zeros([self.length,2])
        self.test=0
        self.start_ind=133
        np.set_printoptions(threshold=sys.maxsize)
        np.set_printoptions(linewidth=np.inf)
        np.random.seed(seed=int(time.time()))
        ## Subscribers
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageSub)

    ## IMAGE INPUT
    def imageSub(self, data):
        # np.set_printoptions(threshold=sys.maxsize)
        # np.set_printoptions(linewidth=np.inf)
        if self.test<17000:
            image=self.br.imgmsg_to_cv2(data)
            # self.data=np.append(self.data, np.array([image]), axis=0)
            self.data[self.count,:,:,:]=image
            # image_string=np.array2string(image).splitlines()
            # self.data.write(''.join(image_string))
            # self.data.write('\n')

            pos=self.gms_client('robot','').pose
            pos_y=pos.position.y
            angle=2*np.arcsin(pos.orientation.z)
            # self.label.write(str(pos_y)+'  '+str(angle)+'\n')
            # self.label=np.append(self.label,np.array([[pos_y,angle]]),axis=0)
            self.label[self.count,:]=np.array([pos_y,angle])

            rand_pos=np.array([np.random.random()*3,(1-2*np.random.random())*0.5])
            rand_angle=(1-2*np.random.random())*(np.pi/2.0)
            self.smsClient('robot', rand_pos,np.sin(rand_angle*0.5))

            self.test+=1
            self.count+=1
            rate = rospy.Rate(10)
            rate.sleep()
            # if self.test%(self.length-1)==0:
            # print(self.count, self.length)
            if self.count==self.length:
                print(self.test)
                # if self.test>self.length:
                # # if True:
                #     # outfile=TemporaryFile()
                #     data=np.load(self.path+'data.npy')
                #     label=np.load(self.path+'label.npy')
                #     data_new=np.append(data,self.data,axis=0)
                #     label_new=np.append(label,self.label,axis=0)
                #     np.save(self.path+'data.npy',data_new)
                #     np.save(self.path+'label.npy',label_new)
                #     self.data[:,:,:,:]=0
                #     self.label[:,:]=0
                #     self.count=0
                # else:
                for i in np.arange(self.length):
                    # print("here")
                    temp=cv2.resize(self.data[i], dsize=(48, 64), interpolation=cv2.INTER_CUBIC)
                    # print(temp)
                    # self.data_resized[i]=temp
                    # print(self.data_resized[i])
                # resized_img = cv2.resize(self.data, dsize=(64, 48), interpolation=cv2.INTER_CUBIC)
                print(np.shape(self.data_resized))
                np.save(self.path+'data_'+str(self.count_file+self.start_ind)+'.npy',self.data_resized)
                np.save(self.path+'label_'+str(self.count_file+self.start_ind)+'.npy',self.label)
                self.data[:,:,:,:]=0
                self.data_resized[:,:,:,:]=0
                self.label[:,:]=0
                self.count=0
                self.count_file+=1

    ## To get robot's position
    def gms_client(self,model_name,relative_entity_name):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms(model_name,relative_entity_name)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def smsClient(self, model_name, pos, orient):
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose.position.x = pos[0]
        state_msg.pose.position.y = pos[1]
        state_msg.pose.position.z = 0.0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = orient
        state_msg.pose.orientation.w = 1.0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def start(self):
        while not rospy.is_shutdown():
            self.loop_rate.sleep()

if __name__=='__main__':
    rospy.init_node("taxinet", anonymous=True)
    robot=RobotTaxi()
    robot.start()