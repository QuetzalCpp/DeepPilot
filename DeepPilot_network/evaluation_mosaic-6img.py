#!/usr/bin/env python
import argparse
import os
import numpy as np
import tensorflow as tf
#========== ROS  ===================
#=== OPENCV ====
import rospy
import cv2
#import video
import sys
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
#=== TWIST ====
from geometry_msgs.msg import Twist
#=== override ===
from std_msgs.msg import Int8
#----------------------------------------------
from keras.models import load_model
#--------------------------------------------
from time import time
import timeit
from std_msgs.msg import Float32MultiArray
import collections
#=============Library for DeepPilot==========================
import math
import network_libs.helper_net as helper_net
import network_libs.net as net
import numpy as np
from tensorflow.keras.optimizers import Adam 
from keras import backend as K

from    gazebo_msgs.srv    import *
import  rospy


class DeepPilot:
	
	def __init__(self):
		#====== ROS
		self.bridge = CvBridge()
		self.imag1 = rospy.Subscriber('/bebop2/camera_base/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		self.Ovr = rospy.Subscriber('/keyboard/override', Int8, self.flag)
		self.override = 0
		self.pub_cmd_vel_Estimation = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		
		#======= DeepPilot
		K.clear_session() # Clear previous models from memory.
		
		adam = Adam(lr=0.0001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0, clipvalue=2.0)
		
		#roll_pitch
		global model1 
		model1 = net.create_posenet_3_separated_outs()
		model1.load_weights('models/models_6img/6img_roll_pitch_model.h5')

		model1.compile(optimizer=adam, loss={'roll': net.euc_lossRoll,
												'pitch': net.euc_lossPitch,
												'yaw': net.euc_lossYaw, 
												'altitude': net.euc_lossAlt})
												
		#altitude
		global model2
		model2 = net.create_posenet_3_separated_outs()
		model2.load_weights('models/models_6img/6img_altitude_model.h5')

		model2.compile(optimizer=adam, loss={	'roll': net.euc_lossRoll,
												'pitch': net.euc_lossPitch,
												'yaw': net.euc_lossYaw, 
												'altitude': net.euc_lossAlt
											})
		#yaw										
		global model3
		model3 = net.create_posenet_3_separated_outs()
		model3.load_weights('models/models_6img/6img_yaw_model.h5')

		model3.compile(optimizer=adam, loss={'roll': net.euc_lossRoll,
												'pitch': net.euc_lossPitch,
												'yaw': net.euc_lossYaw, 
												'altitude': net.euc_lossAlt})
		self.q = collections.deque()
		
		self.frame_cont =  0
		self.cont   			=   0
		self.index   			=   0
		
		self.override = 0
		
		self.pitch = 0.0
		self.roll = 0.0
		self.altitude = 0.0
		self.yaw = 0.0
		
		self.elapse = 0.0
		self.lap_time = 0.0
		self.tic = 0.0
		self.toc = 0.0
		
		self.alpha = 0.1
		
		print("Loaded model from disk")
		
		self.get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		rospy.wait_for_service("/gazebo/get_model_state")
		
		self.model_state = GetModelStateRequest()
		self.model_state.model_name = 'bebop2'
		
		print('===============  READY ================================')
		print('===============  READY ================================')

	def callback(self,data):
		
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		frame = cv2.resize(frame, (640, 360))
		self.flag = 0
		
		self.frame_cont = self.frame_cont + 1

		if len(self.q) < 6 and self.frame_cont == 5: 
			
			if len(self.q) == 5:
				self.q.popleft()
			
			self.q.append(frame.copy()) 
			self.frame_cont = 0
					
		if len(self.q) == 5:
			self.flag = 1

			self.outputImage = np.zeros((720,1920, 3), dtype="uint8")
		
			self.outputImage[0:360, 0:640] = self.q[0]
			self.outputImage[0:360, 640:1280] = self.q[1]
			self.outputImage[0:360, 1280:1920] = self.q[2]
			
			self.outputImage[360:720, 0:640] = self.q[3]
			self.outputImage[360:720, 640:1280] = self.q[4]
			self.outputImage[360:720, 1280:1920] = frame
		
			if self.flag == 1:
				scale = cv2.resize(self.outputImage, (640, 360))
				cv2.imshow('Temporal view', scale)
				cv2.waitKey(1)
			
				im_to_pred = helper_net.preprocess_2(self.outputImage)
				im_to_pred = np.squeeze(np.array(im_to_pred))
				im_to_pred = np.expand_dims(im_to_pred, axis=0)
			
			start = time()
			
			speedpred = model1.predict(im_to_pred)
			speedpred_altitude = model2.predict(im_to_pred)
			speedpred_yaw = model3.predict(im_to_pred)
			
			self.elapse = time() - start
				

			self.pred_roll = round(speedpred[0][0][0],2)
			self.pred_pitch = round(speedpred[1][0][0],2)
			self.pred_yaw = round(speedpred_yaw[2][0][0],2)
			self.pred_altitude = round(speedpred_altitude[3][0][0],2) 
				
			self.pitch = round(self.alpha * self.pitch + (1 - self.alpha) * self.pred_pitch,2)
			self.roll = round(self.alpha * self.roll + (1 - self.alpha) * self.pred_roll,2)
			self.altitude = round(self.alpha * self.altitude + (1 - self.alpha) * self.pred_altitude,2)
			self.yaw = round(self.alpha * self.yaw + (1 - self.alpha) * self.pred_yaw,2)
			
			self.vel_msg.linear.x = self.pitch
			self.vel_msg.linear.y = self.roll
			self.vel_msg.linear.z = self.altitude
			self.vel_msg.angular.x = 0.0
			self.vel_msg.angular.y = 0.0
			self.vel_msg.angular.z = self.yaw
			
			
			if(self.override == 1):
				self.toc = time()
				self.pub_cmd_vel_Estimation.publish(self.vel_msg)
				self.lap_time = self.toc - self.tic 
				self.objstate = self.get_state_service(self.model_state)

				self.state = (self.objstate.pose.position.x, self.objstate.pose.position.y, self.objstate.pose.position.z)
				
			if(self.override == 0):
				self.tic = time()
				
				self.pitch = 0.0
				self.roll = 0.0
				self.altitude = 0.0
				self.yaw = 0.0
			
			print ('Flight Command predicted		|	Smoothed Flight Command')
			print ('')
			print ('Roll: ',  self.pred_roll, '		|	Roll: ',  self.roll )
			print ('Pitch: ',  self.pred_pitch, '		|	Pitch: ',  self.pitch)
			print ('Yaw: ',  self.pred_yaw, '		|	Yaw: ',  self.yaw )
			print ('Altitude: ',  self.pred_altitude, '	|	Altitude: ',  self.altitude )
			print ('Drone Pose: ', round(self.objstate.pose.position.x,2), ', ' , round(self.objstate.pose.position.y,2), ', ', round(self.objstate.pose.position.z,2))
			print ('')
			print ('Time of speed prediction: ', self.elapse , ' Seconds')
			print ('')
			print ('Time of lap: ', self.lap_time , ' Seconds')
			print ('')


	def flag(self,msg):
		if msg.data == 6:
			self.override = 1
		else:
			self.override = 0
		

				
def main():
	rospy.init_node('DeepPilot', anonymous = True)
	print("init_node DeepPilot")
	pred = DeepPilot()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		sys.exit()

if __name__ == '__main__':
	main()
