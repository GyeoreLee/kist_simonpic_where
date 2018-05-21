#!/usr/bin/env python

# this module is to get ROI and find distance (x,y,z) from depth images
# Create: 2016-06-31 by levuanh.hut@gmail.com
# Modify: 2016-09-12 by Sang-Seok Yun (yssmecha@gmail.com)

import roslib
#roslib.load_manifest('human_detector_cnn')
from roslib import scriptutil

import sys, select, termios, tty
import sys, os, time
import rospy
from time import time
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from numpy import *
import numpy as np
from threading import Lock
from pub_msgs.msg import where_msgs
from pub_msgs.msg import roilocation_msgs
from geometry_msgs.msg import Point32

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

import scipy.io as sio
import argparse
import scipy
import cv2


#import lib for kalman tracking
from math import * 
import math
#import gmphd4 as gmphd
#import gmphdROI as gmphdroi
import pickle


fusion_mutex = Lock()

class HumanDetector_RGBD:
	def __init__(self):
		self.init_variable()
		rospy.init_node('sp_hd_gather', anonymous=False)
		rospy.logwarn("sp_hd_gather.py starts...")

		### sub and pub topics
		self.bridge = CvBridge()		
		rospy.Subscriber("/sn_kinect1/depth/depth_roi", Image, self.depth_callback1, queue_size = 1)
		rospy.Subscriber("/sn_kinect2/depth/depth_roi", Image, self.depth_callback2, queue_size = 1)
		rospy.Subscriber("/sn_kinect3/depth/depth_roi", Image, self.depth_callback3, queue_size = 1)
		rospy.Subscriber("/sn_kinect4/depth/depth_roi", Image, self.depth_callback4, queue_size = 1)
				
		rospy.Subscriber("/sn_kinect/detector1", where_msgs, self.where_callback1, queue_size = 2)
		rospy.Subscriber("/sn_kinect/detector2", where_msgs, self.where_callback2, queue_size = 2)
		rospy.Subscriber("/sn_kinect/detector3", where_msgs, self.where_callback3, queue_size = 2)
		rospy.Subscriber("/sn_kinect/detector4", where_msgs, self.where_callback4, queue_size = 2)
		
		self.pub_detection = rospy.Publisher("/sn_kinect/detector", where_msgs, queue_size = 1)

		#fig = plt.figure()
		#ax=fig.add_subplot(111)
		plt.ion()
		#plt.show()

		rospy.Timer(rospy.Duration(0.03), self.timer_callback)
		#rospy.Timer(rospy.Duration(0.01), self.timer_callback)
	
	
	def init_variable(self):
		self.verbose = rospy.get_param('verbose', True)
		self.scale = rospy.get_param('scale', '0.6')
		self.szW = rospy.get_param('szW', '50')
		self.szH = rospy.get_param('szH', '50')
		self.szWH = rospy.get_param('szWH', '1200')
		self.szTOP= rospy.get_param('szTOP', '2.3')
		self.display = rospy.get_param('display', '2')
		print '\nparam: verbose {}, {}, {}, {}, {}, {}'.format(self.verbose, self.scale, self.szW, self.szH, self.szWH, self.szTOP) 
		
		self.psn1 = rospy.get_param("/psn_unit1")
		self.psn2 = rospy.get_param("/psn_unit2")
		self.psn3 = rospy.get_param("/psn_unit3")
		self.psn4 = rospy.get_param("/psn_unit4")

		self.depth1 = None
		self.depth2 = None
		self.depth3 = None
		self.depth4 = None
		self.depth_roi_mask_image1=None
		self.depth_roi_mask_image2=None
		self.depth_roi_mask_image3=None
		self.depth_roi_mask_image4=None
		self.depth_crop=None
		self.counter = 0
		
		self.kinect1_timer = int(time())
		self.kinect2_timer = int(time())
		self.kinect3_timer = int(time())
		self.kinect4_timer = int(time())
		self.kinect1_dat = where_msgs()
		self.kinect2_dat = where_msgs()
		self.kinect3_dat = where_msgs()
		self.kinect4_dat = where_msgs()
		self.depth_kinect1_ready=0
		self.depth_kinect2_ready=0
		self.depth_kinect3_ready=0
		self.depth_kinect4_ready=0
		self.depth_roi_mask_image1_ready=0
		self.depth_roi_mask_image2_ready=0
		self.depth_roi_mask_image3_ready=0
		self.depth_roi_mask_image4_ready=0

		'''
		### kalman tracking parameter setting for tracking ROI
#		sigma_q_roi = 67
#		sigma_r_roi = 60
#		sigma_p0_roi = 55
#		sigma_v_roi = 2
#		p_d_roi = 0.25
#		p_s_roi = 0.59
#		merge_thresh_roi = 30
#		F_roi = [[1, 0, 0, 0, 1, 0, 0, 0], 
#                [0, 1, 0, 0, 0, 1, 0 ,0], 
#                [0, 0, 1, 0, 0, 0 ,1, 0], 
#                [0, 0, 0, 1, 0, 0, 0, 1],
#                [0, 0, 0, 0, 1, 0, 0 ,0],                 
#                [0, 0, 0, 0, 0, 1, 0 ,0], 
#                [0, 0, 0, 0, 0, 0 ,1, 0], 
#                [0, 0, 0, 0, 0, 0, 0, 1]]
#		v_roi = math.pow(sigma_v_roi, 2)
#		q_roi = math.pow(sigma_q_roi, 2)
#		Q_roi = [[q_roi, 0, 0, 0, 0, 0, 0, 0], 
#                [0, q_roi, 0, 0, 0, 0, 0 ,0], 
#                [0, 0, q_roi, 0, 0, 0, 0, 0], 
#                [0, 0, 0, q_roi, 0, 0, 0, 0],
#                [0, 0, 0, 0, v_roi, 0, 0, 0],                 
#                [0, 0, 0, 0, 0, v_roi, 0 ,0], 
#                [0, 0, 0, 0, 0, 0 ,v_roi, 0], 
#                [0, 0, 0, 0, 0, 0, 0, v_roi]]
#		H_roi = [[1, 0, 0, 0, 0, 0, 0, 0], 
#                [0, 1, 0, 0, 0, 0, 0 ,0], 
#                [0, 0, 1, 0, 0, 0 ,0, 0], 
#                [0, 0, 0, 1, 0, 0, 0, 0]]
#		R_roi = [[math.pow(sigma_r_roi, 2), 0, 0, 0], [0, math.pow(sigma_r_roi, 2), 0, 0], [0, 0, math.pow(sigma_r_roi, 2), 0], [0, 0, 0, math.pow(sigma_r_roi, 2)]]
#		p_roi = math.pow(sigma_p0_roi, 2)
#		P0_roi = [[p_roi, 0, 0, 0, 0, 0, 0, 0], 
#                [0, p_roi, 0, 0, 0, 0, 0 ,0], 
#                [0, 0, p_roi, 0, 0, 0, 0, 0], 
#                [0, 0, 0, p_roi, 0, 0, 0, 0],
#                [0, 0, 0, 0, v_roi, 0, 0, 0],                 
#                [0, 0, 0, 0, 0, v_roi, 0 ,0], 
#                [0, 0, 0, 0, 0, 0 ,v_roi, 0], 
#                [0, 0, 0, 0, 0, 0, 0, v_roi]]
#		clutter_intensity_roi = 0.0
#		self.f_gmphd_roi = gmphdroi.GMPHD([], p_s_roi, p_d_roi, F_roi, Q_roi, H_roi, R_roi, P0_roi, clutter_intensity_roi, merge_thresh_roi)

		'''
		### kalman tracking parameter setting
		sigma_q = 0.5 #0.2
		sigma_r = 0.7 #0.1
		sigma_p0 = 0.7
		p_d = 0.2
		p_s = 0.99
		merge_thresh = 0.1
		F = [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]]
		Q = [[math.pow(sigma_q, 2), 0, 0, 0], [0, math.pow(sigma_q, 2), 0, 0], [0, 0, 0.001, 0], [0, 0, 0, 0.001]]
		H = [[1, 0, 0, 0], [0, 1, 0, 0]]
		R = [[math.pow(2*sigma_r, 2), 0], [0, math.pow(2*sigma_r, 2)]]
		P0 = [[math.pow(sigma_p0, 2), 0, 0, 0], [0, math.pow(sigma_p0, 2), 0, 0], [0, 0, 0.01, 0], [0, 0, 0, 0.01]]
		clutter_intensity = 0.0
		self.born_components = []
		#self.f_gmphd = gmphd.GMPHD([], p_s, p_d, F, Q, H, R, P0, clutter_intensity, merge_thresh)
		########################

		self.isBusy = True
		rospy.sleep(1)
		

	### subscribe data depth with only ROI region				
	def depth_callback1(self,data) :		
		### convert ROS image messages to OpenCV images
		self.depth1 = self.bridge.imgmsg_to_cv2(data, "16UC1")
		self.depth_kinect1_ready=1
		self.depth1[np.isnan(self.depth1)] = np.median(self.depth1)
		
		if self.verbose and self.display==1 and self.depth_roi_mask_image1_ready==0:
			cv_image_mask = np.array(self.depth1, dtype=np.float32)
			cv2.normalize(cv_image_mask, cv_image_mask, 0, 1, cv2.NORM_MINMAX)
			small = cv2.resize(cv_image_mask, (0,0), fx=self.scale, fy=self.scale)                     
			### display depth_roi via roi on where_msgs
			k1 = self.kinect1_dat
			for n in range(len(k1.roi)):
				x_ltop = k1.roi[n].x_offset 
				y_ltop = k1.roi[n].y_offset
				x_rbtn = k1.roi[n].x_offset + k1.roi[n].width 
				y_rbtn = k1.roi[n].y_offset + k1.roi[n].height
				if k1.roi[n].width < self.szW and k1.roi[n].height <self.szH:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 1)
				else:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 3)
			cv2.imshow('Depth_ROI 1: ', small)
			cv2.waitKey(3)
	
	def depth_callback2(self,data):
		self.depth2 = self.bridge.imgmsg_to_cv2(data, "16UC1")
		self.depth_kinect2_ready=1
		self.depth2[np.isnan(self.depth2)] = np.median(self.depth2)

		if self.verbose and self.display==2 and self.depth_roi_mask_image2_ready==0:
			cv_image_mask = np.array(self.depth2, dtype=np.float32)
			cv2.normalize(cv_image_mask, cv_image_mask, 0, 1, cv2.NORM_MINMAX)
			small = cv2.resize(cv_image_mask, (0,0), fx=self.scale, fy=self.scale)                    
			### display depth_roi via roi on where_msgs
			k2 = self.kinect2_dat
			for n in range(len(k2.roi)):
				x_ltop = k2.roi[n].x_offset 
				y_ltop = k2.roi[n].y_offset
				x_rbtn = k2.roi[n].x_offset + k2.roi[n].width 
				y_rbtn = k2.roi[n].y_offset + k2.roi[n].height
				if k2.roi[n].width < self.szW and k2.roi[n].height <self.szH:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 1)
				else:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 3)
			cv2.imshow('Depth_ROI 2: ', small)
			cv2.waitKey(3)
	
	def depth_callback3(self,data):
		self.depth3 = self.bridge.imgmsg_to_cv2(data, "16UC1")
		self.depth_kinect3_ready=1
		self.depth3[np.isnan(self.depth3)] = np.median(self.depth3)

		if self.verbose and self.display==3 and self.depth_roi_mask_image3_ready==0:
			cv_image_mask = np.array(self.depth3, dtype=np.float32)
			cv2.normalize(cv_image_mask, cv_image_mask, 0, 1, cv2.NORM_MINMAX)
			small = cv2.resize(cv_image_mask, (0,0), fx=self.scale, fy=self.scale)                     
			### display depth_roi via roi on where_msgs
			k3 = self.kinect3_dat
			for n in range(len(k3.roi)):
				x_ltop = k3.roi[n].x_offset 
				y_ltop = k3.roi[n].y_offset
				x_rbtn = k3.roi[n].x_offset + k3.roi[n].width 
				y_rbtn = k3.roi[n].y_offset + k3.roi[n].height
				if k3.roi[n].width < self.szW and k3.roi[n].height <self.szH:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 1)
				else:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 3)
			cv2.imshow('Depth_ROI 3: ', small)
			cv2.waitKey(3)

	def depth_callback4(self,data):
		self.depth4 = self.bridge.imgmsg_to_cv2(data, "16UC1")
		self.depth_kinect4_ready=1
		self.depth4[np.isnan(self.depth4)] = np.median(self.depth4)

		if self.verbose and self.display==4 and self.depth_roi_mask_image4_ready==0:
			cv_image_mask = np.array(self.depth4, dtype=np.float32)
			cv2.normalize(cv_image_mask, cv_image_mask, 0, 1, cv2.NORM_MINMAX)
			small = cv2.resize(cv_image_mask, (0,0), fx=self.scale, fy=self.scale)                     
			### display depth_roi via roi on where_msgs
			k4 = self.kinect4_dat
			for n in range(len(k4.roi)):
				x_ltop = k4.roi[n].x_offset 
				y_ltop = k4.roi[n].y_offset
				x_rbtn = k4.roi[n].x_offset + k4.roi[n].width 
				y_rbtn = k4.roi[n].y_offset + k4.roi[n].height
				if k4.roi[n].width < self.szW and k4.roi[n].height <self.szH:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 1)
				else:
					cv2.rectangle(small,(x_ltop, y_ltop), (x_rbtn, y_rbtn), (1,1,1), 3)
			cv2.imshow('Depth_ROI 4: ', small)
			cv2.waitKey(3)
		

	### subscribe ROI location           
	def where_callback1(self, data):
		### check the number of people detected
		if len(data.cam_id):
			if data.cam_id[0].data == 'sn_kinect_1':
				self.kinect1_dat = data
				self.kinect1_timer = rospy.get_time()
				#print '1'
				
	def where_callback2(self, data):
		if len(data.cam_id):
			if data.cam_id[0].data == 'sn_kinect_2':
				self.kinect2_dat = data
				self.kinect2_timer = rospy.get_time()
				#print ' 2'
				
	def where_callback3(self, data):
		if len(data.cam_id):
			if data.cam_id[0].data == 'sn_kinect_3':
				self.kinect3_dat = data
				self.kinect3_timer = rospy.get_time()
				#print '  3'
				
	def where_callback4(self, data):
		if len(data.cam_id):
			if data.cam_id[0].data == 'sn_kinect_4':
				self.kinect4_dat = data
				self.kinect4_timer = rospy.get_time()
				#print '   4'
				
	### publish ROI with location for ROI without location
	def timer_callback(self, event):
		''' to remove after-image'''
		if int(time()) - self.kinect1_timer >0.1:
			self.kinect1_dat = where_msgs()
		if int(time()) - self.kinect2_timer >0.1:
			self.kinect2_dat = where_msgs()
		if int(time()) - self.kinect3_timer >0.1:
			self.kinect3_dat = where_msgs()
		if int(time()) - self.kinect4_timer >0.1:
			self.kinect4_dat = where_msgs()

		if self.depth_kinect1_ready ==1:
			#print '\n========== 1 ==================' 
			self.gather_hd_roi(self.depth1, self.kinect1_dat, self.psn1[3], 1)
		if self.depth_kinect2_ready ==1:
			#print '\n========== 2 ==================' 
			self.gather_hd_roi(self.depth2, self.kinect2_dat, self.psn2[3], 2)
		if self.depth_kinect3_ready ==1:
			#print '\n========== 3 ==================' 
			self.gather_hd_roi(self.depth3, self.kinect3_dat, self.psn3[3], 3)
		if self.depth_kinect4_ready ==1:
			#print '\n========== 4 ==================' 
			self.gather_hd_roi(self.depth4, self.kinect4_dat, self.psn4[3], 4)



	### convert ROI depth to location
	def gather_hd_roi(self, data1, data2, data3, psn):
		fusion_mutex.acquire()
		data_depth = data1
		data_depth[np.isnan(data_depth)] = np.median(data_depth)
		data_roi = data2

		rows=480
		cols=640

		szWH= self.szWH
		szW = self.szW
		szH = self.szH
		szTOP=self.szTOP

		
		### update where_msgs info in new variable
		msg_detect = where_msgs()
		msg_detect_z0 = where_msgs()
		msg_detect_z0.header= data_roi.header
		msg_detect_z0.total = 0

		if data_roi.total>0:
			msg_detect.header=data_roi.header
			msg_detect.total=data_roi.total
			msg_detect.cam_id=data_roi.cam_id
			msg_detect.roi=data_roi.roi
			msg_detect.location=[]
			msg_detect.user_id=[]

		cnt=0
#		print msg_detect.roi, '\n\n', data_roi.location

		if len(msg_detect.roi)>0:
			### extract elements of roi(x,y,w,h) in msg_detect.roi
			for roi in msg_detect.roi:
				crop= data_depth[roi.y_offset+10:roi.y_offset+roi.height-10, roi.x_offset+10:roi.x_offset+roi.width-10]
				x = crop.reshape(-1) # 1-dimension

				### filter out data from boundary condition
				ind = np.nonzero(x<400)
				x2 = np.delete(x, ind)
				ind = np.nonzero(x2>4200)
				x3 = np.delete(x2, ind)
				if len(x3) >0:
					self.depth_crop = x3
				else:
					x3=self.depth_crop

				### remove noises by adopting median value
				ret = cv2.medianBlur(x3, 3)
				dist = np.median(ret)

				if math.isnan(dist):
					x3 =self.depth_crop
					ret = cv2.medianBlur(x3, 3)
					dist = np.median(ret)
				else:
					self.depth_crop = x3
					
				'''
				ahist, bin_edge = np.histogram(x3,100)
				ahist =scipy.signal.medfilt(ahist,kernel_size=3)
				ii=np.argmax(ahist)
				dist=bin_edge[ii]
				'''
				
				### estimate the head location for z=0
				HFOV=62.0 #62.0 # 57.0i
				VFOV=48.6 #48.6 # 43.2
				fx = cols / (2.*np.tan(HFOV/2*np.pi/180))
				xw = (cols/2 - (roi.x_offset+roi.width/2)) *dist / fx

				fy = rows / (2.*np.tan(VFOV/2*np.pi/180))
				ycom =-(rows/2 - (roi.y_offset+roi.height/2))*dist / fy
				yw=-(rows/2 - (roi.y_offset))*dist / fy

				zw = dist - (yw - ycom)*sin(data3) 

				OFFSET = 0.1	# 0.05m
				head_x = round(xw/1000+ OFFSET, 2)
				head_y = round(-yw/1000, 2)
				head_z = round(zw/1000- 2*OFFSET, 2)

				#print data_roi.cam_id[cnt].data, ': Mea x({0:.2f}) y({1:.2f}) z({2:.2f})'.format(data_roi.location[cnt].x, data_roi.location[cnt].y, data_roi.location[cnt].z)
				#print data_roi.cam_id[cnt].data, ': Est x({0:.2f}) y({1:.2f}) z({2:.2f})\n'.format(head_x, head_y, head_z)

				if False:#data_roi.location[cnt].z==0 and not math.isnan(float(head_z)):
					loc = Point32(head_x, head_y, head_z)
					msg_detect.user_id.append(-1)
					msg_detect.location.append(loc)
					#msg_detect_z0.location.append(loc)
					#msg_detect_z0.total = msg_detect_z0.total+1
				else:
					loc = Point32(data_roi.location[cnt].x, data_roi.location[cnt].y, data_roi.location[cnt].z)
					msg_detect.user_id.append(0)
					msg_detect.location.append(loc)                
						
				cnt= cnt+1


		'''
		# data association ?
#		msg_track = where_msgs()
#		msg_track.header=msg_detect.header
#		msg_track.total=msg_detect.total
#		msg_track.cam_id=msg_detect.cam_id
#		msg_track.roi=msg_detect.roi
#		msg_track.location=msg_detect.location
#		msg_track.user_id=msg_detect.user_id
#		pscale = 160

		aa = np.zeros((msg_detect.total, 2))
		if psn == 1:
			for ii, loc in enumerate(msg_detect.location):
				if not np.isnan(loc.x):
					aa[ii] = [loc.x, loc.z]
					#print ii,': ', aa

			aa = np.transpose(aa)
#			print '\nNo.LOC({}):\n{}\n---------'.format(aa.shape[1], aa)
		
			plt.clf()	# clear current axis
			#verts =[(0,0), (27,0)]
			#codes =[Path.MOVETO, Path.LINETO]
			#path = Path(verts, codes)
			#patch = patches.PathPatch(path, facecolor='green', lw=2)
			#ax.add_patch(patch)

			'' ' create list(4,the number of aa columns=aa.shape[1])'' '
			bb=np.zeros((4, aa.shape[1]))
			bb[0]=aa[0]	# loc.x
			bb[1]=aa[1] # loc.z

			self.f_gmphd.run_iteration(bb)

			gms = self.f_gmphd.gm
			index=0
			for gm in gms:
				# http://stackoverflow.com/questions/9215658/plot-a-circle-with-pyplot
				#print 'ID({0}): Loc({1:.2f}, {2:.2f}), W({3:.2f}), Detect: {4}'.format(gm.id, gm.mean[0,0], gm.mean[1,0], gm.weight, gm.detect)
				#print 'cov', gm.cov,'\n'
				#print 'ID({0})- cov: {1:.2f}, {2:.2f}, {3:.2f}, {4:.2f}'.format(gm.id, gm.cov[0][0], gm.cov[0][1], gm.cov[0][2], gm.cov[0][3])

				#if gm.detect and len(msg_detect.location)>0  :# > 0.1:
				if gm.detect:
					plt.plot(gm.mean[0,0],4-gm.mean[1,0], 'ro')
					if isinstance(gm.mean[0,0], float) == True and isinstance(gm.mean[1,0], float)== True:
						mintrack=[]
						for n in range(len(msg_detect.location)):
							mintrack.append(sqrt((msg_detect.location[n].x - gm.mean[0,0])**2 + (msg_detect.location[n].z - gm.mean[1,0])**2))
							#print '{0} MD({1:.2f},{2:.2f})\tDET({3:.2f},{4:.2f})\t= {5}'.format(n, gm.mean[0,0], gm.mean[1,0], msg_detect.location[n].x, msg_detect.location[n].z, mintrack)
						if len(mintrack)>0:
							#print min(mintrack), mintrack.index(min(mintrack)), msg_detect.location[mintrack.index(min(mintrack))]
							if min(mintrack)<0.5 and msg_detect.location[mintrack.index(min(mintrack))].y==0.5:
								msg_detect.location[mintrack.index(min(mintrack))].x=(gm.mean[0,0])
								msg_detect.location[mintrack.index(min(mintrack))].z=(gm.mean[1,0])

			N=50
			area =np.pi*(15*np.random.rand(N))**2 # 0 to 5 point radiuses
			#plt.scatter(aa[0], 4-aa[1], s=area, alpha=0.5)
			plt.plot(aa[0], 4-aa[1], 'b+')
			plt.axis([-3,3,0,4])
			plt.draw()
			#plt.pause(0.03)
		'''

		### publish human location
		msg_hd = where_msgs()
		msg_hd.header = msg_detect.header
		msg_hd.total = 0

		if msg_detect.total>0:
			for n in range(msg_detect.total):
				### filter out human data detected with boundary condition
				if msg_detect.roi[n].width*msg_detect.roi[n].height>szWH and msg_detect.roi[n].width>szW and msg_detect.roi[n].height>szH :
					if msg_detect.location[n].y < szTOP :
						msg_hd.total = msg_hd.total+1
						msg_hd.cam_id.append(msg_detect.cam_id[n])
						msg_hd.roi.append(msg_detect.roi[n])
						msg_hd.location.append(msg_detect.location[n])
						msg_hd.user_id.append(msg_detect.user_id[n])
						#print n, ':({0:.2f},{1:.2f},{2:.2f})-({3:d},{4:d})'.format(msg_detect.location[n].x, msg_detect.location[n].y, msg_detect.location[n].z,msg_detect.roi[n].x_offset, msg_detect.roi[n].y_offset)
					else:
						print 'detect the object over {0:.1f}m'.format(szTOP)
				#else:
					#print 'detect the object: W{0:.1f}, H{1:.1f} over (X{0:.1f}, Y{1:.1f}, Z{2:.1f})'.format(msg_detect.roi[n].width, msg_detect.roi[n].height, msg_detect.location[n].x, msg_detect.location[n].y, msg_detect.location[n].z)

		if msg_hd.total>0:
			self.pub_detection.publish(msg_hd)

		fusion_mutex.release()
		
def main(args):
	ic = HumanDetector_RGBD()    
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
