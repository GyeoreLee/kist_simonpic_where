#!/usr/bin/env python

# This module is to conduct data association with the human_roi detected from four where modules
# Create: 2016-09-12 by Sang-Seok Yun (yssmecha@gmail.com)
# Modified : 2017-04-17 by Gyeo-Re Lee (lkrrufp@kist.re.kr)

""""This is where fusion module."""
import sys, os, time
import rospy
import rospkg
from math import *
import math
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Point
from pub_msgs.msg import where_msgs
from nav_msgs.msg import Odometry
from simonpic_msgs.msg import Person, MultiPersons

import sys, cv2
from scipy.spatial import distance
from threading import Lock
import PyKDL
import numpy as np
import copy

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import scipy.io as sio

#import kr_tracker
import imp


# import gmphdROI as gmphdroi

human_mutex = Lock()
human_mutex_rcnn = Lock()
fusion_mutex = Lock()

publish_score_TH = 30.
vel_max = 0.


def CoordinateTransform(loc, psn_pose):
    rot = PyKDL.Rotation.EulerZYX(psn_pose[5], -psn_pose[4], psn_pose[3])

    vec = PyKDL.Vector(loc.x, loc.y, loc.z)

    vec_global = PyKDL.Vector(psn_pose[0], psn_pose[1], psn_pose[2])
    aa = rot * vec + vec_global

    return Point32(aa[0], aa[1], aa[2])


def DistanceXZ(p1, p2):
    # #print '=====>\np1:{0:.2f},{1:.2f}, p2:{2:.2f},{3:.2f}\n\n'.format(p1.x, p1.z, p2.x, p2.z)
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.z - p2.z) ** 2)


def TF_Loc_2_Global(data, psn_param):
    mpersons = MultiPersons()
    mpersons.header = data.header
    # #print '--loc', data.location, '--psn', psn_param
    szBody = 0.2  # size of Body
    for n in range(len(data.location)):
        tmp = Person()
        tmp.location = CoordinateTransform(data.location[n], psn_param)
        tmp.roi = data.roi[n]
        tmp.cam_id = data.cam_id[n]
        if tmp.cam_id.data == "sn_kinect1" or tmp.cam_id.data == "sn_kinect2":
            tmp.location.z = tmp.location.z  # + szBody / 2.0
        else:
            tmp.location.z = tmp.location.z  # - szBody / 2.0
        mpersons.persons.append(tmp)
    # #print '\n\n--loc2', mpersons.persons
    return mpersons


def TF_Loc_2_Global2(data, psn_param):
    mpersons = MultiPersons()
    mpersons.header = data.header
    # #print '--loc', data.location
    szBody = 0.2  # size of Body
    for n in range(len(data.location)):
        tmp = Person()
        tmp.location = CoordinateTransform(data.location[n], psn_param)
        tmp.roi = data.roi[n]
        tmp.cam_id = data.cam_id[n]
        if tmp.cam_id.data == "sn_kinect1" or tmp.cam_id.data == "sn_kinect2":
            tmp.location.z = tmp.location.z + szBody / 2.0
        else:
            tmp.location.z = tmp.location.z - szBody / 2.0
        mpersons.persons.append(tmp)
    # #print '--loc2', mpersons.persons
    return mpersons


class WhereFusion:
    def __init__(self):
        rospy.init_node('sp_hd_fs', anonymous=False)
        self.init_variable()

        # kr_tracker init
        self.object_filter = rospy.get_param('/FP_filter')
	kr_tracker = imp.load_compiled("my_module","kr_tracker.pyc")

        if self.object_filter == False:
            self.tracking = kr_tracker.kr_tracker(30.)
            global publish_score_TH
            publish_score_TH = 40.
        else:
            self.tracking = kr_tracker.kr_tracker(20.)
            global publish_score_TH
            publish_score_TH = 20.

        # set_seq_num for debug
        self.seq = 0

        # for corrected location display
        self.corrected_location = list()

        '''### init node name as sp_hl_hd_fs, disabled to respawn with same name'''
        ### inbound: /sn_kinect/detector from psn_units, outbound: /three_w/where
        if self.object_filter == False:
            rospy.Subscriber("/sn_kinect/detector", where_msgs, self.human_detector_callback, queue_size=1)
        else:
            rospy.Subscriber("/sn_kinect/detector_new1", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector_new2", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector_new3", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector_new4", where_msgs, self.human_detector_callback, queue_size=1)
        # rospy.Subscriber("/sn_kinect/rcnn_detector", where_msgs, self.human_detector_rcnn_callback)
        # rospy.Subscriber("/odom", Odometry, self.robot_remove_callback)

        ### for GUI display
        self.pub_raw = rospy.Publisher("/tone_w/where_raw", MultiPersons, queue_size=1)
        self.pub_rcnn_raw = rospy.Publisher("/tone_w/where_raw_rcnn", MultiPersons, queue_size=1)
        self.pub = rospy.Publisher("/tone_w/where", MultiPersons, queue_size=1)


        plt.ion()

        '''### timer for concatenating elements from psn_units'''
        if self.object_filter == False:
            rospy.Timer(rospy.Duration(0.033), self.timer_callback)
        else:
            rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.logwarn("run sp_hd_fs node (WHERE fusion)...")

        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            # print("Shutting down")
            # self.init_variable()

    def init_variable(self):
        '''### read psn_unit parameter, [x meter,y meter,z meter,r radian,p radian,y radian]'''
        self.psn1 = rospy.get_param('/psn_unit1')
        self.psn2 = rospy.get_param('/psn_unit2')
        self.psn3 = rospy.get_param('/psn_unit3')
        self.psn4 = rospy.get_param('/psn_unit4')

        self.verbose = rospy.get_param('verbose', True)
        self.dist_max = rospy.get_param('dist_max', 0.6)
        self.dist_min = rospy.get_param('dist_min', 0.25)
        # print 'param: verbose {}, Dmax {}, Dmin {}'.format(self.verbose, self.dist_max, self.dist_min)

        rows = [1.6, 2.47, 3.8, 5.05, 6.42]
        seatLoc = [[-1.8, rows[0]], [-1.2, rows[0]], [1.3, rows[0]], [2.1, rows[0]],
                   [-2.3, rows[1] - 0.2], [-1.5, rows[1]], [-0.6, rows[1]], [1.0, rows[1]], [1.8, rows[1]],
                   [2.5, rows[1]],
                   [-2.62, rows[2] + 0.03], [-1.8, rows[2]], [-0.8, rows[2]], [0.2, rows[2]], [0.9, rows[2]],
                   [1.6, rows[2]], [2.4, rows[2]],
                   [-2.9, rows[3]], [-2.1, rows[3]], [-1.4, rows[3]], [-0.7, rows[3]], [0.7, rows[3]], [1.5, rows[3]],
                   [2.2, rows[3]],
                   [-2.3, rows[4]], [-1.5, rows[4]], [-0.8, rows[4]], [0.9, rows[4]], [1.6, rows[4]], [2.4, rows[4]]]
        self.seatLOC = rospy.get_param('seat_location', seatLoc)

        rospack = rospkg.RosPack()
        self.gui_img_path = rospack.get_path('pub_msgs') + '/../../../database/GUI/floor.jpg'

        self.header_timestamp = Header()
        self.kinect1_timer = rospy.get_time()
        self.kinect2_timer = rospy.get_time()
        self.kinect3_timer = rospy.get_time()
        self.kinect4_timer = rospy.get_time()
        self.kinect1_dat = MultiPersons()
        self.kinect2_dat = MultiPersons()
        self.kinect3_dat = MultiPersons()
        self.kinect4_dat = MultiPersons()
        self.outmsg = MultiPersons()
        self.hd_prev = MultiPersons()

        self.header_rcnn_timestamp = Header()
        self.kinect1_rcnn_timer = rospy.get_time()
        self.kinect2_rcnn_timer = rospy.get_time()
        self.kinect3_rcnn_timer = rospy.get_time()
        self.kinect4_rcnn_timer = rospy.get_time()
        self.kinect1_rcnn_dat = MultiPersons()
        self.kinect2_rcnn_dat = MultiPersons()
        self.kinect3_rcnn_dat = MultiPersons()
        self.kinect4_rcnn_dat = MultiPersons()

        self.robotPosition = float(0), float(0), float(0.3)

        # display mode setting for debug
        self.predic_disp_mode = 1
        self.print_status_mode = 0

    def robot_remove_callback(self, data):
        self.robotPosition = float(data.pose.pose.position.y), 0.0, float(data.pose.pose.position.x) - 0.3

    def human_detector_callback(self, data):
        # human_mutex.acquire()
        # #print '===data', data
        self.header_timestamp = data.header
        if len(data.cam_id):
            # #print 'camid', data.cam_id[n].data
            '''#convert to global location by matching 'cam_id' and '/psn_unit' values'''
            if data.cam_id[0].data == 'sn_kinect1':
                self.kinect1_dat = TF_Loc_2_Global(data, self.psn1)
                self.kinect1_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect2':
                self.kinect2_dat = TF_Loc_2_Global(data, self.psn2)
                self.kinect2_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect3':
                self.kinect3_dat = TF_Loc_2_Global(data, self.psn3)
                self.kinect3_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect4':
                self.kinect4_dat = TF_Loc_2_Global(data, self.psn4)
                self.kinect4_timer = rospy.get_time()


                # for Debug
                # self.timer_callback()

                # human_mutex.release()

    def human_detector_rcnn_callback(self, data):
        human_mutex_rcnn.acquire()

        self.header_rcnn_timestamp = data.header
        if len(data.cam_id):
            if data.cam_id[0].data == 'sn_kinect1':
                self.kinect1_rcnn_dat = TF_Loc_2_Global2(data, self.psn1)
                self.kinect1_rcnn_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect2':
                self.kinect2_rcnn_dat = TF_Loc_2_Global2(data, self.psn2)
                self.kinect2_rcnn_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect3':
                self.kinect3_rcnn_dat = TF_Loc_2_Global2(data, self.psn3)
                self.kinect3_rcnn_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect4':
                self.kinect4_rcnn_dat = TF_Loc_2_Global2(data, self.psn4)
                self.kinect4_rcnn_timer = rospy.get_time()
        human_mutex_rcnn.release()

    def timer_callback(self, event):
        T_du = 0.1
        szDA = self.tracking.T

        fusion_mutex.acquire()

        # seq
        self.seq += 1

        ###### remove data that is not subscribed during 3sec
        if rospy.get_time() - self.kinect1_timer > T_du:
            self.kinect1_dat = MultiPersons()
        if rospy.get_time() - self.kinect2_timer > T_du:
            self.kinect2_dat = MultiPersons()
        if rospy.get_time() - self.kinect3_timer > T_du:
            self.kinect3_dat = MultiPersons()
        if rospy.get_time() - self.kinect4_timer > T_du:
            self.kinect4_dat = MultiPersons()

        '''####### human tracking with rcnn_localization ###########'''
        # if rospy.get_time() - self.kinect1_rcnn_timer > T_du:
        #    self.kinect1_rcnn_dat = MultiPersons()
        # if rospy.get_time() - self.kinect2_rcnn_timer > T_du:
        #    self.kinect2_rcnn_dat = MultiPersons()
        # if rospy.get_time() - self.kinect3_rcnn_timer > T_du:
        #    self.kinect3_rcnn_dat = MultiPersons()
        # if rospy.get_time() - self.kinect4_rcnn_timer > T_du:
        #    self.kinect4_rcnn_dat = MultiPersons()

        self.outmsg = MultiPersons()

        ### append human_roi in each kinect
        allP = MultiPersons()
        allP.persons.extend(self.kinect1_dat.persons)
        allP.persons.extend(self.kinect2_dat.persons)
        allP.persons.extend(self.kinect3_dat.persons)
        allP.persons.extend(self.kinect4_dat.persons)
        # #print '\n==================\n', allP.persons


        '''################## tracker part #############################'''
        # timestamp checking for initialize timespand
        if (len(str(self.header_timestamp.stamp.nsecs)) < 9):
            nsecs = '0' + str(self.header_timestamp.stamp.nsecs)
        else:
            nsecs = str(self.header_timestamp.stamp.nsecs)
        current_timestamp = str(self.header_timestamp.stamp.secs) + '.' + nsecs

        # self.tracking.set_param(current_timestamp)


        self.tracking.set_seq(self.seq)

        # keep going flag init(for prediction)
        for n in range(len(self.tracking.X)):
            self.tracking.X[n][3] = False

        # pruning current tracking_models
        self.tracking.obj_pruning(self.tracking.X, self.tracking.P)

        # when observation is exist
        self.corrected_location = []
        if len(allP.persons):

            # Display
            # self.ShowFusionXZ(szDA)


            print '------------------------------------------------------------------'
            print 'seq : ', self.seq

            # keep going flag init(for prediction)
            for n in range(len(self.tracking.X)):
                self.tracking.X[n][3] = False

            # pruning current tracking_models
            # self.tracking.obj_pruning(self.tracking.X, self.tracking.P)


            # for selecting main_cam_id & roi for output msg
            for tracking_model in self.tracking.X:
                if len(tracking_model) > 5:
                    tracking_model.pop()  # 5 ROI
                    tracking_model.pop()  # 6 meas_loc_y
                    tracking_model.pop()  # 7 meas_cam_id
                    tracking_model.pop()  # 8 distance of measurement-cam_location

            # multi_camera input to kr_tracker indivisually.
            for i in range(4):

                # measure formatting
                measurements = []

                # get measurements from detected input!
                ii = 0
                for person_elements in allP.persons:
                    if person_elements.cam_id.data == 'sn_kinect' + str(i + 1):

                        if person_elements.roi.do_rectify == False and self.object_filter == True:
                            continue

                        # seq, id, x, z, roi_info, y, cam_id(str)


                        measurements.append([0,
                                             ii,
                                             person_elements.location.x,
                                             person_elements.location.z,
                                             person_elements.roi,
                                             person_elements.location.y,
                                             person_elements.cam_id.data
                                             ])
                        # measurements.append([where_msgs.header.seq, where_msgs.user_id[i], pos_x, pos_z,])

                    ii += 1

                if measurements:
                    print 'sn_kinect' + str(i + 1),

                    # association
                    assign_index, self.tracking.X, self.tracking.P, miss_num = self.tracking.association(measurements,
                                                                                                         self.tracking.X,
                                                                                                         self.tracking.P)

                    # Kalman filter action (no_prediction)
                    for n in range(len(assign_index)):

                        meas_index = assign_index[n][0]  # meas_index
                        obj_index = assign_index[n][1]  # obj_index

                        # measurement elements to Kalman_matrix(x, z)
                        Z = np.matrix([measurements[meas_index][2],
                                       measurements[meas_index][3],
                                       ]).T  # n is object id

                        # Kalman update
                        self.tracking.X[obj_index][1], self.tracking.P[obj_index][
                            1] = self.tracking.Kalman_filter_update(
                            self.tracking.X[obj_index][1], self.tracking.P[obj_index][1], Z, self.tracking.B,
                            self.tracking.F, self.tracking.H, self.tracking.R, self.tracking.I, self.tracking.Q,
                            self.tracking.u)

                        # #print association result
                        # print 'measure[%d]=>obj[%d]' % (measurements[meas_index][1], self.tracking.X[obj_index][0])
                        score = self.tracking.X[obj_index][2]
                        # print 'score : ', score

                        # kalman update_checking
                        self.tracking.X[obj_index][3] = True

                        # measurement roi, location y to tracking_model (not through the Kalman filter)
                        meas_loc_x = measurements[meas_index][2]
                        meas_loc_z = measurements[meas_index][3]

                        meas_roi = measurements[meas_index][4]
                        meas_loc_y = measurements[meas_index][5]
                        meas_cam_id = measurements[meas_index][6]  # current camera_id

                        cam_location = rospy.get_param('/psn_unit' + str(i + 1))

                        meas_distance = np.sqrt(
                            (cam_location[0] - meas_loc_x) ** 2 + (cam_location[2] - meas_loc_z) ** 2)

                        # Enter the information of the nearest camera
                        try:
                            #                   distance of measurement-cam_location
                            if (meas_distance <= self.tracking.X[obj_index][8]):
                                self.tracking.X[obj_index][5] = meas_roi  # ROI
                                self.tracking.X[obj_index][6] = meas_loc_y
                                self.tracking.X[obj_index][7] = meas_cam_id  # meas_cam_id
                                self.tracking.X[obj_index][8] = meas_distance  # distance of measurement-cam_location
                            else:
                                if (meas_cam_id == self.tracking.X[obj_index][7]):
                                    self.tracking.X[obj_index][5] = meas_roi  # ROI
                                    self.tracking.X[obj_index][6] = meas_loc_y
                                    self.tracking.X[obj_index][7] = meas_cam_id  # meas_cam_id
                                    # self.tracking.X[obj_index][7] = meas_distance  # distance of measurement-cam_location


                        except:
                            self.tracking.X[obj_index].append(meas_roi)  # ROI
                            self.tracking.X[obj_index].append(meas_loc_y)
                            self.tracking.X[obj_index].append(meas_cam_id)  # meas_cam_id
                            self.tracking.X[obj_index].append(meas_distance)  # distance of measurement-cam_location

            # Corrected location store for Display & publish
            corrected_X = copy.deepcopy(self.tracking.X)
            for trackingData in corrected_X:
                if trackingData[2] >= publish_score_TH:
                    self.corrected_location.append(trackingData)

            # Kalman filter action (prediction) about 1 frame

            for n in range(len(self.tracking.X)):
                if self.tracking.X[n][3] == True:
                    # prediction
                    Z = np.matrix([]).T
                    self.tracking.X[n][1], self.tracking.P[n][1] = self.tracking.Kalman_filter(
                        self.tracking.X[n][1], self.tracking.P[n][1],
                        Z, self.tracking.B, self.tracking.F,
                        self.tracking.H, self.tracking.R, self.tracking.I,
                        self.tracking.Q, self.tracking.u)

                    # BnD Score calculation(from Detection)
                    self.tracking.X[n][2] = self.tracking.score(self.tracking.X[n][2], True)

        # when observation is not exist
        if self.tracking.X:
            for n in range(len(self.tracking.X)):
                if self.tracking.X[n][3] != True:
                    # print 'measurement : empty, track_obj : exist'

                    # print 'predict_timespan : ',self.tracking.timespan

                    # prediction
                    # Z = np.matrix([]).T
                    # self.tracking.X[n][1], self.tracking.P[n][1] = self.tracking.Kalman_filter(
                    #    self.tracking.X[n][1], self.tracking.P[n][1],
                    #    Z, self.tracking.B, self.tracking.F,
                    #    self.tracking.H, self.tracking.R, self.tracking.I,
                    #    self.tracking.Q, self.tracking.u)

                    # BnD Score calculation(from not detection)
                    self.tracking.X[n][2] = self.tracking.score(self.tracking.X[n][2], False)

                    obj_id = self.tracking.X[n][0]
                    meas_id = self.tracking.X[n][0]
                    score = self.tracking.X[n][2]

                    self.tracking.X[n][3] = False
                    # print ('obj_id : %d , meas_id : %d, score : %1.2f') % (obj_id, meas_id, score)

        # publish tracked msg
        self.outmsg.header = self.header_timestamp

        if self.corrected_location:
            self.outmsg.total = len(self.corrected_location)
            # self.outmsg.persons = []
            for tracked_element in self.corrected_location:
                tmp = Person()
                tmp.location.x = tracked_element[1].item(0)
                tmp.location.z = tracked_element[1].item(1)

                # tracking output boundary setting
                # if abs(tmp.location.x) > 1.5 or abs(tmp.location.z > 3.0):
                #    self.outmsg.total -= 1
                #    continue

                try:
                    tmp.location.y = tracked_element[6]
                    tmp.id.data = str(tracked_element[0])
                    tmp.roi = tracked_element[5]
                    tmp.cam_id.data = tracked_element[7]
                    # tmp.g_id.data = str(tracked_element[4])

                    # birth or exist flag implement
                    if ((time.time() - tracked_element[4]) < 2.5):
                        tmp.g_id.data = 'birth'
                        print tmp.id.data, 'birth\n'



                    else:
                        tmp.g_id.data = 'exist'
                    self.outmsg.persons.append(tmp)
                except:
                    self.outmsg.total -= 1

                    # self.outmsg.persons.append(tmp)
                    continue



                    # self.outmsg.persons.append(tmp)
        else:
            self.outmsg.total = 0
            # self.outmsg.persons = []

        # Display

        self.ShowFusionXZ(szDA)

        # Publish
        self.pub.publish(self.outmsg)

        fusion_mutex.release()

    def ShowFusionXZ(self, szDA):

        '''### floor.jpg with 800*600 pixels'''
        # cv_image = cv2.imread(self.gui_img_path)

        #                      Height Width
        white_image = np.zeros((550, 800, 3), np.uint8)
        white_image[:, :] = (255, 255, 255)  # (B, G, R)
        cv_image = white_image

        '''### Set plot param'''
        T_du2 = 0.3
        pscale = 100
        margin = 20
        text_offset = -9
        szH = 800 / 2
        # szV = 800
        szV = 500
        Fscale = 0.7  # font_size
        font = cv2.FONT_HERSHEY_SIMPLEX

        colorcode = [(255, 207, 0)
            , (68, 231, 98)
            , (58, 199, 255)
            , (182, 89, 255)]

        # colorcode = (255, 0, 0)
        # colorcode = (64, 64, 128)
        # colorcode = (255, 255, 0)
        # colorcode = (0, 128, 0)

        '''### plot grid on the image with origin'''
        for j in range(1, 8):
            for i in range(1, 8):
                # horizontal line
                cv2.line(cv_image, (0, i * pscale + margin), (szH * 2, i * pscale + margin), (200, 200, 200), 1)

                # vertical line
                cv2.line(cv_image, (j * pscale, 0), (j * pscale, szV), (200, 200, 200), 1)

                # Z meter text
                cv2.putText(cv_image, '{:d}m'.format(j), (10, j * pscale + margin - 5), font, Fscale, (20, 20, 25), 2)

            # X meter text
            cv2.putText(cv_image, '{:d}m'.format(j - 4), (j * pscale, szV + margin), font, Fscale, (20, 20, 25), 2)

        # origin plot
        cv2.putText(cv_image, '+', (szH + text_offset, margin + 6), font, Fscale, (0, 0, 255), 2)
        cv2.putText(cv_image, '(origin)', (szH - 40, 20), font, Fscale, (72, 72, 72), 2)

        ''' plot Kinect locations'''
        OffX1 = int(self.psn1[0] * 100)
        OffZ1 = int(self.psn1[2] * 100)

        OffX2 = int(self.psn2[0] * 100)
        OffZ2 = int(self.psn2[2] * 100)

        OffX3 = int(self.psn3[0] * 100)
        OffZ3 = int(self.psn3[2] * 100)

        OffX4 = int(self.psn4[0] * 100)
        OffZ4 = int(self.psn4[2] * 100)

        # Camera Position
        cv2.putText(cv_image, '+', (szH + OffX1 + text_offset, OffZ1 + margin), font, 0.5, (0, 0, 255), 2)  # kinect1
        cv2.putText(cv_image, '+', (szH + OffX2 + text_offset, OffZ2 + margin), font, 0.5, (0, 0, 255), 2)  # kinect2
        cv2.putText(cv_image, '+', (szH + OffX3 + text_offset, OffZ3 + margin), font, 0.6, (0, 0, 255), 2)  # kinect3
        cv2.putText(cv_image, '+', (szH + OffX4 + text_offset, OffZ4 + margin), font, 0.6, (0, 0, 255), 2)  # kinect4

        # each kinect camera text
        cv2.putText(cv_image, 'K1', (szH + OffX1 + 15, OffZ1 + margin + 12), font, Fscale, colorcode[0], 2)
        cv2.putText(cv_image, 'K2', (szH + OffX2 + 15, OffZ2 + margin + 12), font, Fscale, colorcode[1], 2)
        cv2.putText(cv_image, 'K3', (szH + OffX3 + 15, OffZ3 + margin + 12), font, Fscale, colorcode[2], 2)
        cv2.putText(cv_image, 'K4', (szH + OffX4 + 15, OffZ4 + margin + 12), font, Fscale, colorcode[3], 2)

        '''plot sensor range'''
        HFOV = 58.0
        DIST = 4.5 * pscale  # kinect can obtain 4.5m distance
        # Xin1 = (int)(DIST * np.sin(HFOV / 2 * np.pi / 180  + self.psn1[4]))

        # 1
        Xin1_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn1[4]))
        Zin1_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn1[4]))

        Xin1_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn1[4]))
        Zin1_left = (int)(DIST * np.cos(-HFOV / 2 * np.pi / 180 + self.psn1[4]))

        # 2
        Xin2_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn2[4]))
        Zin2_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn2[4]))

        Xin2_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn2[4]))
        Zin2_left = (int)(DIST * np.cos(-HFOV / 2 * np.pi / 180 + self.psn2[4]))

        # 3
        Xin3_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn3[4]))
        Zin3_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn3[4]))

        Xin3_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn3[4]))
        Zin3_left = (int)(DIST * np.cos(-HFOV / 2 * np.pi / 180 + self.psn3[4]))

        # 4        
        Xin4_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn4[4]))
        Zin4_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn4[4]))

        Xin4_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn4[4]))
        Zin4_left = (int)(DIST * np.cos(-HFOV / 2 * np.pi / 180 + self.psn4[4]))

        # kinect1 sensor range
        cv2.line(cv_image, (szH + OffX1, OffZ1 + margin), (szH + OffX1 + Xin1_left, OffZ1 + margin + Zin1_left),
                 colorcode[0], 1)  # left-line
        cv2.line(cv_image, (szH + OffX1, OffZ1 + margin), (szH + OffX1 + Xin1_right, OffZ1 + margin + Zin1_right),
                 colorcode[0], 1)  # right-line

        # kinect2 sensor range
        cv2.line(cv_image, (szH + OffX2, OffZ2 + margin), (szH + OffX2 + Xin2_left, OffZ2 + margin + Zin2_left),
                 colorcode[1], 1)  # left-line
        cv2.line(cv_image, (szH + OffX2, OffZ2 + margin), (szH + OffX2 + Xin2_right, OffZ2 + margin + Zin2_right),
                 colorcode[1], 1)  # right-line

        # kinect3 sensor range
        cv2.line(cv_image, (szH + OffX3, OffZ3 + margin), (szH + OffX3 + Xin3_left, OffZ3 + margin + Zin3_left),
                 colorcode[2], 1)  # left-line
        cv2.line(cv_image, (szH + OffX3, OffZ3 + margin), (szH + OffX3 + Xin3_right, OffZ3 + margin + Zin3_right),
                 colorcode[2], 1)  # right-line

        # kinect4 sensor range
        cv2.line(cv_image, (szH + OffX4, OffZ4 + margin), (szH + OffX4 + Xin4_left, OffZ4 + margin + Zin4_left),
                 colorcode[3], 1)  # left-line
        cv2.line(cv_image, (szH + OffX4, OffZ4 + margin), (szH + OffX4 + Xin4_right, OffZ4 + margin + Zin4_right),
                 colorcode[3], 1)  # right-line

        '''
        # 1
        Xin1_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn1[4]))
        Zin1_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn1[4]))
        Xin1_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn1[4]))
        Zin1_left = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn1[4]))
        # 2
        Xin2_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn2[4]))
        Zin2_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn2[4]))
        Xin2_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn2[4]))
        Zin2_left = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn2[4]))
        # 3
        Xin3_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn3[4]))
        Zin3_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn3[4]))
        Xin3_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn3[4]))
        Zin3_left = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn3[4]))
        # 4
        Xin4_right = -(int)(DIST * np.sin(HFOV / 2 * np.pi / 180 + self.psn4[4]))
        Zin4_right = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn4[4]))
        Xin4_left = -(int)(DIST * np.sin(-HFOV / 2 * np.pi / 180 + self.psn4[4]))
        Zin4_left = (int)(DIST * np.cos(HFOV / 2 * np.pi / 180 + self.psn4[4]))
        # kinect1 sensor range
        cv2.line(cv_image, (szH + OffX1, OffZ1 + margin), (szH + OffX1 + Xin1_left, OffZ1 + margin + Zin1_left),
                 colorcode[0], 1)  # left-line
        cv2.line(cv_image, (szH + OffX1, OffZ1 + margin), (szH + OffX1 + Xin1_right, OffZ1 + margin + Zin1_right),
                 colorcode[0], 1)  # right-line
        # kinect2 sensor range
        cv2.line(cv_image, (szH + OffX2, OffZ2 + margin), (szH + OffX2 + Xin2_left, OffZ2 + margin + Zin2_left),
                 colorcode[1], 1)  # left-line
        cv2.line(cv_image, (szH + OffX2, OffZ2 + margin), (szH + OffX2 + Xin2_right, OffZ2 + margin + Zin2_right),
                 colorcode[1], 1)  # right-line
        # kinect3 sensor range
        # cv2.line(cv_image, (szH + OffX3, OffZ3 + margin), (szH + OffX3 + Xin3_left,  OffZ3 + margin+ Zin3), colorcode[2], 1) #left-line
        # cv2.line(cv_image, (szH + OffX3, OffZ3 + margin), (szH + OffX3 - Xin3_right, OffZ3 + margin+ Zin3), colorcode[2], 1) #right-line
        # kinect4 sensor range
        cv2.line(cv_image, (szH + OffX4, OffZ4 + margin), (szH + OffX4 + Xin4_left, OffZ4 + margin + Zin4_left),
                 colorcode[3], 1)  # left-line
        cv2.line(cv_image, (szH + OffX4, OffZ4 + margin), (szH + OffX4 + Xin4_right, OffZ4 + margin + Zin4_right),
                 colorcode[3], 1)  # right-line
        '''

        # plot seat Position
        '''
        szSeat = 30
        num = 1
        for x, z in self.seatLOC:
            cv2.rectangle(cv_image, ((int)(x * pscale + szH - szSeat), (int)(z * pscale - szSeat)),
                          ((int)(x * pscale + szH + szSeat), (int)(z * pscale + szSeat)), (0, 255, 0), 2)
            cv2.putText(cv_image, '{:2d}'.format(num), ((int)(x * pscale + szH +10), (int)(z * pscale - 35)), font,
                        Fscale, (0, 0, 0), 2)
            cv2.putText(cv_image, '{:2d}'.format(num), ((int)(x * pscale + szH +10), (int)(z * pscale - 35)), font,
                        Fscale, (150, 150, 250), 1)
            num = num + 1
        '''

        # plot localized student seats
        seated = 0
        Seat = np.zeros(30)
        for ii, pp in enumerate(self.outmsg.persons):
            loc = pp.location
            try:
                tracked_id = int(pp.id.data)
            except:
                tracked_id = 0
            # #print 'prt: ', pp.cam_id.data, ' {0:.1f}, {1:.1f}'.format(loc.x, loc.z)
            #            num =1


            # Plot seat occupancy with its number
            '''
            num = 1
            for x, z in self.seatLOC:
                if fabs(x - loc.x) < 1.0 * szSeat / pscale and fabs(z - loc.z) < 1.0 * szSeat / pscale:
                    # #print '-----> prt: ', pp.cam_id.data, ' {0:.1f}, {1:.1f}'.format(loc.x, loc.z)
                    seated = seated + 1
                    cv2.rectangle(cv_image, ((int)(x * pscale + szH - szSeat), (int)(z * pscale - szSeat)),
                                  ((int)(x * pscale + szH + szSeat), (int)(z * pscale + szSeat)), (232, 128, 255), -1)
                    cv2.putText(cv_image, '{:2d}'.format(num), ((int)(x * pscale + szH + 10), (int)(z * pscale - 35)),
                                font, Fscale, (0, 0, 0), 3)
                    cv2.putText(cv_image, '{:2d}'.format(num), ((int)(x * pscale + szH + 10), (int)(z * pscale - 35)),
                                font, Fscale, (220, 220, 220), 2)
                    break
                num = num + 1
            '''

        #########################plot tracked data##############################
        '''
        for tracked_data in self.corrected_location:
            tracked_location =  tracked_data[1]
            loc_x = tracked_location.item(0)
            loc_z = tracked_location.item(1)
            tracked_id = tracked_data[0]
            # plot the circle of data association
            cv2.circle(cv_image, ((int)(szH + loc_x * pscale), (int)(loc_z * pscale)), int(pscale * szDA), (0, 0, 255),
                       1)
            cv2.circle(cv_image, ((int)(szH + loc_x * pscale), (int)(loc_z * pscale)), 3, (0, 0, 0),
                       5)
            cv2.putText(cv_image, '{:d}'.format(tracked_id), ((int)(szH + loc_x * pscale-30), (int)(loc_z * pscale)),
                        font, 0.7, (0, 0, 0), 2)
        '''

        # Display tracked data when output is generated

        for tracked_data in self.outmsg.persons:
            tracked_location = tracked_data.location
            loc_x = tracked_location.x
            loc_z = tracked_location.z
            tracked_id = tracked_data.id.data
            birth_flag = tracked_data.g_id.data

            # plot the circle of data association boundary
            cv2.circle(cv_image, ((int)(szH + loc_x * pscale), (int)(loc_z * pscale) + margin), int(pscale * szDA),
                       (192, 192, 192),
                       1)
            if birth_flag == 'birth':

                cv2.circle(cv_image, ((int)(szH + loc_x * pscale), (int)(loc_z * pscale) + margin), 3, (0, 0, 255), 5)
            else:

                cv2.circle(cv_image, ((int)(szH + loc_x * pscale), (int)(loc_z * pscale) + margin), 3, (0, 0, 0), 5)

            cv2.putText(cv_image, tracked_id,
                        ((int)(szH + loc_x * pscale - 30), (int)(loc_z * pscale) + margin),
                        font, 0.7, (0, 0, 0), 2)

            # plot arrowline about prediction location
            global vel_max

            if (self.predic_disp_mode):
                for prediction_data in self.tracking.X:
                    p_tracked_id = prediction_data[0]

                    if int(tracked_id) == p_tracked_id:
                        predicted_pos_x = prediction_data[1].item(0)
                        predicted_pos_z = prediction_data[1].item(1)
                        predicted_velocity = sqrt(prediction_data[1].item(2) ** 2 + prediction_data[1].item(3) ** 2)

                        # if (predicted_velocity > vel_max):
                        #    vel_max = predicted_velocity
                        # cv2.putText(cv_image, 'velocity: ' + str('%.2f' % predicted_velocity), (300, 50), font, 0.8, (255, 0, 255), 2)


                        diff_x = (predicted_pos_x - loc_x) * 10
                        diff_z = (predicted_pos_z - loc_z) * 10

                        # cv2.arrowedLine(cv_image,((int)(szH + loc_x * pscale), (int)(loc_z * pscale)),
                        #                ((int)(szH + (loc_x +diff_x)  * pscale), (int)((loc_z+diff_z) * pscale)),
                        #                (0,255,255),3
                        #                )

                        # prediction point display
                        # cv2.circle(cv_image, ((int)(szH + predicted_pos_x * pscale), (int)(predicted_pos_z * pscale)), 3, (0, 255, 255), 5)
                        cv2.circle(cv_image,
                                   ((int)(szH + predicted_pos_x * pscale), (int)(predicted_pos_z * pscale + margin)),
                                   int(pscale * (szDA + log10(predicted_velocity + 1))),
                                   (0, 255, 255),
                                   1)
                        break

        # plot human locations from OpenNI tracker
        # if 1 :
        if rospy.get_time() - self.kinect1_timer < T_du2 or rospy.get_time() - self.kinect2_timer < T_du2:
            allP = MultiPersons()
            allP.persons.extend(self.kinect1_dat.persons)
            allP.persons.extend(self.kinect2_dat.persons)
            allP.persons.extend(self.kinect3_dat.persons)
            allP.persons.extend(self.kinect4_dat.persons)

            '''============ Data Association========================='''

            #             for ii, pp in enumerate(self.kinect1_dat.persons):
            for ii, pp in enumerate(allP.persons):
                loc = pp.location

                # Non-human Filtering
                if pp.roi.do_rectify == False and self.object_filter == True:
                    continue

                color_idx = 0

                if pp.cam_id.data == 'sn_kinect1':
                    color_idx = 0
                    # colorcode = (255, 207, 0)
                    # colorcode = (255, 0, 0)
                elif pp.cam_id.data == 'sn_kinect2':
                    color_idx = 1
                    # colorcode = (68, 231, 98)
                    # colorcode = (64, 64, 128)
                elif pp.cam_id.data == 'sn_kinect3':
                    color_idx = 2
                    # colorcode = (58, 199, 255)
                    # colorcode = (255, 255, 0)
                else:
                    color_idx = 3
                    # colorcode = (182, 89, 255)
                    # colorcode = (0, 128, 0)

                # #print ii, pp.cam_id

                cv2.circle(cv_image, ((int)(szH + loc.x * pscale), (int)(loc.z * pscale + margin)), 1,
                           colorcode[color_idx], 2)
                # cv2.putText(cv_image, '{:.2f}: {:.2f}'.format(loc.x, loc.z),
                #            ((int)(szH + loc.x * pscale + 6), (int)((loc.z * pscale) + 4)), font, 0.4, (255, 0, 0), 1)

                # Detection id display
                # cv2.putText(cv_image, '{:d}'.format(ii),
                #            ((int)(szH + loc.x * pscale - 10), (int)((loc.z * pscale) + 25)), font, 0.7, (255, 0, 125),
                #            2)

        ''' plot human locations from each kinect camera'''
        '''
        if rospy.get_time() - self.kinect1_rcnn_timer < T_du2 or rospy.get_time() - self.kinect2_rcnn_timer < T_du2:
            pscale = 100
            allP = MultiPersons()
            allP.persons.extend(self.kinect1_rcnn_dat.persons)
            allP.persons.extend(self.kinect2_rcnn_dat.persons)
            allP.persons.extend(self.kinect3_rcnn_dat.persons)
            allP.persons.extend(self.kinect4_rcnn_dat.persons)
            for ii, pp in enumerate(allP.persons):
                loc = pp.location
                if pp.cam_id.data == 'sn_kinect1':
                    colorcode = (255, 0, 0)
                elif pp.cam_id.data == 'sn_kinect2':
                    colorcode = (64, 64, 128)
                elif pp.cam_id.data == 'sn_kinect3':
                    colorcode = (255, 255, 0)
                else:
                    colorcode = (0, 128, 0)
                cv2.circle(cv_image, ((int)(szH + loc.x * pscale), (int)(loc.z * pscale)), 7, colorcode, 1)
                #cv2.putText(cv_image, '{:.2f}: {:.2f}'.format(loc.x, loc.z), ( (int)(szH + loc.x*pscale+6), (int)( (loc.z*pscale)+4) ), font , 0.4, (255, 0, 0), 1)
        '''

        cv2.putText(cv_image, 'Total: ' + str(self.outmsg.total), (30, 50), font, 0.8, (0, 0, 0), 3)
        cv2.putText(cv_image, 'Total: ' + str(self.outmsg.total), (30, 50), font, 0.8, (255, 0, 255), 2)
        cv2.putText(cv_image, 'seq: ' + str(self.seq), (150, 50), font, 0.8, (255, 0, 255), 2)
        # cv2.putText(cv_image, ', Occupied: ' + str(seated), (150, 50), font, 0.8, (0, 0, 0), 3)
        # cv2.putText(cv_image, ', Occupied: ' + str(seated), (150, 50), font, 0.8, (0, 0, 255), 2)
        #		small = cv2.resize(cv_image_mask, (0,0), fx=self.scale, fy=self.scale)
        cv2.imshow('Human Localization (sp_hd_fs): ', cv_image)
        cv2.waitKey(3)


if __name__ == '__main__':

    try:
        WhereFusion()
    except rospy.ROSInterruptException:
        pass
