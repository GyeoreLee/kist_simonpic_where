#!/usr/bin/env python

import sys, os, time
import rospy
import rospkg

from pub_msgs.msg import where_msgs
from simonpic_msgs.msg import Person, MultiPersons
from std_msgs.msg import Header
from std_msgs.msg import String

import numpy as np
import PyKDL
from threading import Lock
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn import mixture
import itertools
import time


human_mutex = Lock()
class WhereCalibration:

    # Calibration pair distance parameter
    CALIB_DIST = 1.0
    # Number of pairs for calibration
    SAMPLE_NUM = 2000

    def __init__(self):

        rospy.init_node('sp_hd_calibration', anonymous=False)

        # ROS Param for faster-RCNN-Detection input applying
        self.object_filter = rospy.get_param('/FP_filter')

        self.header_timestamp = Header()
        self.kinect1_timer = rospy.get_time()
        self.kinect2_timer = rospy.get_time()
        self.kinect3_timer = rospy.get_time()
        self.kinect4_timer = rospy.get_time()
        self.kinect1_dat = MultiPersons()
        self.kinect2_dat = MultiPersons()
        self.kinect3_dat = MultiPersons()
        self.kinect4_dat = MultiPersons()

        self.person_trajectory = []

        self.algorithm_working_time_start = []
        self.algorithm_working_time_end = []
        self.once_flag = False

        #parameter recording table for validation and error complementation.
        # row is std cam_id, column is target cam_id
        self.parameter_table = [[[],[],[],[]],
                                [[],[],[],[]],
                                [[],[],[],[]],
                                [[],[],[],[]]]

        self.global_std_param = rospy.get_param('/psn_unit' + str(1))
        self.global_parameter_table = [[self.global_std_param],[],[],[]]


        # cam1 need to set always True because cam1's position is fixed.
        self.calibrated_cam_table = [True, False, True, True]

        if self.object_filter == False:
            rospy.Subscriber("/sn_kinect/detector1", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector2", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector3", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector4", where_msgs, self.human_detector_callback, queue_size=1)
        else:
            # faster-RCNN-Detection Topic
            rospy.Subscriber("/sn_kinect/detector_new1", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector_new2", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector_new3", where_msgs, self.human_detector_callback, queue_size=1)
            rospy.Subscriber("/sn_kinect/detector_new4", where_msgs, self.human_detector_callback, queue_size=1)


        # ros topic for GUI Display
        self.calib_status_pub = rospy.Publisher("/camera_calibration_status", String, queue_size=1)
        self.calib_message_pub = rospy.Publisher("/camera_calibration_message", String, queue_size=1)

        rospy.Timer(rospy.Duration(0.033), self.calibration_callback)

        while not rospy.is_shutdown():
            rospy.sleep(0.01)

    def data_parsing(self, data):
        mpersons = MultiPersons()
        mpersons.header = data.header
        # #print '--loc', data.location, '--psn', psn_param
        szBody = 0.2  # size of Body
        for n in range(len(data.location)):
            tmp = Person()
            tmp.location = data.location[n]
            tmp.roi = data.roi[n]
            tmp.cam_id = data.cam_id[n]
            if tmp.cam_id.data == "sn_kinect1" or tmp.cam_id.data == "sn_kinect2":
                tmp.location.z = tmp.location.z + szBody / 2.0
            else:
                tmp.location.z = tmp.location.z - szBody / 2.0
            mpersons.persons.append(tmp)
        # #print '\n\n--loc2', mpersons.persons
        return mpersons

    def human_detector_callback(self, data):
        human_mutex.acquire()
        # #print '===data', data
        self.header_timestamp = data.header

        if len(data.cam_id):
            # #print 'camid', data.cam_id[n].data
            '''#convert to global location by matching 'cam_id' and '/psn_unit' values'''

            if data.cam_id[0].data == 'sn_kinect1':
                self.kinect1_dat = self.data_parsing(data)
                self.kinect1_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect2':
                self.kinect2_dat = self.data_parsing(data)
                self.kinect2_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect3':
                self.kinect3_dat = self.data_parsing(data)
                self.kinect3_timer = rospy.get_time()

            if data.cam_id[0].data == 'sn_kinect4':
                self.kinect4_dat = self.data_parsing(data)
                self.kinect4_timer = rospy.get_time()


        # for Debug
        #self.timer_callback()

        human_mutex.release()

    def CoordinateTransform(self, input_location, std_cam_param):

        #                             Y                P                   R
        rot = PyKDL.Rotation.EulerZYX(std_cam_param[5], -std_cam_param[4], std_cam_param[3])

        #                  detection_X        detection_Y        detection_Z
        vec = PyKDL.Vector(input_location[0], input_location[1], input_location[2])

        #                         X                 Y                 Z
        vec_global = PyKDL.Vector(std_cam_param[0], std_cam_param[1], std_cam_param[2])

        transformed_location = rot * vec + vec_global

        return transformed_location

    def setRosParam(self,cam_id, std_cam_id ,x_bias, z_bias, pitch ):

        # load a previous target cam-position-RPY param
        self.psn_unit_param = rospy.get_param('/psn_unit'+str(cam_id))

        # load a standard cam-position-RPY param which was used for calibration.
        std_coordinate_param = rospy.get_param('/psn_unit' + str(std_cam_id))

        input_location = [x_bias, 0, z_bias] # X, Y, X
        std_cam_param = std_coordinate_param
        std_cam_param[3] = 0

        # Transforming new cam-position-RPY param to the
        transformed_location = self.CoordinateTransform(input_location, std_cam_param)

        # Rounding the param value
        self.psn_unit_param[0] = round(transformed_location[0],2) # new camera X_bias
        self.psn_unit_param[2] = round(transformed_location[2],2) # new camera Z_bias
        self.psn_unit_param[4] = round(pitch + std_cam_param[4] ,2)  # pitch

        print 'local2std x_bias %.2f z_bias %.2f pitch %.2f degree' % (x_bias, z_bias, np.degrees(pitch))
        print 'std_cam %d target_cam %d x_bias %.2f z_bias %.2f pitch %.2f degree'% (std_cam_id, cam_id, self.psn_unit_param[0], self.psn_unit_param[2], np.degrees(self.psn_unit_param[4]))

        # ROS-param setting for new one.
        rospy.set_param('/psn_unit' + str(cam_id), self.psn_unit_param)
        self.calibrated_cam_table[cam_id-1] = True

    def setGlobalParam(self,cam_id, std_cam_id ,x_bias, z_bias, pitch ):

        # load a previous target cam-position-RPY param
        self.psn_unit_param = rospy.get_param('/psn_unit'+str(cam_id))

        # load a standard cam-position-RPY param which was used for calibration.
        std_coordinate_param = np.mean(np.asarray(self.global_parameter_table[std_cam_id-1]), axis = 0).tolist()


        input_location = [x_bias, 0, z_bias] # X, Y, X
        std_cam_param = std_coordinate_param
        std_cam_param[3] = 0

        # Transforming new cam-position-RPY param to the
        transformed_location = self.CoordinateTransform(input_location, std_cam_param)

        # Rounding the param value
        self.psn_unit_param[0] = round(transformed_location[0],2) # new camera X_bias
        self.psn_unit_param[2] = round(transformed_location[2],2) # new camera Z_bias
        self.psn_unit_param[4] = round(pitch + std_cam_param[4] ,2)  # pitch

        #print 'local2std x_bias %.2f z_bias %.2f pitch %.2f degree' % (x_bias, z_bias, np.degrees(pitch))
        #print 'std_cam %d target_cam %d x_bias %.2f z_bias %.2f pitch %.2f degree'% (std_cam_id, cam_id, self.psn_unit_param[0], self.psn_unit_param[2], np.degrees(self.psn_unit_param[4]))

        # ROS-param setting for new one.
        self.global_parameter_table[cam_id-1].append(self.psn_unit_param)

    def plot_param(self, data, means):
        data_np = np.asarray(data)
        fig = plt.figure()
        #plt.subplot(211)
        ax = fig.gca(projection='3d')
        ax.scatter(data_np.T[0],data_np.T[1],data_np.T[2])
        plt.hold()
        ax.scatter(means.item(0), means.item(1), means.item(2), color = 'red')
        #plt.subplot(234)
        #plt.hist(data_np.T[0],10)
        #plt.subplot(235)
        #plt.hist(data_np.T[1],10)
        #plt.subplot(236)
        #plt.hist(data_np.T[2], 10)

        plt.show()

    def calibration_callback(self,data):

        ###### remove data that is not subscribed during 0.1 sec
        T_du = 0.1

        if rospy.get_time() - self.kinect1_timer > T_du:
            self.kinect1_dat = MultiPersons()
        if rospy.get_time() - self.kinect2_timer > T_du:
            self.kinect2_dat = MultiPersons()
        if rospy.get_time() - self.kinect3_timer > T_du:
            self.kinect3_dat = MultiPersons()
        if rospy.get_time() - self.kinect4_timer > T_du:
            self.kinect4_dat = MultiPersons()


        ### append human_roi in each kinect
        allP = MultiPersons()
        allP.persons.extend(self.kinect1_dat.persons)
        allP.persons.extend(self.kinect2_dat.persons)
        allP.persons.extend(self.kinect3_dat.persons)
        allP.persons.extend(self.kinect4_dat.persons)

        # check unique person.
        camera_check = [0, 0, 0, 0]
        detection_pos = [np.asarray([np.array([])]),np.asarray([]),np.asarray([]),np.asarray([])]


        for person in allP.persons:

            # Verifying input using faster-RCNN-Detection Output
            if person.roi.do_rectify == False and self.object_filter == True:
                continue



            if person.cam_id.data == 'sn_kinect1':
                camera_check[0] += 1
                detection_pos[0] = np.array([person.location.x,person.location.y, person.location.z])

            elif person.cam_id.data == 'sn_kinect2':
                camera_check[1] += 1
                detection_pos[1] = np.array([person.location.x,person.location.y, person.location.z])

            elif person.cam_id.data == 'sn_kinect3':
                camera_check[2] += 1
                detection_pos[2] = np.array([person.location.x,person.location.y, person.location.z])

            elif person.cam_id.data == 'sn_kinect4':
                camera_check[3] += 1
                detection_pos[3] = np.array([person.location.x,person.location.y, person.location.z])

        camera_check_np = np.asarray(camera_check)


        # Camera check status publish for GUI-Display
        self.calib_status_pub.publish(str(camera_check)+ '\n'+ str(self.calibrated_cam_table))
        print self.calibrated_cam_table

        # Check whether the calibration is complete or incomplete.
        if np.count_nonzero(np.asarray(self.calibrated_cam_table))==4:

            #for cam_idx in range(len(camera_check)): # row target cam_idx

            print 'calibration complete!'
            self.calib_message_pub.publish('calibration complete!')
            self.calib_status_pub.publish(
                str(camera_check) + '\n' + str(self.calibrated_cam_table))
            exit(1)
        else:
            '''
            if self.once_flag == True:
                self.algorithm_working_time_end = time.time()
                duration = self.algorithm_working_time_end - self.algorithm_working_time_start
                if duration > 10.:
                    count_matrix = np.zeros((4,4))
                    for i in range(4):
                        for j in range(4):
                            count_matrix[i][j] = len(self.parameter_table[i][j])

                    #To set global calibration param. we start from cam1(std_cam)
                    # Target selection
                    std_cam_idx = 0
                    while(1):


                        target_cam_idx = count_matrix[:, std_cam_idx].argsort()[::-1][0]
                        tmp_index = 1

                        while (1):
                            if self.global_parameter_table[target_cam_idx]:
                                target_cam_idx = count_matrix[:, std_cam_idx].argsort()[::-1][tmp_index]
                                tmp_index = tmp_index +1
                            else:
                                break



                        if self.calibrated_cam_table[std_cam_idx] == True:
                            if self.calibrated_cam_table[target_cam_idx] == False:
                                gmm = mixture.GaussianMixture(n_components=1, covariance_type='full')
                                gmm.fit(self.parameter_table[target_cam_idx][std_cam_idx])

                                x_bias = gmm.means_.item(0)
                                z_bias = gmm.means_.item(1)
                                pitch = gmm.means_.item(2)

                                self.setRosParam(target_idx + 1, std_idx + 1, x_bias, z_bias, pitch)
                                std_cam_idx = target_cam_idx
                                continue

                            gmm = mixture.GaussianMixture(n_components=1, covariance_type='full')
                            gmm.fit(self.parameter_table[target_cam_idx][std_cam_idx])

                            x_bias = gmm.means_.item(0)
                            z_bias = gmm.means_.item(1)
                            pitch = gmm.means_.item(2)

                            self.setGlobalParam(target_cam_idx + 1, std_cam_idx + 1, x_bias, z_bias, pitch)


                        else:

                            gmm = mixture.GaussianMixture(n_components=1, covariance_type='full')
                            gmm.fit(self.parameter_table[target_cam_idx][std_cam_idx])

                            x_bias = gmm.means_.item(0)
                            z_bias = gmm.means_.item(1)
                            pitch = gmm.means_.item(2)

                            self.setRosParam(target_idx + 1, std_idx + 1, x_bias, z_bias, pitch)
                            std_cam_idx = target_cam_idx

            '''




            for target_idx in range(len(self.parameter_table)):
                for std_idx in range(len(self.parameter_table[target_idx])):

                    # Update using multi-parmeter sample.
                    if len(self.parameter_table[target_idx][std_idx]) > self.SAMPLE_NUM \
                            and self.calibrated_cam_table[target_idx] == False\
                            and self.calibrated_cam_table[std_idx] == True:
                        count_matrix = np.zeros((4, 4))
                        for i in range(4):
                            for j in range(4):
                                count_matrix[i][j] = len(self.parameter_table[i][j])
                        print count_matrix
                        print 'Enough stored! target : %d std : %d' %(target_idx+1, std_idx+1)
                        gmm = mixture.GaussianMixture(n_components=1, covariance_type='full')
                        gmm.fit(self.parameter_table[target_idx][std_idx])
                        x_bias = gmm.means_.item(0)
                        z_bias = gmm.means_.item(1)
                        pitch = gmm.means_.item(2)
                        self.setRosParam(target_idx + 1, std_idx + 1, x_bias, z_bias, pitch)
                        continue
                        #self.plot_param(self.parameter_table[2][0],gmm.means_)
                    #else:
                    #    print 'lack of pair std : %d target : %d pairs : %d '%(
                    #        target_idx+1, std_idx+1,len(self.parameter_table[target_idx][std_idx]))

            


        # Calibration condition violation
        print camera_check
        if (camera_check_np > 1).any():
            print 'unique person fail!\n'
            self.calib_message_pub.publish('unique person fail!')
            #print camera_check
            self.person_trajectory = []

        elif (camera_check_np == 0).all():
            print 'no one!\n'
            self.calib_message_pub.publish('no one!')
            self.person_trajectory = []

        elif np.count_nonzero(camera_check_np == 1) < 2:
            print 'psn_off(Only one camera has a detection.)'
            self.calib_message_pub.publish('Only one camera has a detection.')

            #print camera_check
            self.person_trajectory = []

        # Calibration Algorithm work!
        else:
            #print 'condition satisified!'
            #print camera_check

            # At the first time, trajectory is empty
            if not self.person_trajectory:
                self.person_trajectory = []
                self.person_trajectory.append(detection_pos)

            # Calibration algorithm is working here!
            else:
                self.person_trajectory.append(detection_pos)
                #print 'seq of trajectory : ', len(self.person_trajectory)
                self.calib_message_pub.publish('Calibrationing...')

                # Searching position distance from current frame to previous frame.
                for trajectory_seq_index in range(len(self.person_trajectory)-1,0,-1):

                    current_position = np.asarray(self.person_trajectory[len(self.person_trajectory)-1])
                    searching_position = np.asarray(self.person_trajectory[trajectory_seq_index-1])


                    # Find the distance between detections in the matching camera.
                    position_diff = np.empty([4, 3])
                    for cam_index in range(len(current_position)):
                        if current_position[cam_index].any() & searching_position[cam_index].any():
                            position_diff[cam_index] = current_position[cam_index] - searching_position[cam_index]
                        else :
                            position_diff[cam_index] = np.array([0,0,0])

                    position_diff_square = np.square(position_diff)

                    distance = []
                    for squared_positon_each_cam in position_diff_square:
                        if squared_positon_each_cam.any():
                            distance.append(np.sqrt(squared_positon_each_cam[0] + squared_positon_each_cam[2]))
                        else:
                            distance.append(0.)


                    # Calibration Algorithm Start.00000000000000000000000000000000000000000000000000000 nm.0
                    if np.count_nonzero(np.asarray(distance) > self.CALIB_DIST) >=2 :
                        if self.once_flag == False:
                            self.algorithm_working_time_start = time.time()
                            self.once_flag = True

                        #print 'distance : %.2f %.2f %.2f %.2f' % (distance[0], distance[1], distance[2], distance[3])
                        trajectory_substract = position_diff


                        substract_matrix = np.zeros((2,2))
                        '''substract_matrix
                        
                        a   b     standard_cam_position_diff
                        a'  b'    other_cam_position_diff
                        
                        '''

                        # camera loop
                        i = 0
                        std_cam_idx = 0
                        target_cam_idx = 0
                        '''
                        std_cam_check = False
                        target_cam_check = False

                        for cam_data in trajectory_substract:
                            if (np.asarray(distance) > self.CALIB_DIST)[i]:

                                # already calibrated camera, this cam will be standard cam.
                                if self.calibrated_cam_table[i]:
                                    substract_matrix[0] = cam_data[0::2] # a, b
                                    std_cam_idx = i
                                    i = i + 1
                                    std_cam_check = True
                                    continue

                                # not calibrated camera, this cam will be target cam.
                                else:
                                    substract_matrix[1] = cam_data[0::2] # a' ,b'
                                    target_cam_idx = i
                                    i = i + 1

                                    target_cam_check = True
                                    continue
                            else:
                                #
                                i = i+1
                        

                        # condition check!
                        if std_cam_check == False or target_cam_check == False:
                            print 'Error! occured!'
                            break
                        '''

                        permutation_result = itertools.permutations(np.where(np.asarray(distance)>self.CALIB_DIST)[0],2)

                        for (std_cam_idx, target_cam_idx) in permutation_result:
                            substract_matrix[0] = trajectory_substract[std_cam_idx][0::2]  # a, b
                            substract_matrix[1] = trajectory_substract[target_cam_idx][0::2]  # a' ,b'

                            [a ,b] , [a_, b_] = substract_matrix

                            if a >= 0:
                                theta1 = np.arctan2(b, a)
                                theta2 = np.arctan2(b_, a_)
                            else :
                                theta1 = np.arctan2(b, a) + np.pi
                                theta2 = np.arctan2(b_,a_) + np.pi

                            # resolve pitch value
                            pitch = theta1 - theta2
                            if pitch < 0 :
                                pitch = pitch + 2*np.pi

                            # resolve transform_matrix
                            Transform_mat = np.matrix([[np.cos(pitch), -np.sin(pitch)],
                                                       [np.sin(pitch), np.cos(pitch)]])

                            # parameter result from point1
                            result1 = np.asmatrix(current_position[std_cam_idx][0::2]).T \
                                 - Transform_mat*np.asmatrix(current_position[target_cam_idx][0::2]).T

                            # parameter result from point2
                            result2 = np.asmatrix(searching_position[std_cam_idx][0::2]).T \
                                      - Transform_mat * np.asmatrix(searching_position[target_cam_idx][0::2]).T



                            x_bias = (result1.item(0) + result2.item(0))/2.
                            z_bias = (result1.item(1) + result2.item(1))/2.

                            #record calibration parameter
                            #                   target_cam_id
                            self.parameter_table[target_cam_idx][std_cam_idx].append([x_bias, z_bias, pitch])

                            # calibration_target_cam, standard_cam, x_bias, z_bias, pitch
                            #self.setRosParam(target_cam_idx+1, std_cam_idx+1 ,x_bias, z_bias, pitch)
                        #break

            # For step-by-step calibration from cam1 -> cam3 -> etc
            # Unnecessary
            '''
            elif (~np.asarray(detection_pos[0]).any()) & (
                np.count_nonzero(np.asarray(self.calibrated_cam_table)) == 1):

                self.person_trajectory = []
                print 'default calibration failed! go to camera1 view sight!'
                self.calib_message_pub.publish('go to camera1 view sight!!')
            '''







if __name__ == '__main__':

    try:
        WhereCalibration()
    except rospy.ROSInterruptException:
        pass
