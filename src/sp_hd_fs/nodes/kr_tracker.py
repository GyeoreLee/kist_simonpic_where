from math import *
import numpy as np
import time
import datetime

from numpy import matrix

from munkres import Munkres
from scipy.spatial.distance import pdist, squareform
from scipy.stats import multivariate_normal
from sklearn import mixture

class kr_tracker:

    Alpha = 255
    T = 0.6  # Birth Distance Param
    D_TH = 0.99 # Death_Threshhold value
    score_rate = 0.99
    decay_rate = 0.97
    stored_timestamp = 0
    print_mode = 1
    debug_mode = False
    sigma = 0.1


    def __init__(self, sampling_rate):

        print "### Multi-target tracking start ###"
        sampling_rates = sampling_rate
        self.timespan = 1. / sampling_rates

        SIGMA = self.sigma  # measuremnet uncertainty size

        # Kalman states& uncertainty matrix define
        self.X = list()
        self.P = list() #covariance matrix

        # Track_obj_num for init
        self.Track_obj_num = 0

        # Kalman filter's unvariable param
        self.B = np.matrix([(self.timespan ** 2 / 2.), (self.timespan ** 2 / 2.), self.timespan, self.timespan]).T  # external motion
        self.F = np.matrix(
            [[1., 0., self.timespan, 0.],
             [0., 1., 0., self.timespan],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]
             ])

        # next state function: generalize the 2d version to 4d
        self.H = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.]
                            ])
        # measurement function: reflect the fact that we observe X and y but not the two velocities size : 2 by 4 matrix
        self.R = np.matrix(
            [[SIGMA ** 2, 0.],
             [0., SIGMA ** 2],
            ])

        # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
        self.I = np.matrix(np.eye(4))  # 4d identity matrix

        # Q = I
        self.Q = np.matrix([
            [self.timespan ** 4 / 4., 0., self.timespan ** 3 / 2., 0.],
            [0., self.timespan ** 4 / 4., 0., self.timespan ** 3 / 2.],
            [self.timespan ** 3 / 2., 0., self.timespan ** 2., 0.],
            [0., self.timespan ** 3 / 2., 0., self.timespan ** 2.],
            ]) * 1

        # control input
        self.u = 0.

        # output file setting for debug
        if self.debug_mode:
            now = datetime.datetime.now()
            nowDatetime = now.strftime('%Y-%m-%d %H:%M:%S')
            self.file_p = open('where_tracker_'+nowDatetime+'.log','w')
            self.file_p.write('parameter setting\nT : %f\nD_TH : %f\nscore_rate : %f\ndecay_rate : %f\nSIGMA : %f\n'
                         % (self.T, self.D_TH, self.score_rate, self.decay_rate, SIGMA)
                         )

        # set_seq_num for debug
        self.seq = 0

    # set tracker seq according to main program
    def set_seq(self, input_seq):
        # set tracker_seq
        self.seq = input_seq

    # renew timespan param
    def set_param(self,timestamp):



        #float_format input timestamp
        f_timestamp = float(timestamp)

        #print 'stored : ',self.stored_timestamp
        #print 'current : ',timestamp
        if (self.stored_timestamp == 0):
            self.timespan = 0.03333
        # this case is about between diff frame but same timestamp in rosbag
        elif(self.stored_timestamp > f_timestamp) :
            print 'time unmatch!', self.stored_timestamp, f_timestamp
            self.stored_timestamp = f_timestamp
            return 0
            #self.timespan = 0.03333
            #self.timespan = self.stored_timestamp - f_timestamp
        elif(f_timestamp > self.stored_timestamp):
            self.timespan = f_timestamp - self.stored_timestamp
        else:
            self.timespan = 0.03333


        # renew parameter using frame interval
        print 'renewed timespand : ',self.timespan

        if self.debug_mode:
            self.file_p.write('%d renewed timespand : %f\n' % (self.seq, self.timespan))

        self.B = np.matrix(
            [(self.timespan ** 2 / 2.), (self.timespan ** 2 / 2.), self.timespan, self.timespan]).T  # external motion


        self.F = np.matrix(
            [[1., 0., self.timespan, 0.],
             [0., 1., 0., self.timespan],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.],
            ])

        # next state function: generalize the 2d version to 4d
        self.Q = np.matrix([[self.timespan ** 4 / 4., 0., self.timespan ** 3 / 2., 0.],
                            [0., self.timespan ** 4 / 4., 0., self.timespan ** 3 / 2.],
                            [self.timespan ** 3 / 2., 0., self.timespan ** 2., 0.],
                            [0., self.timespan ** 3 / 2., 0., self.timespan ** 2.],
                            ]) * 1

        self.stored_timestamp = f_timestamp

    def clamp(self,n, minn, maxn):
        return max(min(maxn, n), minn)

    # Generation initial tracking_obj
    def state_init(self,X, P, measurement):

        '''
        X.append([measurement[1], np.matrix([measurement[2], measurement[3], 0.,
                                             0., measurement[4], measurement[5], measurement[6]]).T,
                  1.0  # init_score
                     ,
                  False])  # initial state feature [tracker_id,[Position X, Position Y, Velocity X, Velocity y ],score,update_check]
        P.append(
            [measurement[1], matrix(np.eye(7) * 1000)])  # initial uncertainty: [tracker_id,[uncertainty X, Y,Vx,Vy]]
        '''



        # check empty Track_obj_num(id)
        id_table = []
        for i in range(len(X)) :
            exist_id = X[i][0]
            id_table.append(exist_id)

        id_table.sort()

        for i in range(len(id_table)):
            if i < id_table[i]:
                self.Track_obj_num = i
                break
            else:
                self.Track_obj_num = i+1

        # initial state feature [tracker_id,[Position X, Position Y, Velocity X, Velocity y ],score,update_check, birthing_time]
        X.append([self.Track_obj_num, np.matrix([measurement[2], measurement[3], 0., 0.]).T,
                  1.0,  # init_score
                  False, # update_check
                  time.time() # survival_time
                  ])
        P.append(
            [self.Track_obj_num, matrix(np.eye(4) * 1000)])  # initial uncertainty: [tracker_id,[uncertainty X, Y,Vx,Vy]]

        if self.debug_mode:

            self.file_p.write(
                '%d Birth! model_id : %d \n' % (self.seq, self.Track_obj_num))


        #self.Track_obj_num += 1
        return X, P

    # Kalman updating and prediction
    def Kalman_filter(self,X, P, Z, B, F, H, R, I, Q, u):  # X : state, P : uncertainty Z : measurements
        if (Z.any()):
            # measurement update
            y = Z - (H * X)
            S = H * P * H.T + R
            K = P * H.T * S.I
            X = X + (K * y)
            P = (I - (K * H)) * P

        # prediction
        X = (F * X) + B * u
        P = F * P * F.T + Q

        return X, P

    # Kalman only updating
    def Kalman_filter_update(self, X, P, Z, B, F, H, R, I, Q, u):  # X : state, P : uncertainty Z : measurements
        if (Z.any()):
            # measurement update
            y = Z - (H * X)
            S = H * P * H.T + R
            K = P * H.T * S.I
            X = X + (K * y)
            P = (I - (K * H)) * P
        else :
            print 'empty measurement! can not update model!'

        return X, P

    # FeatureNormalize to compute distance
    def featureNormalize(self,List, Size):
        return np.asarray(List) / Size

    # delete tracking_obj
    def obj_pruning(self,X,P):
        # death_check
        death_candidate_num = 0
        for n in range(len(X)):
            # Death_Check
            if X[n][2] < kr_tracker.D_TH:
                print 'death id : ', X[n][0]

                if self.debug_mode:
                    self.file_p.write('%d death id :  %d\n' % (self.seq, X[n][0]))

                death_candidate_num += 1
        while death_candidate_num:
            for n in range(len(X)):
                if X[n][2] < self.D_TH:
                    print X[n][2]
                    X.pop(n)
                    P.pop(n)
                    death_candidate_num -= 1
                    break

    # Check and Birth tracking_obj
    def obj_birthing(self,measurements, mat_form, X, P, indexes):

        birth_check = False

        for row in range(len(mat_form)): # measurements_num

            # condition1 : Assign Cost > Threshold
            condition1 = False
            model_index = 0
            for model_index in range(len(indexes)):
                if row == indexes[model_index][0]:
                    condition1 = mat_form[row][indexes[model_index][1]] > kr_tracker.T
                    break


            # condition2 : not-assigned_measurements birthing
            condition2 = False
            for assign_info in indexes:
                if assign_info[0] == row:
                    condition2 = False
                    break
                condition2 = True

            # condition3: no-tracking model(init)
            condition3 = ~mat_form[row].any()

            if (condition1 or condition2 or condition3) :

                if self.debug_mode:

                    if condition1:
                        self.file_p.write('%d dist between(%d) : %f \n'
                                          % (self.seq, indexes[model_index][1],mat_form[row][indexes[model_index][1]]))

                    self.file_p.write('%d Birth! meas_id : %d signal : %s %s %s\n'
                                      % (self.seq, measurements[row][1],condition1, condition2, condition3))
                    #, 'dist : ', mat_form[row].min()

                X, P = self.state_init(X, P, measurements[row])


                #if condition1 == True:

                #    indexes[row] = indexes[row][:1] + (len(X) -1,) # change assigned model_index to new one.

                #elif condition2 == True:
                #   indexes.append(tuple([row,len(X)-1]))
                #elif condition3 == True:
                #    indexes.append(tuple([row, len(X) - 1]))


                birth_check = True





        return birth_check, X, P


    # Scoring
    def score(self, score,  BnD_flag):
        if BnD_flag:
            score = (score + 1) * self.score_rate
        else:
            score = score * self.decay_rate

        return score

    ## Distance calculation
    #def distance(self,):

    # Matching between observation and tracking_obj
    def association(self, measurements, X, P):

        print '(t) Detection_num : ', len(measurements), '(t-1)tracking_num :', len(X)

        # log-likelihood of Gaussian pdf
        '''
        location_likelihood_mat = []
        # Row : model Column: measurements
        for n in range(len(X)):
            location_likelihood_mat.append([])
            for measure_elements in measurements:
                # init gaussian pdf
                pdf = multivariate_normal(np.asarray(X[n][1]).T[0][0:2].tolist(),
                                          [[P[n][1].item(0), P[n][1].item(1)], [P[n][1].item(7), P[n][1].item(8)]])

                # compute likelihood
                location_likelihood_mat[n].append(pdf.logpdf(measure_elements[2:4]))

        location = (-np.asarray(location_likelihood_mat)).T
        '''

        # prepare 1D array of measurements & tracking Obj
        measurement_pool = []
        print'measure_id|track_id'
        for n in range(len(measurements)):
            #normalized_color = self.featureNormalize(measurements[n][4:7], kr_tracker.Alpha)
            measurement_pool.append(measurements[n][2:4])# + normalized_color.tolist())
            print measurements[n][1],

        print '|',

        for n in range(len(X)):
            #try:
            #    normalized_color = self.featureNormalize(np.choose([4, 5, 6], X[n][1].tolist()).tolist(), kr_tracker.Alpha)
            #except:
            #    print e
            measurement_pool.append(np.choose([0, 1], X[n][1].tolist()).tolist())# + normalized_color.tolist())
            print X[n][0],
        print '\n'
        # make a list using X and measurements

        # compute distance between model with measurements
        squared = squareform(pdist(measurement_pool))
        # print 'squared\n',squared,
        #mat_form = squared[0:len(measurements), len(measurements)-1:]  #
        mat_form = squared[0:len(measurements), len(measurements):]  #



        # Give a penalty to the cost matrix using a score.
        score_list = []
        for model in X:
            model_score = model[2]
            score_list.append(model_score)

        score_array = np.asarray(score_list)
        pre_penaltied_cost = np.empty_like(mat_form)
        pre_penaltied_cost[:] = mat_form
        mat_form = mat_form + (0.3 / score_array)

        # hungarian method applied(pre-birthing)
        # print '\npre-birthing mat_form\n', mat_form
        m = Munkres()
        indexes = m.compute(mat_form.tolist())


        # Birth Check
        #birth_check, X, P, indexes  = self.obj_birthing(measurements, pre_penaltied_cost, X, P, indexes)
        birth_check, X, P = self.obj_birthing(measurements, pre_penaltied_cost, X, P,indexes)

        if birth_check:
            # make a list using X and measurements
            measurement_pool = []
            print'measure_id|track_id'
            for n in range(len(measurements)):
                #normalized_color = self.featureNormalize(measurements[n][4:7], kr_tracker.Alpha)

                measurement_pool.append(measurements[n][2:4])# + normalized_color.tolist())

                print measurements[n][1],

            print '|',

            for n in range(len(X)):
                #normalized_color = self.featureNormalize(np.choose([4, 5, 6], X[n][1].tolist()).tolist(),
                #                                         kr_tracker.Alpha)
                measurement_pool.append(np.choose([0, 1], X[n][1].tolist()).tolist())# + normalized_color.tolist())
                print X[n][0],
            print '\n'



            # hungarian method applied
            squared = squareform(pdist(measurement_pool))
            # print 'squared\n',squared,
            mat_form = squared[0:len(measurements), len(measurements):]  #


            # Give a penalty to the cost matrix using a score.

            score_list = []
            for model in X:
                model_score = model[2]
                score_list.append(model_score)

            score_array = np.asarray(score_list)
            pre_penaltied_cost = np.empty_like(mat_form)
            pre_penaltied_cost[:] = mat_form
            mat_form = mat_form + (0.3 / score_array)


        # distance likelihood matrix
        #print 'location_mat_form(likelihood)\n', location
        #indexes = m.compute(location.tolist())
        #print indexes




        #print '\nmat_form\n', mat_form
        m = Munkres()
        indexes = m.compute(mat_form.tolist())

        # for Debug
        if self.debug_mode:

            # Alive model & its score
            self.file_p.write('%d current alive model_id\n%d ' % (self.seq, self.seq))
            for n in range(len(X)):

                self.file_p.write('%d (%.2f) ' % (X[n][0],X[n][2] ))

            self.file_p.write('\n')


            for n in range(len(indexes)):
                meas_index = indexes[n][0]  # meas_index
                obj_index = indexes[n][1]  # obj_index

                #print 'measure[%d]=>obj[%d]' % (measurements[meas_index][1], X[obj_index][0])
                self.file_p.write('%d measure[%d]=>obj[%d] score: %.2f penaltied_cost: %.2f cost: %.2f\n' %
                                  (self.seq, measurements[meas_index][1], X[obj_index][0], X[obj_index][2],
                                   mat_form[meas_index][obj_index], pre_penaltied_cost[meas_index][obj_index]
                                   ))

            self.file_p.write('---------------------------------------------------------------------------\n')



        miss_num = len(measurements) - len(indexes)

        return indexes, X, P, miss_num  # , miss_data

    '''
    def association(self, measurements, X, P, color_likelihood_result, color_likelihood_idTable):

        print '(t) Detection_num : ', len(measurements), '(t-1)tracking_num :', len(X)


        # log-likelihood of Gaussian pdf

        location_likelihood_mat = []
        # Row : model Column: measurements
        for n in range(len(X)):
            location_likelihood_mat.append([])
            for measure_elements in measurements:

                # init gaussian pdf
                pdf = multivariate_normal(np.asarray(X[n][1]).T[0][0:2].tolist(),
                [[P[n][1].item(0),P[n][1].item(1)],[P[n][1].item(7),P[n][1].item(8)]])

                # compute likelihood
                location_likelihood_mat[n].append(pdf.logpdf(measure_elements[2:4]))

        location = (-np.asarray(location_likelihood_mat)).T

        if color_likelihood_result:

            # Row : model Column : measure => Row : measure Column : model
            color = (-np.asarray(color_likelihood_result)).T


            # remove not detected rows from color_likelihood table
            if len(color_likelihood_idTable)  > len(measurements):
                color = color.tolist()
                tmp_id_table = list(color_likelihood_idTable)

                for measure_elements in measurements :
                    tmp_id_table.remove(measure_elements[1])

                # not detected measure_id remove from color matrix
                for notMeasID in tmp_id_table:

                    color.pop(color_likelihood_idTable.index(notMeasID))
                    #try:
                    #    color.pop(color_likelihood_idTable.index(notMeasID))
                    #except:
                    #    continue

                color = np.asarray(color)


            fused_likelihood = 1*color + 1*location



        # GMM initialize
        #gmm = mixture.GaussianMixture(n_components=1, covariance_type='diag', means_init=np.squeeze(np.asarray(X[0][1])
        #,weights_init=np.array([1.]))
        #    )        np.squeeze(np.asarray(X[0][1]))

        # prepare 1D array of measurements & tracking Obj
        measurement_pool = []
        print'measure_id|track_id'
        for n in range(len(measurements)):
            normalized_color = self.featureNormalize(measurements[n][4:7], kr_tracker.Alpha)
            measurement_pool.append(measurements[n][2:4] + normalized_color.tolist())
            print measurements[n][1],

        print '|',

        for n in range(len(X)):
            normalized_color = self.featureNormalize(np.choose([4, 5, 6], X[n][1].tolist()).tolist(), kr_tracker.Alpha)
            measurement_pool.append(np.choose([0, 1], X[n][1].tolist()).tolist() + normalized_color.tolist())
            print X[n][0],

        # make a list using X and measurements

        # hungarian method applied
        squared = squareform(pdist(measurement_pool))

        # Row : measurements, Column: model
        # print 'squared\n',squared,
        mat_form = squared[0:len(measurements), len(measurements):]  #

        # Birth Check
        birth_check = False
        # birth_check, X, P  = self.obj_birthing(measurements, mat_form, X, P)

        if birth_check:
            # make a list using X and measurements
            measurement_pool = []
            print'measure_id|track_id'
            for n in range(len(measurements)):
                normalized_color = self.featureNormalize(measurements[n][4:7], kr_tracker.Alpha)
                measurement_pool.append(measurements[n][2:4] + normalized_color.tolist())
                print measurements[n][1],

            print '|',

            for n in range(len(X)):
                normalized_color = self.featureNormalize(np.choose([4, 5, 6], X[n][1].tolist()).tolist(),
                                                         kr_tracker.Alpha)
                measurement_pool.append(np.choose([0, 1], X[n][1].tolist()).tolist() + normalized_color.tolist())
                print X[n][0],


            # hungarian method applied
            squared = squareform(pdist(measurement_pool))
            # print 'squared\n',squared,
            mat_form = squared[0:len(measurements), len(measurements):]  #

        # distance matrix
        print '\nlocation_mat_form\n', mat_form
        m = Munkres()
        indexes = m.compute(mat_form.tolist())
        print indexes

        # distance likelihood matrix
        print 'location_mat_form(likelihood)\n', location
        indexes = m.compute(location.tolist())
        print indexes


        # using color feature applied likelihood to hungarian's method
        if color_likelihood_result:
            # color likelihood matrix
            print 'color_mat_form(likelihood)\n', color
            indexes = m.compute(color.tolist())
            print indexes

            print '##########using fused_likelihood!###############'
            print 'fused_mat_form(likelihood)\n', fused_likelihood
            indexes = m.compute(fused_likelihood.tolist())
            print indexes

        miss_num = len(measurements) - len(indexes)

        return indexes, X, P, miss_num  # , miss_data
    '''