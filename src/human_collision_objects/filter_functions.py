#!/usr/bin/env python

import rospy
#import dependencies
import numpy as np
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d,Axes3D
from human_collision_objects.quat_functions import *
# import matplotlib
# matplotlib.use("WxAgg")
            
class human_filter():

    dims_arr = np.empty((7,0))
    dims_avg = np.zeros((7))
    filt = butter(1, 0.1, btype='lowpass', analog=False)
    prevTheta = np.zeros((28,1))
    angs_vecs = np.zeros((28,1))
    t_spine = np.zeros((4,4))
    t_head = np.zeros((4,4))
    t_shoulder_1 = np.zeros((4,4))
    t_elbow_1 = np.zeros((4,4))
    t_wrist_1 = np.zeros((4,4))
    t_shoulder_2 = np.zeros((4,4))
    t_elbow_2 = np.zeros((4,4))
    t_wrist_2 = np.zeros((4,4))
    dims_ready = False
    dims_ratios = np.ones((7))
    dims_reject = 0
    dims_error = np.zeros((7))
    ang_hist = np.empty((28,0))
    ang_change_rate = np.zeros((7))
    vec_change_rate = np.zeros((7))
    dimension_ratios = np.empty((7,0))
    dimensions_matrix = np.empty((7,0))
    calibrate_dimensions = True

    #This function parses the input from the Intel Skeleton Tracker SDK skeleton model sequence
    #The input was 1x45 row vector in the form [x,y,z,x,y,z,...] so it needed parsed into 
    #3x15 matrices with one column per joint
    def parseCoordsIntel(self,coords):  
        coords=coords.T
        points = np.zeros((3,15))
        points[:,0] = 0.5*(coords[:,8]+coords[:,11]) #pelvis (hips midpoint)
        points[:,1] = 0.5*(coords[:,2]+coords[:,5]) #spine (shoulders midpoint)
        points[:,2] = coords[:,0] #head
        points[:,3] = coords[:,2] #shoulder 1
        points[:,4] = coords[:,3] #elbow 1
        points[:,5] = coords[:,4] #wrist 1
        points[:,6] = coords[:,5] #shoulder 2
        points[:,7] = coords[:,6] #elbow 2
        points[:,8] = coords[:,7] #wrist 2
        points[:,9] = coords[:,8]
        points[:,10] = coords[:,9]
        points[:,11] = coords[:,10]
        points[:,12] = coords[:,11]
        points[:,13] = coords[:,12]
        points[:,14] = coords[:,13]
        #points = rotx(-90,'deg')@points
        return points

    def fix_angles(self,theta):
        theta_diff = theta-self.prevTheta
        #3 to -3->theta_diff =-6->theta_diff = 0.28
        theta_diff = theta_diff-(np.abs(theta_diff)>np.pi)*np.sign(theta_diff)*2*np.pi
        theta_diff = (np.abs(theta_diff)<=0.05)*theta_diff+(np.abs(theta_diff)>0.05)*np.sign(theta_diff)*0.05
        theta = self.prevTheta+theta_diff
        return theta.reshape((21,1)),theta_diff.reshape((21,1))

    def get_dimensions(self,points):        
        return np.array([np.linalg.norm(points[:,1]-points[:,0]),np.linalg.norm(points[:,2]-points[:,1]),
                         np.linalg.norm(points[:,6]-points[:,3]),np.linalg.norm(points[:,4]-points[:,3]),
                         np.linalg.norm(points[:,5]-points[:,4]),np.linalg.norm(points[:,7]-points[:,6]),
                         np.linalg.norm(points[:,8]-points[:,7])])

    def filter_input(self,points):
        ang_success,theta_raw = self.JointCarts2Angles_Fixed(points) #get current human joint angles
        #theta,theta_diff = self.fix_angles(theta_raw.reshape((28,1)))
        self.ang_hist = np.append(self.ang_hist,theta_raw.reshape((28,1)),axis=1)
        self.prevTheta = theta_raw.reshape((28,1))
        if self.ang_hist.shape[1]>10:
            self.ang_hist = np.delete(self.ang_hist,0,1)
        tmp = lfilter(self.filt[0],self.filt[1],self.ang_hist) #smooth the computed angles with the low pass filter
        dims = self.get_dimensions(points)
        if self.dims_arr.shape[1]>10:
            self.dims_arr = np.delete(self.dims_arr,0,1)
            if np.count_nonzero(self.dims_avg)==self.dims_avg.shape[0]:
                self.dims_ready = True
            else:
                self.dims_ready = False
        else:
            self.dims_ready = False

        if self.dims_arr.shape[1]>2:
            tol = 2.0*self.dims_avg
            tol[3] = max(tol[3],tol[5])
            tol[5] = max(tol[3],tol[5])
            tol[4] = max(tol[4],tol[6])
            tol[6] = max(tol[4],tol[6])
            self.dims_error = np.abs(self.dims_avg-dims)
            if not max(dims[3],dims[5])==0:
                elbow_deviation = np.abs(dims[3]-dims[5])/max(dims[3],dims[5])
            else:
                elbow_deviation=1
            if not max(dims[4],dims[6])==0:
                wrist_deviation = np.abs(dims[4]-dims[6])/max(dims[4],dims[6])
            else:
                wrist_deviation = 1
            #print('{}, {}'.format(elbow_deviation,wrist_deviation))
            #print(dims)
            if (np.sum(self.dims_error>tol)==0) and (elbow_deviation<0.2) and (wrist_deviation<0.2):
                self.dims_arr = np.append(self.dims_arr,dims.reshape((7,1)),axis=1)
                self.dims_reject = 0
            else:
                self.dims_arr = np.append(self.dims_arr,dims.reshape((7,1)),axis=1)#added this
                self.dims_reject+=1
                #print('reject')
            if self.dims_reject>20:
                self.dims_arr = dims.reshape((7,1))
        else:
            self.dims_arr = np.append(self.dims_arr,dims.reshape((7,1)),axis=1)
        self.dims_avg = np.mean(self.dims_arr,axis=1)
        if not self.dims_avg[0]==0:
            self.dims_ratios = self.dims_avg/self.dims_avg[0]
        else:
            self.dims_ratios = np.zeros_like(self.dims_avg)
        return tmp[:,-1]

        
    #This function extracts the human joint angles from the 3x15 matrix of points, each
    #column is x,y,z for a skeleton point
    #Each joint has a roll,pitch,yaw for improved accuracy even though many joints
    #dont have 3 DOFs
    def JointCarts2Angles(self,points):
        vects = np.zeros((3,14))
        vects[:,0] = points[:,1]-points[:,0] #pelvis->spine
        vects[:,1] = points[:,2]-points[:,1] #spine->head top
        vects[:,2] = points[:,3]-points[:,1] #L shoulder->spine
        vects[:,3] = points[:,6]-points[:,1] #L elbow->L shoulder
        vects[:,4] = points[:,4]-points[:,3] #L wrist->L elbow
        vects[:,5] = points[:,5]-points[:,4] #R shoulder->spine
        vects[:,6] = points[:,7]-points[:,6] #R elbow->R shoulder
        vects[:,7] = points[:,8]-points[:,7] #R wrist->R elbow
        vects[:,8] = points[:,9]-points[:,0] #L hip->pelvis
        vects[:,9] = points[:,12]-points[:,0] #L knee->L hip
        vects[:,10] = points[:,10]-points[:,9] #L ankle->L knee
        vects[:,11] = points[:,11]-points[:,10] #R hip->pelvis
        vects[:,12] = points[:,13]-points[:,12] #R knee->R hip
        vects[:,13] = points[:,14]-points[:,13] #R ankle->R knee
        shoulder2shoulder = points[:,6]-points[:,3] #L shoulder->R shoulder vector

        theta = np.zeros((21,1))
        success = True
        #go from fixed z axis to the pelvis->spine vector
        #align x axis with new z (pelvis->spine vector) crossed with fixed y
        norm_z1 = np.linalg.norm(vects[:,0])
        if norm_z1>0:
            z_1 = vects[:,0]/norm_z1
            vec = np.cross(np.array([0,0,1]).T,z_1)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(np.array([0,0,1]).T,z_1))
                vec = vec/np.linalg.norm(vec)
            else:
                if np.array([0,0,-1]).all()==z_1.all():
                    ang = np.pi
                else:
                    ang = 0    
        else:
            z_1 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q1 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        #print('r:{}'.format(z_1))
        x = quaternion_multiply(q1,np.array([0,1,0,0]))
        y = quaternion_multiply(q1,np.array([0,0,1,0]))
        z_spine = quaternion_multiply(quaternion_multiply(q1,np.array([0,0,0,1])),quat_inv(q1))
        #print('z1:{}'.format(z_1))
        #print('q1:{}'.format(z_spine[1:]))
        #print('quats:{}'.format(np.concatenate((x[1:].reshape((3,1)),y[1:].reshape((3,1)),z[1:].reshape((3,1))),axis=1)))

        #go from spine z axis (z1) to spine->head vector
        #new y axis aligned with new z (spine->head vector) crossed with spine x axis (x1)
        norm_z2 = np.linalg.norm(vects[:,1])
        if norm_z2>0:
            z_2 = vects[:,1]/norm_z2
            vec = np.cross(z_1,z_2)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_1,z_2))
                vec = vec/np.linalg.norm(vec)
            else:
                if z_1.all()==z_2.all():
                    ang = 0
                else:
                    ang = np.pi    
        else:
            z_2 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q2 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_neck = quaternion_multiply(quaternion_multiply(q2,np.concatenate(([0],z_1))),quat_inv(q2))
        #print('z2:{}'.format(z_2))
        #print('q2:{}'.format(z_neck[1:]))

        #go form spine x axis (x_1) to the shoulder->shoulder vector
        #new y axis is spine z axis (z_1) crossed with shoulder->shoulder vector
        norm_x3 = np.linalg.norm(shoulder2shoulder)
        if norm_x3>0:
            x_3 = shoulder2shoulder/norm_x3
            vec = np.cross(z_1,x_3)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_1,x_3))
                vec = vec/np.linalg.norm(vec)
            else:
                if x_3.all()==z_1.all():
                    ang = 0
                else:
                    ang = np.pi    
        else:
            x_3 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q3 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_shoulders = quaternion_multiply(quaternion_multiply(q3,np.concatenate(([0],z_1))),quat_inv(q3))
        #print('z3:{}'.format(x_3))
        #print('q3:{}'.format(z_shoulders[1:]))

        z_4 = vects[:,4]
        norm_z4 = np.linalg.norm(z_4)
        if norm_z4>0:
            z_4 = z_4/norm_z4
            vec = np.cross(x_3,z_4)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(x_3,z_4))
                vec = vec/np.linalg.norm(vec)
            else:
                if x_3.all()==z_4.all():
                    ang = 0
                else:
                    ang = np.pi    
        else:
            z_4 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q4 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_e1 = quaternion_multiply(quaternion_multiply(q4,np.concatenate(([0],x_3))),quat_inv(q4))
        #print('z4:{}'.format(z_4))
        #print('q4:{}'.format(z_e1[1:]))

        #align the elbow z axis with the elbow1->wrist1 vector
        #align the wrist y axis with the cross product of the elbow1->wrist1 vector and the shoulder1->elbow1 vector
        norm_z5 = np.linalg.norm(vects[:,5])
        if norm_z5>0:
            z_5 = vects[:,5]/norm_z5
            vec = np.cross(z_4,z_5)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_4,z_5))
                vec = vec/np.linalg.norm(vec)
            else:
                if z_4.all()==z_5.all():
                    ang = 0
                else:
                    ang = np.pi
        else:
            z_5 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q5 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_w1 = quaternion_multiply(quaternion_multiply(q5,np.concatenate(([0],z_4))),quat_inv(q5))
        #print('z5:{}'.format(z_5))
        #print('q5:{}'.format(z_w1[1:]))

        #go from shoulder z axis to the opposite of the shoulder2->elbow2 vector
        #align the elbow y axis with the shoulder->shoulder vector crossed with elbow2->shoulder2
        z_6 = vects[:,6]
        norm_z6 = np.linalg.norm(z_6)
        if norm_z6>0:
            z_6 = z_6/norm_z6
            vec = np.cross(x_3,z_6)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(x_3,z_6))
                vec = vec/np.linalg.norm(vec)
            else:
                if x_3.all()==z_6.all():
                    ang = np.pi
                else:
                    ang = 0    
        else:
            z_6 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q6 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_e2 = quaternion_multiply(quaternion_multiply(q6,np.concatenate(([0],x_3))),quat_inv(q6))
        #print('z6:{}'.format(z_6))
        #print('q6:{}'.format(z_e2[1:]))

        #align the elbow z axis with the elbow2->wrist2 vector
        #align the wrist y axis with the cross product of the elbow2->wrist2 vector and the shoulder2->elbow2 vector
        norm_z7 = np.linalg.norm(vects[:,7])
        if norm_z7>0:
            z_7 = vects[:,7]/norm_z7
            vec = np.cross(z_6,z_7)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_6,z_7))
                vec = vec/np.linalg.norm(vec)
            else:
                if z_6.all()==z_7.all():
                    ang = 0
                else:
                    ang = np.pi
        else:
            z_7 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q7 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_w2 = quaternion_multiply(quaternion_multiply(q7,np.concatenate(([0],z_6))),quat_inv(q7)) 
        #print('z7:{}'.format(z_7))
        #print('q7:{}'.format(z_w2[1:])) 

        return success,np.concatenate((q1,q2,q3,q4,q5,q6,q7))

    #This function extracts the human joint angles from the 3x15 matrix of points, each
    #column is x,y,z for a skeleton point
    #Each joint has a roll,pitch,yaw for improved accuracy even though many joints
    #dont have 3 DOFs
    def JointCarts2Angles_Fixed(self,points):
        vects = np.zeros((3,14))
        vects[:,0] = points[:,1]-points[:,0] #pelvis->spine
        vects[:,1] = points[:,2]-points[:,1] #spine->head top
        vects[:,2] = points[:,3]-points[:,1] #L shoulder->spine
        vects[:,3] = points[:,6]-points[:,1] #L elbow->L shoulder
        vects[:,4] = points[:,4]-points[:,3] #L wrist->L elbow
        vects[:,5] = points[:,5]-points[:,4] #R shoulder->spine
        vects[:,6] = points[:,7]-points[:,6] #R elbow->R shoulder
        vects[:,7] = points[:,8]-points[:,7] #R wrist->R elbow
        vects[:,8] = points[:,9]-points[:,0] #L hip->pelvis
        vects[:,9] = points[:,12]-points[:,0] #L knee->L hip
        vects[:,10] = points[:,10]-points[:,9] #L ankle->L knee
        vects[:,11] = points[:,11]-points[:,10] #R hip->pelvis
        vects[:,12] = points[:,13]-points[:,12] #R knee->R hip
        vects[:,13] = points[:,14]-points[:,13] #R ankle->R knee
        shoulder2shoulder = points[:,6]-points[:,3] #L shoulder->R shoulder vector

        z_axis = np.array([0,0,1]).T

        theta = np.zeros((21,1))
        success = True
        #go from fixed z axis to the pelvis->spine vector
        #align x axis with new z (pelvis->spine vector) crossed with fixed y
        norm_z1 = np.linalg.norm(vects[:,0])
        if norm_z1>0:
            z_1 = vects[:,0]/norm_z1
            vec = np.cross(z_axis,z_1)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_axis,z_1))
                vec = vec/np.linalg.norm(vec)
            else:
                if (np.array([0,0,-1])==z_1).all():
                    ang = np.pi
                else:
                    ang = 0    
                vec = [1,0,0]  
        else:
            z_1 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q1 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        #print('r:{}'.format(z_1))
        x = quaternion_multiply(q1,np.array([0,1,0,0]))
        y = quaternion_multiply(q1,np.array([0,0,1,0]))
        z_spine = quaternion_multiply(quaternion_multiply(q1,np.array([0,0,0,1])),quat_inv(q1))
        #print('z1:{}'.format(z_1))
        #print('q1:{}'.format(z_spine[1:]))
        #print('quats:{}'.format(np.concatenate((x[1:].reshape((3,1)),y[1:].reshape((3,1)),z[1:].reshape((3,1))),axis=1)))

        #go from spine z axis (z1) to spine->head vector
        #new y axis aligned with new z (spine->head vector) crossed with spine x axis (x1)
        norm_z2 = np.linalg.norm(vects[:,1])
        if norm_z2>0:
            z_2 = vects[:,1]/norm_z2
            vec = np.cross(z_axis,z_2)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_axis,z_2))
                vec = vec/np.linalg.norm(vec)
            else:
                if (z_axis==z_2).all():
                    ang = 0
                else:
                    ang = np.pi 
                vec = [1,0,0]     
        else:
            z_2 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q2 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_neck = quaternion_multiply(quaternion_multiply(q2,np.concatenate(([0],z_axis))),quat_inv(q2))
        #print('z2:{}'.format(z_2))
        #print('q2:{}'.format(z_neck[1:]))

        #go form spine x axis (x_1) to the shoulder->shoulder vector
        #new y axis is spine z axis (z_1) crossed with shoulder->shoulder vector
        norm_x3 = np.linalg.norm(shoulder2shoulder)
        if norm_x3>0:
            x_3 = shoulder2shoulder/norm_x3
            vec = np.cross(z_axis,x_3)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_axis,x_3))
                vec = vec/np.linalg.norm(vec)
            else:
                if (x_3==z_axis).all():
                    ang = 0
                else:
                    ang = np.pi  
                vec = [1,0,0]    
        else:
            x_3 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q3 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_shoulders = quaternion_multiply(quaternion_multiply(q3,np.concatenate(([0],z_axis))),quat_inv(q3))
        #print('z3:{}'.format(x_3))
        #print('q3:{}'.format(z_shoulders[1:]))

        z_4 = vects[:,4]
        norm_z4 = np.linalg.norm(z_4)
        if norm_z4>0:
            z_4 = z_4/norm_z4
            vec = np.cross(z_axis,z_4)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_axis,z_4))
                vec = vec/np.linalg.norm(vec)
            else:
                if (z_axis==z_4).all():
                    ang = 0
                else:
                    ang = np.pi 
                vec = [1,0,0]  
        else:
            z_4 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q4 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_e1 = quaternion_multiply(quaternion_multiply(q4,np.concatenate(([0],z_axis))),quat_inv(q4))
        #print('z4:{}'.format(z_4))
        #print('q4:{}'.format(z_e1[1:]))

        #align the elbow z axis with the elbow1->wrist1 vector
        #align the wrist y axis with the cross product of the elbow1->wrist1 vector and the shoulder1->elbow1 vector
        norm_z5 = np.linalg.norm(vects[:,5])
        if norm_z5>0:
            z_5 = vects[:,5]/norm_z5
            vec = np.cross(z_axis,z_5)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_axis,z_5))
                vec = vec/np.linalg.norm(vec)
            else:
                if (z_axis==z_5).all():
                    ang = 0
                else:
                    ang = np.pi
                vec = [1,0,0]  
        else:
            z_5 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q5 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_w1 = quaternion_multiply(quaternion_multiply(q5,np.concatenate(([0],z_axis))),quat_inv(q5))
        #print('z5:{}'.format(z_5))
        #print('q5:{}'.format(z_w1[1:]))

        #go from shoulder z axis to the opposite of the shoulder2->elbow2 vector
        #align the elbow y axis with the shoulder->shoulder vector crossed with elbow2->shoulder2
        z_6 = vects[:,6]
        norm_z6 = np.linalg.norm(z_6)
        if norm_z6>0:
            z_6 = z_6/norm_z6
            vec = np.cross(z_axis,z_6)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_axis,z_6))
                vec = vec/np.linalg.norm(vec)
            else:
                if (z_axis==z_6).all():
                    ang = 0
                else:
                    ang = np.pi   
                vec = [1,0,0]   
        else:
            z_6 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q6 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_e2 = quaternion_multiply(quaternion_multiply(q6,np.concatenate(([0],z_axis))),quat_inv(q6))
        #print('z6:{}'.format(z_6))
        #print('q6:{}'.format(z_e2[1:]))

        #align the elbow z axis with the elbow2->wrist2 vector
        #align the wrist y axis with the cross product of the elbow2->wrist2 vector and the shoulder2->elbow2 vector
        norm_z7 = np.linalg.norm(vects[:,7])
        if norm_z7>0:
            z_7 = vects[:,7]/norm_z7
            vec = np.cross(z_axis,z_7)
            if np.linalg.norm(vec)>0:
                ang = np.arctan2(np.linalg.norm(vec),np.dot(z_axis,z_7))
                vec = vec/np.linalg.norm(vec)
            else:
                if (z_axis==z_7).all():
                    ang = 0
                else:
                    ang = np.pi
                vec = [1,0,0]  
        else:
            z_7 = np.array([0,0,0])
            vec = [0,0,0]
            ang = 0
        q7 = np.array([np.cos(ang/2),np.sin(ang/2)*vec[0],np.sin(ang/2)*vec[1],np.sin(ang/2)*vec[2]])
        z_w2 = quaternion_multiply(quaternion_multiply(q7,np.concatenate(([0],z_axis))),quat_inv(q7)) 
        #print('z7:{}'.format(z_7))
        #print('q7:{}'.format(z_w2[1:])) 

        return success,np.concatenate((q1,q2,q3,q4,q5,q6,q7))

    #function to compute the spine, shoulder, elbow, wrist, and head top locations
    #given the rpy at each joint.  The coordinates of the pelvis must be given since
    #other transformations start from that location.  
    #dimension[0] = spine length, [1]=neck/head length, [2] = shoulder to shoulder length
    #[3] = L upper arm length, [4] = L forearm length, [5]/[6] = R upper arm/forearm length
    def forwardKinematics(self,quats,pelvis_coords,dimension):
        points = np.zeros((3,15))
        points[:,0] = pelvis_coords
        q1 = quats[:4]
        z_spine = quaternion_multiply(quaternion_multiply(q1,np.array([0,0,0,1])),quat_inv(q1))
        points[:,1] = points[:,0]+dimension[0]*np.array(z_spine[1:]).reshape((3))
        q2 = quats[4:8]
        z_neck = quaternion_multiply(quaternion_multiply(q2,z_spine),quat_inv(q2))
        points[:,2] = points[:,1]+dimension[1]*np.array(z_neck[1:]).reshape((3))
        q3 = quats[8:12]
        z_shoulders = quaternion_multiply(quaternion_multiply(q3,z_spine),quat_inv(q3))
        points[:,3] = points[:,1]-0.5*dimension[2]*np.array(z_shoulders[1:]).reshape((3))
        q4 = quats[12:16]
        z_e1 = quaternion_multiply(quaternion_multiply(q4,z_shoulders),quat_inv(q4))
        points[:,4] = points[:,3]+dimension[3]*np.array(z_e1[1:]).reshape((3))
        q5 = quats[16:20]
        z_w1 = quaternion_multiply(quaternion_multiply(q5,z_e1),quat_inv(q5))
        points[:,5] = points[:,4]+(dimension[4]+0.1)*np.array(z_w1[1:]).reshape((3))
        points[:,6] = points[:,1]+0.5*dimension[2]*np.array(z_shoulders[1:]).reshape((3))
        q6 = quats[20:24]
        z_e2 = quaternion_multiply(quaternion_multiply(q6,z_shoulders),quat_inv(q6))
        points[:,7] = points[:,6]+dimension[5]*np.array(z_e2[1:]).reshape((3))
        q7 = quats[24:]
        z_w2 = quaternion_multiply(quaternion_multiply(q7,z_e2),quat_inv(q7))
        points[:,8] = points[:,7]+(dimension[6]+0.1)*np.array(z_w2[1:]).reshape((3))

        return points

    #function to compute the spine, shoulder, elbow, wrist, and head top locations
    #given the rpy at each joint.  The coordinates of the pelvis must be given since
    #other transformations start from that location.  
    #dimension[0] = spine length, [1]=neck/head length, [2] = shoulder to shoulder length
    #[3] = L upper arm length, [4] = L forearm length, [5]/[6] = R upper arm/forearm length
    def forwardKinematics_fixed(self,quats,pelvis_coords,dimension):
        z_axis_quat = np.array([0,0,0,1])
        points = np.zeros((3,15))
        points[:,0] = pelvis_coords
        q1 = quats[:4]
        z_spine = quaternion_multiply(quaternion_multiply(q1,np.array([0,0,0,1])),quat_inv(q1))
        points[:,1] = points[:,0]+dimension[0]*np.array(z_spine[1:]).reshape((3))
        q2 = quats[4:8]
        z_neck = quaternion_multiply(quaternion_multiply(q2,z_axis_quat),quat_inv(q2))
        points[:,2] = points[:,1]+dimension[1]*np.array(z_neck[1:]).reshape((3))
        q3 = quats[8:12]
        z_shoulders = quaternion_multiply(quaternion_multiply(q3,z_axis_quat),quat_inv(q3))
        points[:,3] = points[:,1]-0.5*dimension[2]*np.array(z_shoulders[1:]).reshape((3))
        q4 = quats[12:16]
        z_e1 = quaternion_multiply(quaternion_multiply(q4,z_axis_quat),quat_inv(q4))
        points[:,4] = points[:,3]+dimension[3]*np.array(z_e1[1:]).reshape((3))
        q5 = quats[16:20]
        z_w1 = quaternion_multiply(quaternion_multiply(q5,z_axis_quat),quat_inv(q5))
        points[:,5] = points[:,4]+(dimension[4]+0.1)*np.array(z_w1[1:]).reshape((3))
        points[:,6] = points[:,1]+0.5*dimension[2]*np.array(z_shoulders[1:]).reshape((3))
        q6 = quats[20:24]
        z_e2 = quaternion_multiply(quaternion_multiply(q6,z_axis_quat),quat_inv(q6))
        points[:,7] = points[:,6]+dimension[5]*np.array(z_e2[1:]).reshape((3))
        q7 = quats[24:]
        z_w2 = quaternion_multiply(quaternion_multiply(q7,z_axis_quat),quat_inv(q7))
        points[:,8] = points[:,7]+(dimension[6]+0.1)*np.array(z_w2[1:]).reshape((3))
        return points