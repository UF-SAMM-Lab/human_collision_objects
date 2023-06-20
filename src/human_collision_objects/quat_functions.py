#!/usr/bin/env python
import rospy
import numpy as np
import math

def quaternion_multiply(Q0,Q1):
    """
    Multiplies two quaternions.
 
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
 
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
 
    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
     
    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
    
    s1 = Q0[0]
    v1 = Q0[1:].reshape((3))
    
    s2 = Q1[0]
    v2 = Q1[1:].reshape((3))
    '''
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    '''
    Q0Q1_s = s1*s2-np.dot(v1,v2)
    Q0Q1_v = s1*v2+s2*v1+np.cross(v1,v2)
    
    # Create a 4 element array containing the final quaternion
    #final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
    final_quaternion = np.concatenate((np.array([Q0Q1_s]).reshape((1)), Q0Q1_v))
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion.reshape((4))

def quat_inv(q):
    return np.array([q[0],-q[1],-q[2],-q[3]])/np.linalg.norm(q)**2

def quat_to_ang_vec(q,prev_vec):
    ang = 2*math.acos(q[0])
    if not ang==0:
        vec = q[1:]/math.sin(ang/2)
    else:
        principal_axis = np.array([[1,0,0],[0,1,0],[0,0,1]]) #each row is either x y or z
        nearest_axis = np.argmin(np.linalg.norm(np.tile(prev_vec.reshape((1,3)),(3,1))-principal_axis,axis=0)) #get the prinicipal axis that is nearest to the previous rotation axis
        vec = principal_axis[nearest_axis,:] #set rotation axis = to nearest principal axis (x y or z)
    return np.append(np.array([ang]),vec)