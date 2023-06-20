#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int16
from visualization_msgs.msg import Marker
import time
import rospkg
import os
from human_collision_objects.filter_functions import *
from human_collision_objects.quat_functions import *

import rosnode
from tf.transformations import quaternion_matrix

rospack = rospkg.RosPack()

this_pack = rospack.get_path('human_collision_objects')

joint_pos = np.zeros((29,3))
world_to_c2 = np.eye(4)
world_to_c1 = np.eye(4)
use_camera_mode = rospy.get_param('/use_camera_mode')
left_cam_idx = [0,1,2,3,7,9,11,13,15,17,19,21,23,25,27]
right_cam_idx = [4,5,6,8,10,12,14,16,18,20,22,24,26,28]

tasks = []
past_observations = None
t_since_start = 0.0
start_time = None
rec_rate = 1/10
cur_observation = None
num_features = None
pub_prediction = None
angle_filter = human_filter()
time_of_last_skeleton = 0.0
time_since_last_skeleton = 0.0
combined_joint_locations = None
new_data = False
last_step = -1
prev_time = None
timer_idx = 0
observation_id=0
prev_obs_id = 0
prev_scale_factor =0.0
motion_onset = False
num_step_inc=0

pause_data_timer = True

num_threads = 8

num_recordings = 0

prev_best_task = 0
prev_t_step =0
first_motion = True
start_t= 0
node_list = rosnode.get_node_names()
print(node_list)

joint_vels = np.zeros((18,10))
j_v_idx = 0
max_avg_vel=0
start_task = False
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
is_new_task = False
stop_record = False
sharework = False
human_dimensions_param  = None
pub_quats = None
pub_skel = None    
pub_dims = None
joint_names = ['      head',
               ' spine-top',
               'r shoulder',
               '   r elbow',
               '   r wrist',
               'l shoulder',
               '   l elbow',
               '   l wrist',
               '   r   hip',
               '   l   hip']

def skeleton1_callback(data):
    global world_to_c1,use_camera_mode,right_cam_idx
    if world_to_c1 is None:
        return
    idx = data.id
    if (idx>28): 
        return
    if (idx not in right_cam_idx) and (use_camera_mode=='both'):
        return
    pt = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z,1]).reshape((4,1))
    pt = world_to_c1@pt
    joint_pos[idx,:3] = pt[:3].flatten()

def skeleton2_callback(data):
    global world_to_c2,use_camera_mode,left_cam_idx
    if world_to_c2 is None:
        return
    idx = data.id
    if (idx>28): 
        return
    if (idx not in left_cam_idx) and (use_camera_mode=='both'):
        return
    pt = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z,1]).reshape((4,1))
    pt = world_to_c2@pt
    joint_pos[idx,:3] = pt[:3].flatten()

def convert_quats_to_angvec(quats,prev_ang_vecs):
    angs_vecs = np.empty((0))
    for i in range(int(len(quats)/4)):
        angs_vecs = np.append(angs_vecs,quat_to_ang_vec(quats[i*4:(i+1)*4,0],prev_ang_vecs[i*4+1:(i+1)*4]))
    return angs_vecs

def skeleton_callback(data):
    global sharework, pub_quats, time_of_last_skeleton, angle_filter, time_since_last_skeleton
    global joint_names
    #convert points to quat representation


    #head,spine,shoulder,elbow,hand,shoulder,elbow,hand,hip,knee,ankle,hip,knee,ankle,eye,eye,ear,ear
    #ggl0,     ,11      ,13   ,17-19,12     ,14   ,18-20,23,25  ,27   ,24 ,26  ,28   ,2  ,5  ,7  ,8 
    #op 0,1    ,2       ,3    ,4    ,5      ,6    ,7   ,8  ,9   ,10   ,11 ,12  ,13   ,14 ,15 ,16 ,17
    google_joint_idx = [0,11,12,14,16,11,13,15,24,26,28,23,25,27,5,2,8,7]

    # pt_data = np.array(data.data)

    tmp_calibrate_dimensions = True
    if np.count_nonzero(angle_filter.dims_avg)==0:
        angle_filter.dimensions_ratios = np.empty((7,0))
        angle_filter.dimensions_matrix = np.empty((7,0))
        angle_filter.dimension_ratios_dev = 100*np.ones((7))
        angle_filter.calibrate_dimensions = True
        tmp_calibrate_dimensions = False

    # if (time.time()-time_of_last_skeleton>time_since_last_skeleton):
    time_since_last_skeleton = time.time()-time_of_last_skeleton

    time_of_last_skeleton = time.time()
    
    all_joints = np.array(data.data).reshape((29,3)).copy()
    joint_location = all_joints[google_joint_idx,:3]
    joint_location[1,:] = 0.5*(joint_location[2,:3]+joint_location[5,:3])
    joints_to_print = np.array([0,1,2,3,4,5,6,7,8,11])
    print_str = ''
    for i in range(joints_to_print.shape[0]):
        print_str += '{}:{}'.format(joint_names[i],joint_location[joints_to_print[i],:])
        print_str += '\n'
    print(print_str)

    current_joint_location = angle_filter.parseCoordsIntel(joint_location)

    tmp = angle_filter.filter_input(current_joint_location)

    if angle_filter.calibrate_dimensions and angle_filter.dims_ready:
        angle_filter.dimension_ratios = np.append(angle_filter.dimension_ratios,angle_filter.dims_ratios.reshape((7,1)),axis = 1)
        angle_filter.dimensions_matrix = np.append(angle_filter.dimensions_matrix,angle_filter.dims_avg.reshape((7,1)),axis=1)
        angle_filter.human_dimensions = np.mean(angle_filter.dimensions_matrix,axis=1)
        #print(cam.dimension_ratios.shape[1])
        if angle_filter.dimension_ratios.shape[1]>50:
            angle_filter.dimension_ratios = np.delete(angle_filter.dimension_ratios,0,1)
            angle_filter.dimensions_matrix = np.delete(angle_filter.dimensions_matrix,0,1)
            angle_filter.dimension_ratios_dev = np.std(angle_filter.dimension_ratios,axis=1)
            #print(cam.dimension_ratios_dev)
            # print('dim ratios {}'.format(angle_filter.dimension_ratios[:,1:]))
            if np.sum(angle_filter.dimension_ratios_dev)<0.05 and np.all(angle_filter.dimension_ratios[1:,:]<1):
                angle_filter.calibrate_dimensions = False
                print('human calibrated:{}'.format(angle_filter.human_dimensions))
    
    angs_vecs = convert_quats_to_angvec(angle_filter.prevTheta,angle_filter.angs_vecs)
    # angle_filter.ang_change_rate,angle_filter.vec_change_rate = compute_angvec_change_rate(angs_vecs,angle_filter.angs_vecs,dt)
    angle_filter.angs_vecs = angs_vecs
        
    angle_filter.data_ready = (angle_filter.dims_arr.shape[1]>0) and (time.time()-time_of_last_skeleton<1.0)
    tmp_observation = angle_filter.prevTheta[:,0].copy()

    if not angle_filter.calibrate_dimensions:
        combined_human_dimensions = angle_filter.human_dimensions
    else:
        combined_human_dimensions = human_dimensions_param
    # print('{}, {}'.format(joint_location[8,:],joint_location[11,:]))
    pelvis_location = 0.5*(joint_location[8,:]+joint_location[11,:]) #pelvis (hips midpoint)
    cur_observation = np.array([0])
    cur_observation = np.append(cur_observation,pelvis_location)
    cur_observation = np.append(cur_observation,tmp_observation)
    quats_msg = Float32MultiArray()
    quats_msg.data = cur_observation
    pts_msg = Float32MultiArray()
    pts_msg.data = current_joint_location.T.reshape((-1)).astype('float')
    dims_msg = Float32MultiArray()
    dims_msg.data = combined_human_dimensions.astype('float')
    if pub_quats:
        pub_quats.publish(quats_msg)
    if pub_skel:
        pub_skel.publish(pts_msg)
    if pub_dims:
        pub_dims.publish(dims_msg)


if __name__=='__main__':
    rospy.init_node('sharework_tracking',anonymous=True)
    
    world_to_c1_xyzrpw = np.asarray(rospy.get_param('/sharework_tracking/world_to_c1').split(','),dtype=float)
    c1_to_c1_xyzrpw = np.asarray(rospy.get_param('/sharework_tracking/c1_c1_color').split(','),dtype=float)
    world_to_c2_xyzrpw = np.asarray(rospy.get_param('/sharework_tracking/world_to_c2').split(','),dtype=float)
    c2_to_c2_xyzrpw = np.asarray(rospy.get_param('/sharework_tracking/c2_c2_color').split(','),dtype=float)
    
    world_to_c1 = quaternion_matrix(world_to_c1_xyzrpw[3:])
    world_to_c1[:3,3] = np.array(world_to_c1_xyzrpw[:3])

    c1_c1 = quaternion_matrix(c1_to_c1_xyzrpw[3:])
    c1_c1[:3,3] = np.array(c1_to_c1_xyzrpw[:3]) 

    world_to_c1 = world_to_c1@c1_c1

    world_to_c2 = quaternion_matrix(world_to_c2_xyzrpw[3:])
    world_to_c2[:3,3] = np.array(world_to_c2_xyzrpw[:3])

    c2_c2 = quaternion_matrix(c2_to_c2_xyzrpw[3:])
    c2_c2[:3,3] = np.array(c2_to_c2_xyzrpw[:3]) 

    world_to_c2 = world_to_c2@c2_c2

    # world_to_c2[:3,:3] = world_to_c2[:3,:3].T
    # world_to_c2[:3,3] = -world_to_c2[:3,:3]@world_to_c2[:3,3]
    # rospy.Subscriber('/skeleton_points', Float32MultiArray, skeleton_callback)
    right_cam_sub = None
    left_cam_sub = None
    if (use_camera_mode=='right') or (use_camera_mode=='both'):
        right_cam_sub = rospy.Subscriber('/camera/keypoints_filtered', Marker, skeleton1_callback)
    if (use_camera_mode=='left') or (use_camera_mode=='both'):
        left_cam_sub = rospy.Subscriber('/camera2/keypoints_filtered', Marker, skeleton2_callback)

    pub_joints = rospy.Publisher('/skeleton_points', Float32MultiArray, queue_size=1)
    pub_skel = rospy.Publisher('/skeleton', Float32MultiArray, queue_size=1)
    pub_quats = rospy.Publisher('/skeleton_quats', Float32MultiArray, queue_size=1)
    human_dimensions_param = np.asarray(rospy.get_param("/human_link_lengths"),dtype=float)
    print('human dimensions from param {}'.format(human_dimensions_param))
    pub_dims = rospy.Publisher('/human_0/dimensions',Float32MultiArray,queue_size=1)

    joint_msg = Float32MultiArray()

    while (not rospy.is_shutdown()):
        cams_ready = True
        if (use_camera_mode=='right') or (use_camera_mode=='both'):
            if right_cam_sub.get_num_connections()<1:
                rospy.logerr_throttle(5, "/camera/keypoints_filtered is not being published!")
                cams_ready = False
        if (use_camera_mode=='left') or (use_camera_mode=='both'):
            if left_cam_sub.get_num_connections()<1:
                rospy.logerr_throttle(5, "/camera2/keypoints_filtered is not being published!")
                cams_ready = False
        if cams_ready:
            joint_msg.data = joint_pos.flatten()
            pub_joints.publish(joint_msg)
            skeleton_callback(joint_msg)
        rospy.sleep(0.01)

    