#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker
import rosbag

import rosnode
from tf.transformations import quaternion_matrix


if __name__=='__main__':
    rospy.init_node('play_bag',anonymous=True)
    pubs = {}
    topics = ['/camera/keypoints_filtered','/camera2/keypoints_filtered']
    for topic in topics:
        pubs[topic] = rospy.Publisher(topic,Marker,queue_size=1)

    bag = rosbag.Bag(rospy.get_param('/bag_file'), 'r')
    top_start_t = {}
    start_t = rospy.Time.now()
    for topic, msg, t in bag.read_messages(topics=topics):
        if rospy.is_shutdown():
            break
        print((rospy.Time.now()-start_t).to_sec())
        if not topic in top_start_t.keys():
            top_start_t[topic] = t
        duration_diff = (t-top_start_t[topic]).to_sec()-(rospy.Time.now()-start_t).to_sec()
        print(duration_diff)
        if duration_diff>0:
            rospy.sleep(duration_diff)
        msg.header.stamp = rospy.Time.now()
        pubs[topic].publish(msg)
        
    bag.close()

    