#!/usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from ar_track_alvar.msg import AlvarMarkers

FIXED_MATRIX = np.array([[1,0],[0,1],[0,0]])

TOPIC_NAME = 'renoir/pic2world'
POSE_TOPIC = '/ar_pose_marker'

def ar_callback(msg):
    markers = msg.markers
    timestamp = msg.header.stamp

    for marker in markers:
        print marker.id
        pose = marker.pose #geometry_msgs/PoseStamped
    #TODO(Justin) Update pic2world matrix

def init_subscribers():
    ar_sub = rospy.Subscriber(POSE_TOPIC, AlvarMarkers, ar_callback)

def main():
    rospy.init_node('pic2world_node')
    pub = rospy.Publisher(TOPIC_NAME, numpy_msg(Floats))
    init_subscribers()

    r = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        pub.publish(FIXED_MATRIX)
        r.sleep()

if __name__ == "__main__":
    main()
