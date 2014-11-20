#!/usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from ar_track_alvar.msg import AlvarMarkers

FIXED_MATRIX = np.array([[1,0,0],[0,1,0],[0,0,1]])

TOPIC_NAME = 'renoir/pic2world'
POSE_TOPIC = '/ar_pose_marker'

def ar_callback(pub, msg):
    markers = msg.markers
    timestamp = msg.header.stamp

    for marker in markers:
        marker_id = marker.id
        pose = marker.pose.pose 
        pos = pose.position
        ori = pose.orientation
        pos = [pos.x, pos.y, pos.z]
        ori = [ori.x, ori.y, ori.z, ori.w]
        print '---'; print marker_id; print pos; print ori
        # Assume these are relative to the baxter base frame
        projection = np.eye(3, dtype=np.float32)

        # Compute projection. Have the position and orientation of upper
        # left corner

        #Compress to a 3x3 array because you can only publish 1D arrays
        to_publish = projection.reshape(9)
        pub.publish(to_publish)

def init_subscribers(matrix_pub):
    ar_sub = rospy.Subscriber(POSE_TOPIC, AlvarMarkers, 
        lambda x: ar_callback(matrix_pub, x))

def main():
    rospy.init_node('pic2world_node')
    pub = rospy.Publisher(TOPIC_NAME, numpy_msg(Floats))
    init_subscribers(pub)

    r = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == "__main__":
    main()
