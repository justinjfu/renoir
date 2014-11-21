#!/usr/bin/env python
import rospy
import numpy as np
import tf
import sys
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from ar_track_alvar.msg import AlvarMarkers

FIXED_MATRIX = np.array([[1,0,0],[0,1,0],[0,0,1]])

X_AXIS = np.array([1,0,0,0])
Y_AXIS = np.array([0,1,0,0])
Z_AXIS = np.array([0,0,1,0])
ORIGIN = np.array([0,0,0,1])

TOPIC_NAME = 'renoir/pic2world'
POSE_TOPIC = '/ar_pose_marker'
BAXTER_BASE_DEFAULT_FRAME = '/usb_cam'

matrix_pub = None
tf_listener = None

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

def calculate_tf_matrix(output_frame):
    try:
        trans, rot = tf_listener.lookupTransform('/ar_pose_marker_0', output_frame, rospy.Time(0))
        matrix = tf_listener.fromTranslationRotation(trans, rot)
        xax = matrix.dot(X_AXIS)
        yax = matrix.dot(Y_AXIS)
        offset = matrix.dot(ORIGIN)

        proj = np.eye(3, dtype=np.float32)
        # Compute projection. Have the position and orientation of upper
        # left corner
        X_PAPER_SCALE = 11.5/100
        Y_PAPER_SCALE = 8/100
        proj[:,0] = xax[0:3]*X_PAPER_SCALE
        proj[:,1] = yax[0:3]*Y_PAPER_SCALE
        proj[:,2] = offset[0:3]

        #Compress to a 3x3 array because you can only publish 1D arrays
        to_publish = proj.reshape(9)
        pub.publish(to_publish)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue


def main(args):
    global matrix_pub, tf_listener
    if len(args) == 1:
        output_frame = args[0]
    else:
        output_frame = BAXTER_BASE_DEFAULT_FRAME
    rospy.init_node('pic2world_node')
    matrix_pub = rospy.Publisher(TOPIC_NAME, numpy_msg(Floats))
    tf_listener = listener = tf.TransformListener()
    #init_subscribers(pub)

    r = rospy.Rate(5) #5Hz
    while not rospy.is_shutdown():
        calculate_tf_matrix(output_frame)
        r.sleep()

if __name__ == "__main__":
    main(sys.argv)
