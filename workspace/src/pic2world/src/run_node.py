#!/usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

FIXED_MATRIX = np.array([[1,0],[0,1],[0,0]])

TOPIC_NAME = 'renoir/pic2world'

def main():
    rospy.init_node('renoir/pic2world_node')
    pub = rospy.Publisher(TOPIC_NAME, numpy_msg(Floats))

    r = rospy.Rate(10) #10Hz

    while not rospy.is_shutdown():
        pub.publish(FIXED_MATRIX)
        r.sleep()

if __name__ == "__main__":
    main()
