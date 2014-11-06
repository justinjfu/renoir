#!/usr/bin/env python
"""
Driver script
"""
import rospy
import draw_path

def main():
    rospy.init_node('renoir/draw')

    r = rospy.Rate(10) #10Hz

    while not rospy.is_shutdown():
        r.sleep()

if __name__ == "__main__":
    main()
