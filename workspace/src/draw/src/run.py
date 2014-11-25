#!/usr/bin/env python
"""
Driver script
"""
import rospy
import draw_path
from robot_state import ROBOT_STATE

def main():
    rospy.init_node('drawpath_node')
    ROBOT_STATE.init()

    # r = rospy.Rate(10) #10Hz

    world_point = [0.536, 0.245, -0.180]
    draw_path.bring_down_world(world_point)
    draw_path.bring_up()

    # while not rospy.is_shutdown():
    #     r.sleep()

if __name__ == "__main__":
    main()
