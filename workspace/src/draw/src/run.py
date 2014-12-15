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

    #world_point = [0.536, 0.245, -0.100]
    origin = draw_path.pic2world([0,0])
    point2 = draw_path.pic2world([50,0])
    point3 = draw_path.pic2world([0,50])


    draw_path.bring_down_world(origin)
    draw_path.draw_line([origin, point2])
    draw_path.draw_line([point2, point3])
    draw_path.draw_line([point3, origin])
    draw_path.bring_up()

    # while not rospy.is_shutdown():
    #     r.sleep()

if __name__ == "__main__":
    main()
