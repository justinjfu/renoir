#!/usr/bin/env python
"""
Driver script
"""
import rospy
import draw_path
from robot_state import ROBOT_STATE
from vectorize import filtered_segments, read_image, LIFT

def get_waypoints(img):
    waypoints = []
    for seg in filtered_segments(img, {'scan_range':2}):
        if seg is LIFT:
            yield waypoints
            waypoints = []
        else:
            waypoints.append(seg)

def picture_plan(waypoints_list):
    draw_path.bring_up()
    for waypts in waypoints_list:
        draw_path.bring_down(waypts[0])
        draw_path.draw_waypoints(waypts)
        draw_path.bring_up()

def main():
    rospy.init_node('drawpath_node')
    ROBOT_STATE.init()

    img = read_image('cartman.jpg', show=True, thresh=120)
    waypoints = get_waypoints(img)
    picture_plan(waypoints)
    #for seg in get_waypoints(img):
    #    print seg

if __name__ == "__main__":
    main()
