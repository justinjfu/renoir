#!/usr/bin/env python
"""
Driver script
 """
import rospy
import draw_path
from robot_state import ROBOT_STATE
from vectorize import filtered_segments, read_image, LIFT
import matplotlib.pyplot as plt
import numpy as np

def get_waypoints(img, params):
    waypoints = []
    first_point = True
    for seg in filtered_segments(img, params):
        if seg is LIFT:
            yield waypoints
            waypoints = []
            first_point = True
        else:
            if first_point:
              waypoints.append(seg[0])   
              waypoints.append(seg[1])
              first_point=False
            else:
              waypoints.append(seg[1])

def picture_plan(waypoints_list):
    draw_path.bring_down(waypoints_list[0])
    draw_path.draw_waypoints(waypoints_list)
    draw_path.bring_up()


POKE = 'pokeball.png'
CART = 'cartman.jpg'
CAL = 'cal.jpg'
TREE = 'tree.jpg'
LETA = 'a.jpg'
PARAMS = {
  POKE: {'scan_range':2, 'K':0.0, 'thresh':150},
  CART: {'scan_range':3, 'K':1.0, 'thresh':130},
  CAL: {'scan_range':1, 'K':0.0, 'thresh':150},
  TREE: {'scan_range':4, 'K':0.0, 'thresh':150},
  LETA: {'scan_range':5, 'K':0.0, 'thresh':150}
}
def main():
    rospy.init_node('drawpath_node')
    ROBOT_STATE.init()
    img_id = LETA

    img = read_image(img_id, show=True, thresh=PARAMS[img_id]['thresh'])
    img_cpy = np.copy(img)
    waypoints = get_waypoints(img, PARAMS[img_id])
    n = 0
    for wp in waypoints:
        print n,":",wp
        n+=1
        for pt in wp:
            img_cpy[pt[0],pt[1]]=0
        plt.imshow(img_cpy, interpolation='nearest', cmap='Greys')    
        plt.show()
        picture_plan(wp)
        print "here1"


if __name__ == "__main__":
    main()
